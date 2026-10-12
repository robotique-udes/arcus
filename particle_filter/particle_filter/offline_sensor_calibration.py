#!/usr/bin/env python3

import math
from collections import defaultdict
from math import erf

import numpy as np
import range_libc
import tf_transformations
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import LaserScan

BAG_PATH = "/path/to/your/rosbag"
SCAN_TOPIC = "/scan"
MAP_TOPIC = "/map"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
MAP_FRAME = "map"
LASER_FRAME = "ego_racecar/laser"
ROBOT_BASE_FRAME = "ego_racecar/base_link"
MAX_RANGE = 30.0
HIT_THRESHOLD = 0.5
MAX_ITERATIONS = 1000


class BagReader:
    def __init__(self, bag_path):
        self.reader = SequentialReader()
        storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
        converter_options = ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        self.reader.open(storage_options, converter_options)
        self.topic_types = {}
        for topic_info in self.reader.get_metadata().topics_with_message_types:
            self.topic_types[topic_info.name] = topic_info.type

    def iter_messages(self):
        while self.reader.has_next():
            topic, data, _ = self.reader.read_next()
            yield topic, data


class TFStore:
    def __init__(self):
        self.transforms = defaultdict(list)

    def add_tf_message(self, msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            key = (parent, child)
            stamp_ns = stamp_to_ns(transform.header.stamp)
            self.transforms[key].append((stamp_ns, transform))

    def lookup_transform(self, parent, child, stamp_ns):
        key = (parent, child)
        entries = self.transforms.get(key, [])
        if not entries:
            return None

        best = None
        for t_ns, transform in entries:
            if t_ns <= stamp_ns:
                if best is None or t_ns > best[0]:
                    best = (t_ns, transform)
        return best[1] if best is not None else None


def stamp_to_ns(stamp):
    return int(stamp.sec * 1_000_000_000 + stamp.nanosec)


def quaternion_to_matrix(qx, qy, qz, qw):
    return tf_transformations.quaternion_matrix([qx, qy, qz, qw])


def occupancy_grid_to_array(msg):
    grid = np.asarray(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
    return grid


def transform_point_by_tf(point, tf_msg):
    # point: (x, y, z) in source frame
    # tf_msg: geometry_msgs/TransformStamped from source -> target
    rot = quaternion_to_matrix(
        tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y,
        tf_msg.transform.rotation.z,
        tf_msg.transform.rotation.w,
    )
    x, y, z = point
    p = np.array([x, y, z, 1.0])
    out = rot @ p
    return (
        out[0] + tf_msg.transform.translation.x,
        out[1] + tf_msg.transform.translation.y,
        out[2] + tf_msg.transform.translation.z,
    )


def compute_expected_ranges_from_map(scan_msg, map_msg, tf_store, map_frame, laser_frame, max_range):
    if map_msg is None:
        return None

    stamp_ns = stamp_to_ns(scan_msg.header.stamp)
    tf_laser_to_map = tf_store.lookup_transform(map_frame, laser_frame, stamp_ns)
    if tf_laser_to_map is None:
        return None

    tx = tf_laser_to_map.transform.translation.x
    ty = tf_laser_to_map.transform.translation.y
    q = tf_laser_to_map.transform.rotation
    yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    n_beams = len(scan_msg.ranges)
    queries = np.zeros((n_beams, 3), dtype=np.float32)
    queries[:, 0] = tx
    queries[:, 1] = ty
    queries[:, 2] = yaw + np.linspace(
        scan_msg.angle_min,
        scan_msg.angle_min + (n_beams - 1) * scan_msg.angle_increment,
        n_beams,
        dtype=np.float32,
    )

    max_range_px = int(max_range / map_msg.info.resolution)
    o_map = range_libc.PyOMap(map_msg)
    range_method = range_libc.PyRayMarching(o_map, max_range_px)
    ranges_px = np.zeros(n_beams, dtype=np.float32)
    range_method.calc_range_many(queries, ranges_px)

    expected = np.asarray(ranges_px, dtype=np.float32) * map_msg.info.resolution
    return [float(min(v, max_range)) for v in expected]


def estimate_sensor_model(z, z_star, max_range, init=None,
                          max_iter=MAX_ITERATIONS, tol=1e-8):
    z = np.asarray(z, dtype=float)
    z_star = np.asarray(z_star, dtype=float)

    z = np.where(np.isposinf(z), max_range, z)
    ok = np.isfinite(z) & np.isfinite(z_star)
    z, z_star = np.clip(z[ok], 0.0, max_range), z_star[ok]
    n = len(z)

    p = dict(z_hit=0.7, z_short=0.1, z_max=0.1, z_rand=0.1,
             sigma_hit=0.3, lambda_short=1.0)
    if init:
        p.update(init)

    eps = 1e-12
    max_mask = z >= max_range - 1e-6
    short_mask = (z < z_star) & ~max_mask
    p_max = max_mask.astype(float)
    p_rand = np.full(n, 1.0 / max_range)

    prev_ll = -np.inf
    for _ in range(max_iter):
        s, lam = p["sigma_hit"], p["lambda_short"]

        # component densities
        cdf = lambda x: 0.5 * (1.0 + erf(x / np.sqrt(2.0)))
        norm = cdf((max_range - z_star) / s) - cdf(-z_star / s)
        p_hit = np.exp(-0.5 * ((z - z_star) / s) ** 2) / (np.sqrt(2 * np.pi) * s)
        p_hit /= np.maximum(norm, eps)

        denom = -np.expm1(-lam * z_star)
        p_short = np.zeros(n)
        v = short_mask & (denom > eps)
        p_short[v] = lam * np.exp(-lam * z[v]) / denom[v]

        # E-step
        w_hit = p["z_hit"] * p_hit
        w_short = p["z_short"] * p_short
        w_max = p["z_max"] * p_max
        w_rand = p["z_rand"] * p_rand
        total = np.maximum(w_hit + w_short + w_max + w_rand, eps)
        e_hit, e_short = w_hit / total, w_short / total
        e_max, e_rand = w_max / total, w_rand / total

        # M-step
        p["z_hit"], p["z_short"] = e_hit.sum() / n, e_short.sum() / n
        p["z_max"], p["z_rand"] = e_max.sum() / n, e_rand.sum() / n
        p["sigma_hit"] = max(np.sqrt(np.sum(e_hit * (z - z_star) ** 2)
                                     / max(e_hit.sum(), eps)), 1e-3)
        p["lambda_short"] = e_short.sum() / max(np.sum(e_short * z), eps)

        ll = np.sum(np.log(total))
        if abs(ll - prev_ll) < tol * abs(ll):
            break
        prev_ll = ll

    return p


def main():
    bag = BagReader(BAG_PATH)
    tf_store = TFStore()
    map_msg = None

    accumulated_ranges = []
    accumulated_expected = []

    for topic, data in bag.iter_messages():
        if topic == MAP_TOPIC:
            map_msg = deserialize_message(data, get_message(bag.topic_types[topic]))
            continue

        if topic == TF_TOPIC:
            tf_msg = deserialize_message(data, get_message(bag.topic_types[topic]))
            tf_store.add_tf_message(tf_msg)
            continue

        if topic == TF_STATIC_TOPIC:
            tf_msg = deserialize_message(data, get_message(bag.topic_types[topic]))
            tf_store.add_tf_message(tf_msg)
            continue

        if topic == SCAN_TOPIC:
            scan_msg = deserialize_message(data, get_message(bag.topic_types[topic]))
            if not isinstance(scan_msg, LaserScan):
                continue

            expected_ranges = compute_expected_ranges_from_map(
                scan_msg,
                map_msg,
                tf_store,
                MAP_FRAME,
                LASER_FRAME,
                MAX_RANGE,
            )
            if expected_ranges is None:
                continue

            accumulated_ranges.append(scan_msg.ranges)
            accumulated_expected.append(expected_ranges)

    if not accumulated_ranges:
        raise RuntimeError("No LaserScan messages were found for the provided scan topic.")

    flat_ranges = []
    flat_expected = []
    for ranges, expected in zip(accumulated_ranges, accumulated_expected):
        for z, z_star in zip(ranges, expected):
            flat_ranges.append(float(z))
            flat_expected.append(float(z_star))

    estimate = estimate_sensor_model(
        flat_ranges,
        flat_expected,
        MAX_RANGE,
        init={"z_hit": 0.75, "z_short": 0.1, "z_max": 0.05, "z_rand": 0.1, "sigma_hit": 0.3, "lambda_short": 1.0},
    )

    print("Estimated sensor model parameters:")
    print(f"  z_hit    = {estimate['z_hit']:.6f}")
    print(f"  z_short  = {estimate['z_short']:.6f}")
    print(f"  z_max    = {estimate['z_max']:.6f}")
    print(f"  z_rand   = {estimate['z_rand']:.6f}")
    print(f"  sigma_hit= {estimate['sigma_hit']:.6f}")



if __name__ == "__main__":
    main()
