#!/usr/bin/env python3

import math
from collections import defaultdict
from math import erf

import numpy as np
import range_libc
import transforms3d.euler
import transforms3d.quaternions
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import LaserScan

BAG_PATH = "/sim_ws/src/arcus/resources/rosbag2_2026_10_14-00_33_28"
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
SCAN_BEAM_STRIDE = 4
MAX_SCAN_MESSAGES_TO_USE = 500


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
        for topic_info in self.reader.get_metadata().topics_with_message_count:
            self.topic_types[topic_info.topic_metadata.name] = topic_info.topic_metadata.type

    def iter_messages(self):
        while self.reader.has_next():
            topic, data, _ = self.reader.read_next()
            yield topic, data


class TFStore:
    def __init__(self):
        self.transforms = defaultdict(list)

    def add_tf_message(self, msg):
        for transform in msg.transforms:
            parent = normalize_frame_name(transform.header.frame_id)
            child = normalize_frame_name(transform.child_frame_id)
            key = (parent, child)
            stamp_ns = stamp_to_ns(transform.header.stamp)
            self.transforms[key].append((stamp_ns, transform))

    def lookup_transform(self, parent, child, stamp_ns):
        key = (normalize_frame_name(parent), normalize_frame_name(child))
        entries = self.transforms.get(key, [])
        if not entries:
            return None

        best = None
        for t_ns, transform in entries:
            if t_ns <= stamp_ns:
                if best is None or t_ns > best[0]:
                    best = (t_ns, transform)
        return best[1] if best is not None else None

    def latest_transforms(self, stamp_ns):
        latest = {}
        for (parent, child), entries in self.transforms.items():
            best = None
            for t_ns, transform in entries:
                if t_ns <= stamp_ns and (best is None or t_ns > best[0]):
                    best = (t_ns, transform)
            if best is not None:
                latest[(parent, child)] = best[1]
        return latest


def stamp_to_ns(stamp):
    return int(stamp.sec * 1_000_000_000 + stamp.nanosec)


def normalize_frame_name(frame_name):
    return frame_name.lstrip("/")


def quaternion_to_matrix(qx, qy, qz, qw):
    x = qx
    y = qy
    z = qz
    w = qw
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def transform_to_matrix(tf_msg):
    matrix = np.eye(4, dtype=float)
    q = tf_msg.transform.rotation
    t = tf_msg.transform.translation
    matrix[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def matrix_to_yaw(matrix):
    return math.atan2(matrix[1, 0], matrix[0, 0])


def compose_tf_chain(tf_store, source_frame, target_frame, stamp_ns):
    source_frame = normalize_frame_name(source_frame)
    target_frame = normalize_frame_name(target_frame)
    if source_frame == target_frame:
        return np.eye(4, dtype=float)

    latest_transforms = tf_store.latest_transforms(stamp_ns)
    graph = defaultdict(list)
    for (parent, child), tf_msg in latest_transforms.items():
        direct = transform_to_matrix(tf_msg)
        inverse = np.linalg.inv(direct)
        graph[child].append((parent, direct))
        graph[parent].append((child, inverse))

    queue = [(source_frame, np.eye(4, dtype=float))]
    visited = {source_frame}
    while queue:
        current_frame, current_matrix = queue.pop(0)
        if current_frame == target_frame:
            return current_matrix
        for next_frame, step_matrix in graph.get(current_frame, []):
            if next_frame in visited:
                continue
            visited.add(next_frame)
            queue.append((next_frame, step_matrix @ current_matrix))

    return None


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


def compute_expected_ranges_from_map(
    scan_msg,
    map_msg,
    tf_store,
    map_frame,
    laser_frame,
    max_range,
    beam_stride=1,
):
    if map_msg is None:
        return None

    stamp_ns = stamp_to_ns(scan_msg.header.stamp)
    map_frame_candidates = [normalize_frame_name(map_msg.header.frame_id), normalize_frame_name(map_frame), "map"]
    laser_frame_candidates = [
        normalize_frame_name(scan_msg.header.frame_id),
        normalize_frame_name(laser_frame),
        "laser",
        "ego_racecar/laser",
    ]

    tf_laser_to_map = None
    for map_frame_name in map_frame_candidates:
        for laser_frame_name in laser_frame_candidates:
            tf_laser_to_map = compose_tf_chain(tf_store, laser_frame_name, map_frame_name, stamp_ns)
            if tf_laser_to_map is not None:
                break
        if tf_laser_to_map is not None:
            break
    if tf_laser_to_map is None:
        return None
    
    beam_stride = max(1, int(beam_stride))
    beam_indices = np.arange(0, len(scan_msg.ranges), beam_stride, dtype=np.int32)
    if beam_indices.size == 0:
        return None

    tx = tf_laser_to_map[0, 3]
    ty = tf_laser_to_map[1, 3]
    yaw = matrix_to_yaw(tf_laser_to_map)

    n_beams = len(beam_indices)
    queries = np.zeros((n_beams, 3), dtype=np.float32)
    queries[:, 0] = tx
    queries[:, 1] = ty
    queries[:, 2] = yaw + (scan_msg.angle_min + beam_indices * scan_msg.angle_increment)

    max_range_px = int(max_range / map_msg.info.resolution)
    o_map = range_libc.PyOMap(map_msg)
    range_method = range_libc.PyRayMarching(o_map, max_range_px)
    ranges_px = np.zeros(n_beams, dtype=np.float32)
    range_method.calc_range_many(queries, ranges_px)

    expected = np.asarray(ranges_px, dtype=np.float32) * map_msg.info.resolution
    sampled_observed = [float(scan_msg.ranges[idx]) for idx in beam_indices]
    sampled_expected = [float(min(v, max_range)) for v in expected]
    return sampled_observed, sampled_expected


def estimate_sensor_model(z, z_star, max_range, init=None,
                          max_iter=MAX_ITERATIONS, tol=1e-8):
    z = np.asarray(z, dtype=float)
    z_star = np.asarray(z_star, dtype=float)

    z = np.where(np.isposinf(z), max_range, z)
    ok = np.isfinite(z) & np.isfinite(z_star)
    z, z_star = np.clip(z[ok], 0.0, max_range), z_star[ok]
    n = len(z)

    p = dict(z_hit=0.9, z_short=0.05, z_max=0.01, z_rand=0.04,
             sigma_hit=2.2, lambda_short=1.0)
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
        cdf = lambda x: 0.5 * (1.0 + np.vectorize(erf, otypes=[float])(x / np.sqrt(2.0)))
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

        print("Updated sensor model parameters:")
        print(f"  z_hit    = {p['z_hit']:.6f}")
        print(f"  z_short  = {p['z_short']:.6f}")
        print(f"  z_max    = {p['z_max']:.6f}")
        print(f"  z_rand   = {p['z_rand']:.6f}")
        print(f"  sigma_hit= {p['sigma_hit']:.6f}")
        print(f"  lambda_short= {p['lambda_short']:.6f}")

        ll = np.sum(np.log(total))
        if abs(ll - prev_ll) < tol * abs(ll):
            break
        prev_ll = ll

    return p


def main():
    print("[offline_sensor_calibration] Opening bag and reading messages...", flush=True)
    print(
        f"[offline_sensor_calibration] Settings: SCAN_BEAM_STRIDE={SCAN_BEAM_STRIDE}, MAX_SCAN_MESSAGES_TO_USE={MAX_SCAN_MESSAGES_TO_USE}",
        flush=True,
    )
    bag = BagReader(BAG_PATH)
    tf_store = TFStore()
    map_msg = None

    scan_messages = []
    message_count = 0
    scan_count = 0
    tf_count = 0
    map_count = 0

    for topic, data in bag.iter_messages():
        message_count += 1
        if topic == MAP_TOPIC:
            map_msg = deserialize_message(data, get_message(bag.topic_types[topic]))
            map_count += 1
            print("[offline_sensor_calibration] Received /map", flush=True)
            continue

        if topic == TF_TOPIC:
            tf_msg = deserialize_message(data, get_message(bag.topic_types[topic]))
            tf_store.add_tf_message(tf_msg)
            tf_count += 1
            if tf_count % 2000 == 0:
                print(f"[offline_sensor_calibration] Processed {tf_count} /tf messages (total messages: {message_count})", flush=True)
            continue

        if topic == TF_STATIC_TOPIC:
            tf_msg = deserialize_message(data, get_message(bag.topic_types[topic]))
            tf_store.add_tf_message(tf_msg)
            continue

        if topic == SCAN_TOPIC:
            scan_msg = deserialize_message(data, get_message(bag.topic_types[topic]))
            scan_messages.append(scan_msg)
            scan_count += 1
            if scan_count % 500 == 0:
                print(f"[offline_sensor_calibration] Collected {scan_count} /scan messages (total messages: {message_count})", flush=True)

    print(
        f"[offline_sensor_calibration] Finished reading bag: {message_count} messages, {scan_count} scans, {tf_count} tf messages, {map_count} maps",
        flush=True,
    )

    accumulated_ranges = []
    accumulated_expected = []
    if MAX_SCAN_MESSAGES_TO_USE > 0:
        scan_messages = scan_messages[:MAX_SCAN_MESSAGES_TO_USE]
    print(
        f"[offline_sensor_calibration] Computing expected ranges for {len(scan_messages)} scans with beam stride {SCAN_BEAM_STRIDE}...",
        flush=True,
    )
    processed_scans = 0
    for scan_msg in scan_messages:
        result = compute_expected_ranges_from_map(
            scan_msg,
            map_msg,
            tf_store,
            MAP_FRAME,
            LASER_FRAME,
            MAX_RANGE,
            beam_stride=SCAN_BEAM_STRIDE,
        )
        if result is None:
            continue

        sampled_observed, expected_ranges = result

        accumulated_ranges.append(sampled_observed)
        accumulated_expected.append(expected_ranges)
        processed_scans += 1
        if processed_scans % 100 == 0:
            print(f"[offline_sensor_calibration] Built expected ranges for {processed_scans} scans", flush=True)

    if not accumulated_ranges:
        raise RuntimeError("No LaserScan messages were found for the provided scan topic.")

    print(
        f"[offline_sensor_calibration] Estimating sensor model from {len(accumulated_ranges)} scans / {sum(len(r) for r in accumulated_ranges)} beams...",
        flush=True,
    )
    flat_ranges = []
    flat_expected = []
    for ranges, expected in zip(accumulated_ranges, accumulated_expected):
        for z, z_star in zip(ranges, expected):
            flat_ranges.append(float(z))
            flat_expected.append(float(z_star))

    print("[offline_sensor_calibration] Running EM optimization...", flush=True)
    estimate = estimate_sensor_model(
        flat_ranges,
        flat_expected,
        MAX_RANGE,
        init={"z_hit": 0.75, "z_short": 0.1, "z_max": 0.05, "z_rand": 0.1, "sigma_hit": 0.3, "lambda_short": 1.0},
    )

    print("Final estimated sensor model parameters:")
    print(f"  z_hit    = {estimate['z_hit']:.6f}")
    print(f"  z_short  = {estimate['z_short']:.6f}")
    print(f"  z_max    = {estimate['z_max']:.6f}")
    print(f"  z_rand   = {estimate['z_rand']:.6f}")
    print(f"  sigma_hit= {estimate['sigma_hit']:.6f}")
    print(f"  lambda_short= {p['lambda_short']:.6f}")



if __name__ == "__main__":
    main()
