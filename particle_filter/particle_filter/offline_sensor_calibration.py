#!/usr/bin/env python3
"""Offline beam-model calibration (EM) from a rosbag2 recording."""

import math
from collections import defaultdict, deque
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
import range_libc
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation, Slerp
from scipy.special import erf

BAG_PATH = "/sim_ws/src/arcus/resources/rosbag2_2026_10_14-00_33_28"
SCAN_TOPIC = "/scan"
MAP_TOPIC = "/map"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
MAP_FRAME = "map"
LASER_FRAME = "ego_racecar/laser"

MAX_RANGE = 10.0  # use the particle filter's max_range
THETA_DISCRETIZATION = 112.0  # cddt, pcddt and glt only
SCAN_BEAM_STRIDE = 4
MAX_SCAN_MESSAGES_TO_USE = 500  # 0 keeps every scan
MAX_ITERATIONS = 1000
DEBUG_PLOTS = True

LOG = "[offline_sensor_calibration]"
EPS = 1e-12


# ---------------------------------------------------------------- bag and TF
def stamp_to_ns(stamp):
    return int(stamp.sec * 1_000_000_000 + stamp.nanosec)


def frame_name(name):
    return name.lstrip("/")


def pose_to_matrix(position, quat_xyzw):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    matrix[:3, 3] = position
    return matrix


class TFStore:
    """Interpolated /tf, always-valid /tf_static, and chained frame lookups."""

    def __init__(self):
        self.raw = defaultdict(list)  # edge -> [(stamp_ns, position, quat)]
        self.dynamic = {}  # edge -> (stamps, positions, quats)
        self.static = {}  # edge -> 4x4 matrix
        self.neighbors = defaultdict(list)
        self.paths = {}

    def add(self, msg, is_static):
        for tf in msg.transforms:
            edge = (frame_name(tf.header.frame_id), frame_name(tf.child_frame_id))
            t, q = tf.transform.translation, tf.transform.rotation
            position, quat = (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)
            if is_static:
                self.static[edge] = pose_to_matrix(position, quat)
            else:
                self.raw[edge].append((stamp_to_ns(tf.header.stamp), position, quat))

    def finalize(self):
        """Call once after all messages are added."""
        for edge, entries in self.raw.items():
            entries.sort(key=lambda e: e[0])
            self.dynamic[edge] = (
                np.array([e[0] for e in entries], dtype=np.int64),
                np.array([e[1] for e in entries]),
                np.array([e[2] for e in entries]),
            )
        for parent, child in list(self.dynamic) + list(self.static):
            self.neighbors[child].append((parent, (parent, child), False))
            self.neighbors[parent].append((child, (parent, child), True))

    def edge_matrix(self, edge, stamp_ns):
        if edge not in self.dynamic:
            return self.static.get(edge)

        stamps, positions, quats = self.dynamic[edge]
        if stamp_ns < stamps[0] or stamp_ns > stamps[-1]:
            return None
        i1 = min(int(np.searchsorted(stamps, stamp_ns, side="right")), len(stamps) - 1)
        i0 = max(i1 - 1, 0)
        if i0 == i1:
            return pose_to_matrix(positions[i0], quats[i0])

        alpha = (stamp_ns - stamps[i0]) / (stamps[i1] - stamps[i0])
        position = positions[i0] + alpha * (positions[i1] - positions[i0])
        slerp = Slerp([0.0, 1.0], Rotation.from_quat([quats[i0], quats[i1]]))
        return pose_to_matrix(position, slerp(alpha).as_quat())

    def find_path(self, source, target):
        if (source, target) in self.paths:
            return self.paths[(source, target)]

        path = None
        visited = {source}
        queue = deque([(source, [])])
        while queue:
            frame, steps = queue.popleft()
            if frame == target:
                path = steps
                break
            for next_frame, edge, invert in self.neighbors[frame]:
                if next_frame not in visited:
                    visited.add(next_frame)
                    queue.append((next_frame, steps + [(edge, invert)]))

        self.paths[(source, target)] = path
        return path

    def lookup(self, source, target, stamp_ns):
        """4x4 matrix mapping points in the source frame into the target frame."""
        source, target = frame_name(source), frame_name(target)
        path = self.find_path(source, target)
        if path is None:
            return None

        result = np.eye(4)
        for edge, invert in path:
            step = self.edge_matrix(edge, stamp_ns)
            if step is None:
                return None
            result = (np.linalg.inv(step) if invert else step) @ result
        return result


def read_bag(bag_path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    types = {t.topic_metadata.name: t.topic_metadata.type
             for t in reader.get_metadata().topics_with_message_count}

    tf_store = TFStore()
    map_msg = None
    scan_messages = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in (MAP_TOPIC, TF_TOPIC, TF_STATIC_TOPIC, SCAN_TOPIC):
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == MAP_TOPIC:
            map_msg = msg
        elif topic == SCAN_TOPIC:
            scan_messages.append(msg)
        else:
            tf_store.add(msg, is_static=(topic == TF_STATIC_TOPIC))

    tf_store.finalize()
    print(f"{LOG} Read {len(scan_messages)} scans")
    return map_msg, tf_store, scan_messages


# ------------------------------------------------------------- expected ranges
class RayCaster:
    """range_libc as the particle filter uses it: world-frame poses in, meters out."""

    def __init__(self, map_msg, max_range):
        self.o_map = range_libc.PyOMap(map_msg)  # must outlive the method
        max_range_px = int(max_range / map_msg.info.resolution)

        self.method = range_libc.PyRayMarching(self.o_map, max_range_px)

    def ranges(self, x, y, headings):
        queries = np.zeros((len(headings), 3), dtype=np.float32)
        queries[:, 0] = x
        queries[:, 1] = y
        queries[:, 2] = headings
        out = np.zeros(len(headings), dtype=np.float32)
        self.method.calc_range_many(queries, out)
        return out.astype(float)


@dataclass
class ScanResult:
    x: float  # laser pose in the map frame
    y: float
    yaw: float
    angles: np.ndarray  # beam angles in the laser frame
    observed: np.ndarray
    expected: np.ndarray


def expected_ranges(scan_msg, caster, tf_store):
    stamp_ns = stamp_to_ns(scan_msg.header.stamp)
    laser_to_map = None
    for laser_frame in (scan_msg.header.frame_id, LASER_FRAME):
        laser_to_map = tf_store.lookup(laser_frame, MAP_FRAME, stamp_ns)
        if laser_to_map is not None:
            break
    if laser_to_map is None:
        return None

    beams = np.arange(0, len(scan_msg.ranges), SCAN_BEAM_STRIDE)
    angles = scan_msg.angle_min + beams * scan_msg.angle_increment
    x, y = laser_to_map[0, 3], laser_to_map[1, 3]
    yaw = math.atan2(laser_to_map[1, 0], laser_to_map[0, 0])

    expected = np.minimum(caster.ranges(x, y, yaw + angles), MAX_RANGE)
    observed = np.asarray(scan_msg.ranges, dtype=float)[beams]
    return ScanResult(x, y, yaw, angles, observed, expected)


# ------------------------------------------------------------------------ EM
def clean(z, z_star):
    """Map +inf to max range, drop NaNs, clip to [0, MAX_RANGE]."""
    z = np.where(np.isposinf(z), MAX_RANGE, z)
    ok = np.isfinite(z) & np.isfinite(z_star)
    return np.clip(z[ok], 0.0, MAX_RANGE), z_star[ok]


def normal_cdf(x):
    return 0.5 * (1.0 + erf(x / np.sqrt(2.0)))


def hit_pdf(z, z_star, sigma):
    """Gaussian around z_star, truncated to [0, MAX_RANGE]."""
    pdf = np.exp(-0.5 * ((z - z_star) / sigma) ** 2) / (np.sqrt(2 * np.pi) * sigma)
    mass = normal_cdf((MAX_RANGE - z_star) / sigma) - normal_cdf(-z_star / sigma)
    return pdf / np.maximum(mass, EPS)


def short_pdf(z, z_star, lam):
    """Exponential on [0, z_star), truncated."""
    norm = -np.expm1(-lam * z_star)
    valid = (z < z_star) & (norm > EPS)
    return np.where(valid, lam * np.exp(-lam * z) / np.maximum(norm, EPS), 0.0)


def print_params(p, header):
    print(header)
    for name in ("z_hit", "z_short", "z_max", "z_rand", "sigma_hit", "lambda_short"):
        print(f"  {name:<12} = {p[name]:.6f}")


def print_pf_params(p, resolution):
    """The estimate in the particle filter's units: sigma_hit and lambda_short per pixel."""
    print("Particle filter parameters:")
    for name in ("z_hit", "z_short", "z_max", "z_rand"):
        print(f"  {name:<12} = {p[name]:.6f}")
    print(f"  sigma_hit    = {p['sigma_hit'] / resolution:.4f}  # pixels")
    print(f"  lambda_short = {p['lambda_short'] * resolution:.6f}  # 1/pixel")
    print(f"  max_range    = {MAX_RANGE}")


def estimate_sensor_model(z, z_star, sigma_min):
    """EM for the beam model (Probabilistic Robotics, Table 6.2)."""
    n = len(z)
    p = dict(z_hit=0.75, z_short=0.1, z_max=0.05, z_rand=0.1, sigma_hit=0.3, lambda_short=1.0)

    is_max = z >= MAX_RANGE - 1e-6
    p_max = is_max.astype(float)
    p_rand = np.where(is_max, 0.0, 1.0 / MAX_RANGE)

    ll_history = []
    prev_ll = -np.inf
    for iteration in range(1, MAX_ITERATIONS + 1):
        p_hit = np.where(is_max, 0.0, hit_pdf(z, z_star, p["sigma_hit"]))
        p_short = short_pdf(z, z_star, p["lambda_short"])

        # E-step
        w_hit = p["z_hit"] * p_hit
        w_short = p["z_short"] * p_short
        w_max = p["z_max"] * p_max
        w_rand = p["z_rand"] * p_rand
        total = np.maximum(w_hit + w_short + w_max + w_rand, EPS)
        e_hit, e_short = w_hit / total, w_short / total
        e_max, e_rand = w_max / total, w_rand / total

        # M-step
        p["z_hit"], p["z_short"] = e_hit.sum() / n, e_short.sum() / n
        p["z_max"], p["z_rand"] = e_max.sum() / n, e_rand.sum() / n
        sigma = np.sqrt(np.sum(e_hit * (z - z_star) ** 2) / max(e_hit.sum(), EPS))
        p["sigma_hit"] = max(sigma, sigma_min)
        p["lambda_short"] = e_short.sum() / max(np.sum(e_short * z), EPS)

        ll = np.sum(np.log(total))
        ll_history.append(ll)
        print_params(p, f"{LOG} iteration {iteration}, log-likelihood {ll:.2f}")
        if abs(ll - prev_ll) < 1e-8 * abs(ll):
            break
        prev_ll = ll

    return p, ll_history


# --------------------------------------------------------------------- plots
def print_residual_summary(z, z_star, map_info):
    residuals = (z - z_star)[z < MAX_RANGE - 1e-6]
    percentiles = np.percentile(residuals, [5, 25, 50, 75, 95])
    print(f"{LOG} residual percentiles 5/25/50/75/95 (m): {np.round(percentiles, 3)}")
    print(f"{LOG} fraction within 10 cm: {np.mean(np.abs(residuals) < 0.1):.3f}")
    print(f"{LOG} beams at max range: {np.mean(z >= MAX_RANGE - 1e-6):.3f}")
    print(f"{LOG} map resolution: {map_info.resolution} m/px")


def show(fig, title):
    fig.tight_layout()
    fig.canvas.manager.set_window_title(title)


def plot_scatter_and_residuals(z, z_star):
    sample = np.random.default_rng(0).choice(len(z), min(20000, len(z)), replace=False)
    residuals = (z - z_star)[z < MAX_RANGE - 1e-6]

    fig, ax = plt.subplots(1, 2, figsize=(12, 5))
    ax[0].scatter(z_star[sample], z[sample], s=2, alpha=0.2)
    ax[0].plot([0, MAX_RANGE], [0, MAX_RANGE], "r--", lw=1)
    ax[0].set(xlabel="expected z* (m)", ylabel="measured z (m)", title="measured vs expected")
    ax[1].hist(residuals[np.abs(residuals) < 1.0], bins=200, log=True)
    ax[1].set(xlabel="z - z* (m)", ylabel="count (log)",
              title="residuals, max-range beams excluded (|r| < 1 m)")
    show(fig, "01 scatter and residuals")


def plot_scans_on_map(scans, map_msg, n_scans=4):
    info = map_msg.info
    grid = np.asarray(map_msg.data, dtype=float).reshape((info.height, info.width))
    image = np.where(grid < 0, 0.5, 1.0 - grid / 100.0)
    x0, y0 = info.origin.position.x, info.origin.position.y
    extent = [x0, x0 + info.width * info.resolution, y0, y0 + info.height * info.resolution]
    half = min(MAX_RANGE, 15.0)

    picks = np.unique(np.linspace(0, len(scans) - 1, n_scans).astype(int))
    fig, axes = plt.subplots(1, len(picks), figsize=(5 * len(picks), 5), squeeze=False)
    for ax, i in zip(axes[0], picks):
        s = scans[i]
        ax.imshow(image, cmap="gray", vmin=0, vmax=1, origin="lower", extent=extent)
        for ranges, color, label, size in ((s.expected, "tab:orange", "expected", 22),
                                           (s.observed, "tab:blue", "measured", 5)):
            hit = np.isfinite(ranges) & (ranges < MAX_RANGE - 1e-6)
            heading = s.yaw + s.angles[hit]
            ax.scatter(s.x + ranges[hit] * np.cos(heading), s.y + ranges[hit] * np.sin(heading),
                       s=size, c=color, label=label)
        ax.arrow(s.x, s.y, 0.8 * np.cos(s.yaw), 0.8 * np.sin(s.yaw), color="red", head_width=0.3)
        ax.set(xlim=(s.x - half, s.x + half), ylim=(s.y - half, s.y + half),
               title=f"scan {i}", xlabel="x (m)", ylabel="y (m)")
        ax.set_aspect("equal")
    axes[0][0].legend(loc="upper right", fontsize=8)
    show(fig, "03 scans on map")


def plot_fit_check(z, z_star, p, ll_history):
    keep = z < MAX_RANGE - 1e-6
    residuals = (z - z_star)[keep]
    z_star_sample = np.random.default_rng(0).choice(z_star[keep], min(2000, keep.sum()), replace=False)

    # Mixture density of the residual, averaged over the sampled z*. Scaled up because
    # the histogram leaves out max-range beams.
    offsets = np.linspace(-1.0, 1.0, 401)
    zs = z_star_sample[None, :]
    zz = zs + offsets[:, None]
    inside = (zz >= 0.0) & (zz < MAX_RANGE)
    scale = 1.0 / (1.0 - p["z_max"])
    components = {
        "hit": p["z_hit"] * hit_pdf(zz, zs, p["sigma_hit"]),
        "short": p["z_short"] * short_pdf(zz, zs, p["lambda_short"]),
        "rand": np.full_like(zz, p["z_rand"] / MAX_RANGE),
    }
    curves = {name: scale * np.where(inside, c, 0.0).mean(axis=1) for name, c in components.items()}
    mixture = sum(curves.values())

    bins = np.linspace(-1.0, 1.0, 101)
    counts, _ = np.histogram(residuals, bins=bins)
    density = counts / (len(residuals) * (bins[1] - bins[0]))

    fig, ax = plt.subplots(1, 2, figsize=(12, 5))
    ax[0].bar(0.5 * (bins[:-1] + bins[1:]), density, width=bins[1] - bins[0], alpha=0.4, label="data")
    ax[0].plot(offsets, mixture, "r", label="fitted mixture")
    for name, curve in curves.items():
        ax[0].plot(offsets, curve, "--", lw=1, label=name)
    ax[0].set(xlabel="z - z* (m)", ylabel="density (1/m)", yscale="log",
              title="fit vs data (max-range beams excluded)")
    ax[0].set_ylim(1e-4, 3.0 * max(density.max(), mixture.max()))
    ax[0].legend(fontsize=8)
    ax[1].plot(np.arange(1, len(ll_history) + 1), ll_history, ".-")
    ax[1].set(xlabel="EM iteration", ylabel="log-likelihood", title="EM convergence")
    show(fig, "07 fit check")


# ---------------------------------------------------------------------- main
def main():
    map_msg, tf_store, scan_messages = read_bag(BAG_PATH)
    if map_msg is None:
        raise RuntimeError(f"No message was received on {MAP_TOPIC}.")

    if 0 < MAX_SCAN_MESSAGES_TO_USE < len(scan_messages):
        scan_messages = scan_messages[:: math.ceil(len(scan_messages) / MAX_SCAN_MESSAGES_TO_USE)]

    caster = RayCaster(map_msg, MAX_RANGE)
    scans = [s for s in (expected_ranges(m, caster, tf_store) for m in scan_messages) if s]
    if not scans:
        raise RuntimeError("No scans could be matched with a map pose. Check the TF frames "
                           "and that the scan stamps fall inside the recorded /tf time range.")
    print(f"{LOG} Computed expected ranges for {len(scans)} scans")

    z, z_star = clean(np.concatenate([s.observed for s in scans]),
                      np.concatenate([s.expected for s in scans]))
    resolution = map_msg.info.resolution
    print_residual_summary(z, z_star, map_msg.info)

    if DEBUG_PLOTS:
        plot_scatter_and_residuals(z, z_star)
        plot_scans_on_map(scans, map_msg)
        print(f"{LOG} Close all plot windows to continue.")
        plt.show()

    estimate, ll_history = estimate_sensor_model(z, z_star, sigma_min=0.5 * resolution)
    print_params(estimate, "Final estimated sensor model parameters (meters):")
    print_pf_params(estimate, resolution)

    if DEBUG_PLOTS:
        plot_fit_check(z, z_star, estimate, ll_history)
        plt.show()


if __name__ == "__main__":
    main()