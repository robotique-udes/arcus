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
from scipy.special import erf

BAG_PATH = "/sim_ws/src/arcus/resources/rosbag2_2026_10_14-00_33_28"
SCAN_TOPIC = "/scan"
MAP_TOPIC = "/map"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
MAP_FRAME = "map"
LASER_FRAME = "ego_racecar/laser"
MAX_RANGE = 10.0
MAX_ITERATIONS = 1000
PRINT_EVERY = 1  # log every EM iteration; 0 disables
SCAN_BEAM_STRIDE = 4
MAX_SCAN_MESSAGES_TO_USE = 500  # 0 = keep every scan
RANGE_METHOD = "rm"  # range_libc method, as in the particle filter: bl, rm, rmgpu, cddt, pcddt, glt
THETA_DISCRETIZATION = 112.0  # only used by cddt / pcddt / glt
DEBUG_PLOTS = True  # windows open and block until closed; EM continues afterwards

LOG = "[offline_sensor_calibration]"


# --------------------------------------------------------------------------
# Bag reading
# --------------------------------------------------------------------------
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
        for info in self.reader.get_metadata().topics_with_message_count:
            self.topic_types[info.topic_metadata.name] = info.topic_metadata.type

    def iter_messages(self):
        while self.reader.has_next():
            topic, data, _ = self.reader.read_next()
            yield topic, data


# --------------------------------------------------------------------------
# Small math helpers
# --------------------------------------------------------------------------
def stamp_to_ns(stamp):
    return int(stamp.sec * 1_000_000_000 + stamp.nanosec)


def normalize_frame_name(frame_name):
    return frame_name.lstrip("/")


def quaternion_to_matrix(qx, qy, qz, qw):
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def pose_to_matrix(position, quat_xyzw):
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = quaternion_to_matrix(*quat_xyzw)
    matrix[:3, 3] = position
    return matrix


def matrix_to_yaw(matrix):
    return math.atan2(matrix[1, 0], matrix[0, 0])


def slerp(q0, q1, t):
    """Spherical interpolation between two unit quaternions (x, y, z, w)."""
    dot = float(np.dot(q0, q1))
    if dot < 0.0:  # take the short way around
        q1, dot = -q1, -dot

    if dot > 0.9995:  # nearly parallel: lerp is stable and accurate
        q = q0 + t * (q1 - q0)
    else:
        theta0 = math.acos(dot)
        s0 = math.sin((1.0 - t) * theta0) / math.sin(theta0)
        s1 = math.sin(t * theta0) / math.sin(theta0)
        q = s0 * q0 + s1 * q1
    return q / np.linalg.norm(q)


# --------------------------------------------------------------------------
# TF store: interpolated dynamic transforms, always-valid static transforms,
# and cached frame chains.
# --------------------------------------------------------------------------
class TFStore:
    def __init__(self):
        self._raw = defaultdict(list)   # key -> [(stamp_ns, pos, quat)]
        self._dynamic = {}              # key -> (stamps, positions, quats)
        self._static = {}               # key -> 4x4 matrix
        self._adjacency = defaultdict(list)
        self._path_cache = {}

    def add_tf_message(self, msg, is_static):
        for tf in msg.transforms:
            key = (
                normalize_frame_name(tf.header.frame_id),
                normalize_frame_name(tf.child_frame_id),
            )
            t, q = tf.transform.translation, tf.transform.rotation
            position = (t.x, t.y, t.z)
            quat = (q.x, q.y, q.z, q.w)
            if is_static:
                self._static[key] = pose_to_matrix(position, quat)
            else:
                self._raw[key].append((stamp_to_ns(tf.header.stamp), position, quat))

    def finalize(self):
        """Call once after all messages are added: sort, convert, build graph."""
        for key, entries in self._raw.items():
            entries.sort(key=lambda e: e[0])
            self._dynamic[key] = (
                np.array([e[0] for e in entries], dtype=np.int64),
                np.array([e[1] for e in entries], dtype=float),
                np.array([e[2] for e in entries], dtype=float),
            )
        self._raw.clear()

        for parent, child in list(self._dynamic) + list(self._static):
            key = (parent, child)
            self._adjacency[child].append((parent, key, False))  # child -> parent: direct
            self._adjacency[parent].append((child, key, True))   # parent -> child: inverse

    def _edge_matrix(self, key, stamp_ns):
        """4x4 transform (child -> parent) for one edge at stamp_ns."""
        if key in self._dynamic:
            stamps, positions, quats = self._dynamic[key]
            if stamp_ns < stamps[0] or stamp_ns > stamps[-1]:
                return None  # cannot interpolate outside the recorded range

            i1 = int(np.searchsorted(stamps, stamp_ns, side="right"))
            if i1 == len(stamps):  # stamp equals the last sample exactly
                return pose_to_matrix(positions[-1], quats[-1])

            i0 = i1 - 1
            alpha = (stamp_ns - stamps[i0]) / (stamps[i1] - stamps[i0])
            position = positions[i0] + alpha * (positions[i1] - positions[i0])
            quat = slerp(quats[i0], quats[i1], alpha)
            return pose_to_matrix(position, quat)

        return self._static.get(key)  # static transforms ignore the stamp

    def _find_path(self, source, target):
        """BFS over frames; returns [(edge_key, invert)] or None. Cached."""
        cache_key = (source, target)
        if cache_key in self._path_cache:
            return self._path_cache[cache_key]

        path = None
        visited = {source}
        queue = deque([(source, [])])
        while queue:
            frame, steps = queue.popleft()
            if frame == target:
                path = steps
                break
            for next_frame, key, invert in self._adjacency.get(frame, []):
                if next_frame not in visited:
                    visited.add(next_frame)
                    queue.append((next_frame, steps + [(key, invert)]))

        self._path_cache[cache_key] = path
        return path

    def lookup_chain(self, source_frame, target_frame, stamp_ns):
        """4x4 matrix mapping points in source_frame into target_frame."""
        source = normalize_frame_name(source_frame)
        target = normalize_frame_name(target_frame)
        if source == target:
            return np.eye(4, dtype=float)

        path = self._find_path(source, target)
        if path is None:
            return None

        result = np.eye(4, dtype=float)
        for key, invert in path:
            step = self._edge_matrix(key, stamp_ns)
            if step is None:
                return None
            if invert:
                step = np.linalg.inv(step)
            result = step @ result
        return result


# --------------------------------------------------------------------------
# Expected ranges via ray casting
# --------------------------------------------------------------------------
class RangeLibcCaster:
    """range_libc used the way the particle filter uses it.

    In this build calc_range_many takes WORLD coordinates (x, y, theta in the map
    frame) and returns ranges in METERS. The world <-> grid conversion happens inside
    range_libc, so nothing is converted here.
    """

    def __init__(self, map_msg, max_range, method="rm", theta_discretization=112.0):
        self._o_map = range_libc.PyOMap(map_msg)  # keep alive with the method
        max_range_px = int(max_range / map_msg.info.resolution)

        if method == "bl":
            self._method = range_libc.PyBresenhamsLine(self._o_map, max_range_px)
        elif "cddt" in method:
            self._method = range_libc.PyCDDTCast(self._o_map, max_range_px, theta_discretization)
            if method == "pcddt":
                self._method.prune()
        elif method == "rm":
            self._method = range_libc.PyRayMarching(self._o_map, max_range_px)
        elif method == "rmgpu":
            self._method = range_libc.PyRayMarchingGPU(self._o_map, max_range_px)
        elif method == "glt":
            self._method = range_libc.PyGiantLUTCast(self._o_map, max_range_px, theta_discretization)
        else:
            raise ValueError(f"Unknown range method: {method}")

    def calc_range_world(self, x, y, thetas):
        """Ranges in meters for rays from world pose (x, y) at world headings thetas."""
        queries = np.zeros((len(thetas), 3), dtype=np.float32)
        queries[:, 0] = x
        queries[:, 1] = y
        queries[:, 2] = thetas
        ranges = np.zeros(len(thetas), dtype=np.float32)
        self._method.calc_range_many(queries, ranges)
        return ranges.astype(float)


def build_range_method(map_msg, max_range):
    """Build the ray caster once."""
    return RangeLibcCaster(map_msg, max_range, RANGE_METHOD, THETA_DISCRETIZATION)


@dataclass
class ScanResult:
    """Everything the debug plots need to know about one processed scan."""
    stamp_ns: int
    x: float            # laser pose in the map frame
    y: float
    yaw: float
    angles: np.ndarray  # beam angles in the laser frame (sampled beams only)
    observed: np.ndarray
    expected: np.ndarray


def compute_expected_ranges_from_map(
    scan_msg,
    range_method,
    tf_store,
    map_frame,
    laser_frame,
    max_range,
    beam_stride=1,
):
    stamp_ns = stamp_to_ns(scan_msg.header.stamp)

    laser_to_map = None
    for laser_name in (scan_msg.header.frame_id, laser_frame):
        laser_to_map = tf_store.lookup_chain(laser_name, map_frame, stamp_ns)
        if laser_to_map is not None:
            break
    if laser_to_map is None:
        return None

    beam_indices = np.arange(0, len(scan_msg.ranges), max(1, int(beam_stride)))
    if beam_indices.size == 0:
        return None

    yaw = matrix_to_yaw(laser_to_map)
    angles = scan_msg.angle_min + beam_indices * scan_msg.angle_increment

    # The caster takes the laser pose in the map frame (meters) and returns meters.
    ranges_m = range_method.calc_range_world(
        laser_to_map[0, 3], laser_to_map[1, 3], yaw + angles
    )
    expected = np.minimum(ranges_m, max_range)
    observed = np.asarray(scan_msg.ranges, dtype=float)[beam_indices]
    return ScanResult(
        stamp_ns=stamp_ns,
        x=float(laser_to_map[0, 3]),
        y=float(laser_to_map[1, 3]),
        yaw=float(yaw),
        angles=np.asarray(angles, dtype=float),
        observed=observed,
        expected=expected.astype(float),
    )


# --------------------------------------------------------------------------
# EM for the beam model (Probabilistic Robotics, Table 6.2)
# --------------------------------------------------------------------------
def print_params(p, header):
    print(header)
    print(f"  z_hit        = {p['z_hit']:.6f}")
    print(f"  z_short      = {p['z_short']:.6f}")
    print(f"  z_max        = {p['z_max']:.6f}")
    print(f"  z_rand       = {p['z_rand']:.6f}")
    print(f"  sigma_hit    = {p['sigma_hit']:.6f}")
    print(f"  lambda_short = {p['lambda_short']:.6f}")


def normal_cdf(x):
    return 0.5 * (1.0 + erf(x / np.sqrt(2.0)))


def estimate_sensor_model(z, z_star, max_range, init=None, sigma_min=1e-3,
                          max_iter=MAX_ITERATIONS, tol=1e-8,
                          print_every=PRINT_EVERY):
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
    p["sigma_hit"] = max(p["sigma_hit"], sigma_min)

    eps = 1e-12

    # Data-only quantities: compute once.
    is_max = z >= max_range - 1e-6
    is_short = (z < z_star) & ~is_max
    p_max = is_max.astype(float)
    p_rand = np.where(is_max, 0.0, 1.0 / max_range)  # max readings are discrete events

    prev_ll = -np.inf
    iteration = 0
    ll_history = []
    for iteration in range(1, max_iter + 1):
        s, lam = p["sigma_hit"], p["lambda_short"]

        # Component densities
        norm = normal_cdf((max_range - z_star) / s) - normal_cdf(-z_star / s)
        p_hit = np.exp(-0.5 * ((z - z_star) / s) ** 2) / (np.sqrt(2 * np.pi) * s)
        p_hit /= np.maximum(norm, eps)
        p_hit[is_max] = 0.0  # max readings are discrete events

        denom = -np.expm1(-lam * z_star)
        p_short = np.zeros(n)
        v = is_short & (denom > eps)
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
        sigma = np.sqrt(np.sum(e_hit * (z - z_star) ** 2) / max(e_hit.sum(), eps))
        p["sigma_hit"] = max(sigma, sigma_min)
        p["lambda_short"] = e_short.sum() / max(np.sum(e_short * z), eps)

        ll = np.sum(np.log(total))
        ll_history.append(ll)
        converged = abs(ll - prev_ll) < tol * abs(ll)

        if print_every and iteration % print_every == 0:
            print_params(p, f"{LOG} iteration {iteration}, log-likelihood {ll:.2f}")
        if converged:
            break
        prev_ll = ll

    status = "converged" if converged else "hit max_iter"
    print(f"{LOG} EM {status} after {iteration} iterations (log-likelihood {ll:.2f})")
    return p, ll_history


# --------------------------------------------------------------------------
# Debug plots
# --------------------------------------------------------------------------
def flatten_scans(scans, max_range):
    """Concatenate per-scan results into aligned flat arrays (valid beams only)."""
    z = np.concatenate([s.observed for s in scans])
    z_star = np.concatenate([s.expected for s in scans])

    z = np.where(np.isposinf(z), max_range, z)
    valid = np.isfinite(z) & np.isfinite(z_star)
    z = np.clip(z[valid], 0.0, max_range)
    return {
        "z": z,
        "z_star": z_star[valid],
        "is_max": z >= max_range - 1e-6,
    }


def residuals_without_max(d):
    """Residual z - z* for non-max beams, with the matching mask."""
    keep = ~d["is_max"]
    return d["z"][keep] - d["z_star"][keep], keep


def finish_figure(fig, title):
    """Lay the figure out and label its window; plt.show() displays all of them."""
    fig.tight_layout()
    fig.canvas.manager.set_window_title(title)


def print_residual_summary(d, map_msg, max_range):
    res, _ = residuals_without_max(d)
    pct = np.percentile(res, [5, 25, 50, 75, 95])
    print(f"{LOG} --- data check ---")
    print(f"{LOG} beams: {len(d['z'])}, at max range: {d['is_max'].mean():.3f}")
    print(f"{LOG} residual percentiles 5/25/50/75/95 (m): {np.round(pct, 3)}")
    print(f"{LOG} fraction |z - z*| < 0.1 m: {np.mean(np.abs(res) < 0.1):.3f}")
    print(
        f"{LOG} MAX_RANGE={max_range}, map origin=({map_msg.info.origin.position.x:.2f}, "
        f"{map_msg.info.origin.position.y:.2f}), resolution={map_msg.info.resolution}",
        flush=True,
    )


def plot_scatter_and_residuals(d, max_range):
    """1 + 2: measured vs expected, and the residual histogram."""
    rng = np.random.default_rng(0)
    n = len(d["z"])
    sel = rng.choice(n, min(20000, n), replace=False)
    res, _ = residuals_without_max(d)

    fig, ax = plt.subplots(1, 2, figsize=(12, 5))
    ax[0].scatter(d["z_star"][sel], d["z"][sel], s=2, alpha=0.2)
    ax[0].plot([0, max_range], [0, max_range], "r--", lw=1)
    ax[0].set(xlabel="expected z* (m)", ylabel="measured z (m)", title="measured vs expected")

    ax[1].hist(res[np.abs(res) < 1.0], bins=200, log=True)
    ax[1].set(xlabel="z - z* (m)", ylabel="count (log)",
              title="residuals, max-range beams excluded (|r| < 1 m)")
    finish_figure(fig, "01 scatter and residuals")


def plot_scans_on_map(scans, map_msg, max_range, n_scans=4):
    """3: measured and expected hit points over the occupancy grid."""
    info = map_msg.info
    grid = np.asarray(map_msg.data, dtype=float).reshape((info.height, info.width))
    image = np.where(grid < 0, 0.5, 1.0 - grid / 100.0)  # unknown grey, free white, occupied black
    ox, oy = info.origin.position.x, info.origin.position.y
    extent = [ox, ox + info.width * info.resolution, oy, oy + info.height * info.resolution]

    picks = np.unique(np.linspace(0, len(scans) - 1, n_scans).astype(int))
    fig, axes = plt.subplots(1, len(picks), figsize=(5 * len(picks), 5), squeeze=False)
    half = min(max_range, 15.0)

    for ax, idx in zip(axes[0], picks):
        s = scans[idx]
        ax.imshow(image, cmap="gray", vmin=0, vmax=1, origin="lower", extent=extent)

        z = np.where(np.isposinf(s.observed), max_range, s.observed)
        # Expected first (large marker), measured on top (small marker), so both stay visible.
        for ranges, color, label, size in ((s.expected, "tab:orange", "expected", 22),
                                           (z, "tab:blue", "measured", 5)):
            ok = np.isfinite(ranges) & (ranges < max_range - 1e-6)
            heading = s.yaw + s.angles[ok]
            ax.scatter(s.x + ranges[ok] * np.cos(heading),
                       s.y + ranges[ok] * np.sin(heading),
                       s=size, c=color, label=label)

        ax.arrow(s.x, s.y, 0.8 * np.cos(s.yaw), 0.8 * np.sin(s.yaw),
                 color="red", head_width=0.3)
        ax.set(xlim=(s.x - half, s.x + half), ylim=(s.y - half, s.y + half),
               title=f"scan {idx}", xlabel="x (m)", ylabel="y (m)")
        ax.set_aspect("equal")
    axes[0][0].legend(loc="upper right", fontsize=8)
    finish_figure(fig, "03 scans on map")


def continuous_components(z, z_star, p, max_range):
    """Weighted hit / short / rand densities (the max component is a point mass)."""
    s, lam = p["sigma_hit"], p["lambda_short"]
    inside = (z >= 0.0) & (z < max_range)

    norm = normal_cdf((max_range - z_star) / s) - normal_cdf(-z_star / s)
    hit = np.exp(-0.5 * ((z - z_star) / s) ** 2) / (np.sqrt(2 * np.pi) * s * np.maximum(norm, 1e-12))

    denom = -np.expm1(-lam * z_star)
    short_ok = (z >= 0.0) & (z < z_star) & (denom > 1e-12)
    short = np.where(short_ok, lam * np.exp(-lam * np.where(short_ok, z, 0.0))
                     / np.maximum(denom, 1e-12), 0.0)

    rand = np.full_like(z, 1.0 / max_range)
    return (np.where(inside, p["z_hit"] * hit, 0.0),
            np.where(inside, p["z_short"] * short, 0.0),
            np.where(inside, p["z_rand"] * rand, 0.0))


def plot_fit_check(d, p, ll_history, max_range):
    """7: fitted mixture over the residual histogram, plus the EM convergence curve."""
    res, keep = residuals_without_max(d)
    z_star_pool = d["z_star"][keep]
    rng = np.random.default_rng(0)
    z_star_sample = rng.choice(z_star_pool, min(2000, len(z_star_pool)), replace=False)

    grid = np.linspace(-1.0, 1.0, 401)
    z = z_star_sample[None, :] + grid[:, None]
    hit, short, rand = continuous_components(z, z_star_sample[None, :], p, max_range)
    # Average over the sampled z*, then rescale: the histogram excludes max-range beams.
    scale = 1.0 / max(1.0 - p["z_max"], 1e-12)
    curves = {"hit": hit, "short": short, "rand": rand}
    curves = {k: scale * v.mean(axis=1) for k, v in curves.items()}

    fig, ax = plt.subplots(1, 2, figsize=(12, 5))
    bins = np.linspace(-1.0, 1.0, 101)
    counts, _ = np.histogram(res, bins=bins)
    density = counts / (len(res) * (bins[1] - bins[0]))  # normalised by ALL non-max beams
    ax[0].bar(0.5 * (bins[:-1] + bins[1:]), density, width=bins[1] - bins[0],
              alpha=0.4, label="data")
    ax[0].plot(grid, sum(curves.values()), "r", label="fitted mixture")
    for name, curve in curves.items():
        ax[0].plot(grid, curve, "--", lw=1, label=name)
    ax[0].set(xlabel="z - z* (m)", ylabel="density (1/m)", yscale="log",
              title="fit vs data (max-range beams excluded)")
    ax[0].set_ylim(1e-4, 3.0 * max(density.max(), sum(curves.values()).max()))
    ax[0].legend(fontsize=8)

    ax[1].plot(np.arange(1, len(ll_history) + 1), ll_history, ".-")
    ax[1].set(xlabel="EM iteration", ylabel="log-likelihood", title="EM convergence")
    finish_figure(fig, "07 fit check")


def run_data_plots(scans, map_msg, max_range):
    """Plots 01 and 03: shown before EM so they are available even if EM misbehaves."""
    d = flatten_scans(scans, max_range)
    print_residual_summary(d, map_msg, max_range)
    plot_scatter_and_residuals(d, max_range)
    plot_scans_on_map(scans, map_msg, max_range)
    print(f"{LOG} Showing plots; close all windows to continue.", flush=True)
    plt.show()
    return d


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def read_bag(bag_path):
    bag = BagReader(bag_path)
    tf_store = TFStore()
    map_msg = None
    scan_messages = []
    counts = defaultdict(int)

    for topic, data in bag.iter_messages():
        if topic not in (MAP_TOPIC, TF_TOPIC, TF_STATIC_TOPIC, SCAN_TOPIC):
            continue
        msg = deserialize_message(data, get_message(bag.topic_types[topic]))
        counts[topic] += 1

        if topic == MAP_TOPIC:
            map_msg = msg
            print(f"{LOG} Received {MAP_TOPIC}", flush=True)
        elif topic == TF_TOPIC:
            tf_store.add_tf_message(msg, is_static=False)
            if counts[topic] % 2000 == 0:
                print(f"{LOG} Processed {counts[topic]} {TF_TOPIC} messages", flush=True)
        elif topic == TF_STATIC_TOPIC:
            tf_store.add_tf_message(msg, is_static=True)
        else:
            scan_messages.append(msg)
            if counts[topic] % 500 == 0:
                print(f"{LOG} Collected {counts[topic]} {SCAN_TOPIC} messages", flush=True)

    tf_store.finalize()
    print(
        f"{LOG} Finished reading bag: {counts[SCAN_TOPIC]} scans, "
        f"{counts[TF_TOPIC]} tf, {counts[TF_STATIC_TOPIC]} tf_static, "
        f"{counts[MAP_TOPIC]} maps",
        flush=True,
    )
    return map_msg, tf_store, scan_messages


def downsample_scans(scan_messages, max_scans):
    if max_scans <= 0 or len(scan_messages) <= max_scans:
        return scan_messages
    skip = max(0, math.ceil(len(scan_messages) / max_scans) - 1)
    kept = scan_messages[:: skip + 1][:max_scans]
    print(f"{LOG} Downsampled to {len(kept)} scans (skip={skip})", flush=True)
    return kept


def main():
    print(f"{LOG} Opening bag and reading messages...", flush=True)
    print(
        f"{LOG} Settings: SCAN_BEAM_STRIDE={SCAN_BEAM_STRIDE}, "
        f"MAX_SCAN_MESSAGES_TO_USE={MAX_SCAN_MESSAGES_TO_USE}",
        flush=True,
    )

    map_msg, tf_store, scan_messages = read_bag(BAG_PATH)
    if map_msg is None:
        raise RuntimeError(f"No message was received on {MAP_TOPIC}.")
    scan_messages = downsample_scans(scan_messages, MAX_SCAN_MESSAGES_TO_USE)

    resolution = map_msg.info.resolution
    range_method = build_range_method(map_msg, MAX_RANGE)  # build once

    print(f"{LOG} Computing expected ranges for {len(scan_messages)} scans...", flush=True)
    scans = []
    for scan_msg in scan_messages:
        result = compute_expected_ranges_from_map(
            scan_msg, range_method, tf_store,
            MAP_FRAME, LASER_FRAME, MAX_RANGE, beam_stride=SCAN_BEAM_STRIDE,
        )
        if result is None:
            continue
        scans.append(result)
        if len(scans) % 100 == 0:
            print(f"{LOG} Built expected ranges for {len(scans)} scans", flush=True)

    if not scans:
        raise RuntimeError(
            "No scans could be matched with a map pose. Check the TF frames "
            "and that the scan stamps fall inside the recorded /tf time range."
        )

    print(
        f"{LOG} Scan range_min={scan_messages[0].range_min}, range_max={scan_messages[0].range_max}",
        flush=True,
    )
    plot_data = run_data_plots(scans, map_msg, MAX_RANGE) if DEBUG_PLOTS else None

    flat_ranges = np.concatenate([s.observed for s in scans])
    flat_expected = np.concatenate([s.expected for s in scans])
    print(
        f"{LOG} Estimating sensor model from {len(scans)} scans / "
        f"{len(flat_ranges)} beams (sigma floor {0.5 * resolution:.4f} m)...",
        flush=True,
    )

    estimate, ll_history = estimate_sensor_model(
        flat_ranges,
        flat_expected,
        MAX_RANGE,
        init={"z_hit": 0.75, "z_short": 0.1, "z_max": 0.05, "z_rand": 0.1,
              "sigma_hit": 0.3, "lambda_short": 1.0},
        sigma_min=0.5 * resolution,
    )
    print_params(estimate, "Final estimated sensor model parameters:")

    if DEBUG_PLOTS:
        plot_fit_check(plot_data, estimate, ll_history, MAX_RANGE)
        plt.show()


if __name__ == "__main__":
    main()