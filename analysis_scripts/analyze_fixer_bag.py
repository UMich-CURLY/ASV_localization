#!/usr/bin/env python3
"""Shared helpers for the DRIFT bag analyzers (quaternion math, interpolation,
statistics, and bag-path resolution).

Quaternion convention: [w, x, y, z] ("Eigen order"), matching the rest of
the toolchain (see analyze_multi_fixer_bag.py's own docstring).
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

EPSILON = 1e-9


# --------------------------------------------------------------------------
# Quaternion / angle helpers
# --------------------------------------------------------------------------

def quaternion_array(orientation) -> np.ndarray:
    """Extract [w, x, y, z] from a geometry_msgs/Quaternion-like object."""
    return np.array(
        [orientation.w, orientation.x, orientation.y, orientation.z],
        dtype=float,
    )


def quaternion_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if norm <= EPSILON:
        raise ValueError("Cannot normalize a near-zero quaternion.")
    return q / norm


def quaternion_inverse(q: np.ndarray) -> np.ndarray:
    """Inverse (= conjugate for unit quaternions) of [w, x, y, z]."""
    w, x, y, z = q
    return np.array([w, -x, -y, -z], dtype=float)


def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product q1 * q2, both [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quaternion_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix from a [w, x, y, z] quaternion (need not be
    pre-normalized; normalized internally)."""
    w, x, y, z = quaternion_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def quaternion_yaw(q: np.ndarray) -> float:
    """Z-axis Euler angle (yaw) of a [w, x, y, z] quaternion, radians."""
    w, x, y, z = q
    norm_sq = w * w + x * x + y * y + z * z
    if norm_sq <= EPSILON:
        return 0.0
    w, x, y, z = w / math.sqrt(norm_sq), x / math.sqrt(norm_sq), y / math.sqrt(norm_sq), z / math.sqrt(norm_sq)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_quaternion(yaw_rad: float) -> np.ndarray:
    """[w, x, y, z] quaternion for a pure yaw (Z-axis) rotation."""
    half = 0.5 * yaw_rad
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=float)


def wrap_angle(angle):
    """Wrap angle(s) in radians to (-pi, pi]. Works on scalars or ndarrays."""
    return -((-angle + np.pi) % (2 * np.pi) - np.pi)


# --------------------------------------------------------------------------
# Time / interpolation helpers
# --------------------------------------------------------------------------

def stamp_seconds(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def interpolate(x: np.ndarray, y: np.ndarray, x_new: np.ndarray) -> np.ndarray:
    """Linear interpolation of y(x) at x_new. y may be 1-D or 2-D (N, k);
    x must be increasing. Out-of-range queries clamp to the nearest edge
    value (matches np.interp's default behavior), which is safe here since
    every call site already restricts x_new to the overlapping time range
    of its sources before calling this.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    x_new = np.asarray(x_new, dtype=float)
    if y.ndim == 1:
        return np.interp(x_new, x, y)
    return np.column_stack([np.interp(x_new, x, y[:, col]) for col in range(y.shape[1])])


def nearest_gap(source_times: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    """For each query time, the absolute gap (seconds) to the nearest entry
    in source_times (which must be sorted increasing)."""
    source_times = np.asarray(source_times, dtype=float)
    query_times = np.asarray(query_times, dtype=float)
    idx = np.searchsorted(source_times, query_times)
    idx_lo = np.clip(idx - 1, 0, len(source_times) - 1)
    idx_hi = np.clip(idx, 0, len(source_times) - 1)
    gap_lo = np.abs(query_times - source_times[idx_lo])
    gap_hi = np.abs(query_times - source_times[idx_hi])
    return np.minimum(gap_lo, gap_hi)


# --------------------------------------------------------------------------
# Statistics helpers
# --------------------------------------------------------------------------

def scalar_statistics(values: np.ndarray) -> dict[str, float]:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if len(values) == 0:
        return {"mean": float("nan"), "rmse": float("nan"), "p95_abs": float("nan"), "max_abs": float("nan")}
    return {
        "mean": float(np.mean(values)),
        "rmse": float(np.sqrt(np.mean(values ** 2))),
        "p95_abs": float(np.percentile(np.abs(values), 95)),
        "max_abs": float(np.max(np.abs(values))),
    }


def timing_statistics(messages: np.ndarray) -> dict[str, float]:
    """messages[:, 0] is header stamp seconds, assumed time-ordered."""
    stamps = messages[:, 0]
    if len(stamps) < 2:
        return {"rate": float("nan"), "p95_ms": float("nan"), "gaps_20_ms": 0}
    dt = np.diff(stamps)
    dt = dt[dt > 0]
    duration = stamps[-1] - stamps[0]
    rate = (len(stamps) - 1) / duration if duration > 0 else float("nan")
    return {
        "rate": float(rate),
        "p95_ms": float(np.percentile(dt, 95) * 1000.0) if len(dt) else float("nan"),
        "gaps_20_ms": int(np.sum(dt > 0.020)),
    }


def centered_heading_lag(
    evaluation_time: np.ndarray,
    pose_yaw: np.ndarray,
    reference_time: np.ndarray,
    reference_yaw: np.ndarray,
    max_lag_s: float = 0.3,
    step_s: float = 0.005,
) -> tuple[float, float]:
    """Estimate the time shift (seconds) that best aligns pose_yaw (sampled
    at evaluation_time) to the reference heading series, by minimizing
    circular RMSE over a small grid search. Returns (best_lag_s, rmse at
    that lag). Positive lag means pose_yaw is delayed relative to the
    reference (pose_yaw(t) matches reference(t - lag)).
    """
    evaluation_time = np.asarray(evaluation_time, dtype=float)
    pose_yaw = np.asarray(pose_yaw, dtype=float)
    reference_time = np.asarray(reference_time, dtype=float)
    reference_yaw = np.asarray(reference_yaw, dtype=float)

    lo = reference_time[0]
    hi = reference_time[-1]
    lags = np.arange(-max_lag_s, max_lag_s + step_s / 2.0, step_s)

    best_lag = 0.0
    best_rmse = float("inf")
    for lag in lags:
        query = evaluation_time - lag
        valid = (query >= lo) & (query <= hi)
        if np.count_nonzero(valid) < max(3, len(evaluation_time) // 4):
            continue
        shifted_reference = np.interp(query[valid], reference_time, reference_yaw)
        error = wrap_angle(pose_yaw[valid] - shifted_reference)
        rmse = float(np.sqrt(np.mean(error ** 2)))
        if rmse < best_rmse:
            best_rmse = rmse
            best_lag = float(lag)

    if not math.isfinite(best_rmse):
        # Fall back to zero lag over whatever overlap exists.
        query = evaluation_time
        valid = (query >= lo) & (query <= hi)
        shifted_reference = np.interp(query[valid], reference_time, reference_yaw)
        error = wrap_angle(pose_yaw[valid] - shifted_reference)
        best_lag = 0.0
        best_rmse = float(np.sqrt(np.mean(error ** 2))) if np.any(valid) else float("nan")

    return best_lag, best_rmse


def fit_body_fixed_position_offset(
    position: np.ndarray,
    reference_position_xy: np.ndarray,
    reference_yaw: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Least-squares fit of a constant body-frame 2D offset d such that

        reference_position_xy[i] ~= position[i] + R(reference_yaw[i]) @ d

    for all i, with R(yaw) the standard body-to-world rotation
    [[cos, -sin], [sin, cos]] (consistent with the body/world convention
    used elsewhere in analyze_multi_fixer_bag.py for GNSS twist rotation).

    Returns (predicted_reference_position, d, residual_norm_per_sample).
    """
    position = np.asarray(position, dtype=float)[:, :2]
    reference_position_xy = np.asarray(reference_position_xy, dtype=float)[:, :2]
    reference_yaw = np.asarray(reference_yaw, dtype=float)
    n = len(reference_yaw)

    cosine = np.cos(reference_yaw)
    sine = np.sin(reference_yaw)

    # Design matrix stacking each sample's 2x2 body-to-world rotation block.
    A = np.zeros((2 * n, 2))
    A[0::2, 0] = cosine
    A[0::2, 1] = -sine
    A[1::2, 0] = sine
    A[1::2, 1] = cosine

    rhs = (reference_position_xy - position).reshape(-1)

    d, _residuals, _rank, _sv = np.linalg.lstsq(A, rhs, rcond=None)

    predicted = position + np.column_stack(
        (cosine * d[0] - sine * d[1], sine * d[0] + cosine * d[1])
    )
    residual_norm = np.linalg.norm(reference_position_xy - predicted, axis=1)
    return predicted, d, residual_norm


_WGS84_SEMI_MAJOR_M = 6378137.0


def reference_positions_to_local(reference: np.ndarray) -> tuple[np.ndarray, str]:
    """reference[:, 2:5] holds /pose's position field. CORRECTED (was wrong
    in an earlier version of this file): this is raw geodetic
    longitude/latitude/altitude in degrees/degrees/meters -- e.g.
    (-83.7013..., 42.2752..., 195.3...) -- not already a local ENU frame.
    Verified directly against bag content: x/y stay within a few 1e-4
    degrees of a Michigan-area lat/lon the whole run, only visible at full
    float precision (a 2-decimal print of the range looks constant and was
    the tell that something was off).

    Converts to a local ENU (east, north, up) frame with the first sample
    as the origin, using an equirectangular approximation (flat-earth,
    correct to well under a centimeter over the runs analyzed here, which
    span tens of meters). Column order is (east, north, up) to match the
    (x, y) = (east, north) convention used everywhere else in this module.
    """
    geo = np.asarray(reference[:, 2:5], dtype=float)
    lon0, lat0, alt0 = geo[0]
    lat0_rad = math.radians(lat0)
    east = np.radians(geo[:, 0] - lon0) * _WGS84_SEMI_MAJOR_M * math.cos(lat0_rad)
    north = np.radians(geo[:, 1] - lat0) * _WGS84_SEMI_MAJOR_M
    up = geo[:, 2] - alt0
    return np.column_stack((east, north, up)), "ENU meters (equirectangular, first dual-GNSS sample as origin)"


def resolve_bag_path(path: Path) -> Path:
    """Accept either a rosbag2 directory (containing metadata.yaml) or a
    direct path to its .db3 file, and return the directory SequentialReader
    expects."""
    path = Path(path)
    if path.is_file():
        if path.suffix == ".db3":
            return path.parent
        raise SystemExit(f"Not a rosbag2 directory or .db3 file: {path}")
    if path.is_dir():
        if (path / "metadata.yaml").exists():
            return path
        raise SystemExit(f"No metadata.yaml found in bag directory: {path}")
    raise SystemExit(f"Bag path does not exist: {path}")
