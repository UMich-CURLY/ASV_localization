#!/usr/bin/env python3
"""Compare all localization pose/twist variants recorded in one ROS 2 bag.

Example:
  source /opt/ros/humble/setup.bash
  ./tools/analyze_multi_fixer_bag.py BAG --body-to-gps -0.381 0.3937 0.3048

The comparison is evaluated at common dual-GNSS timestamps so differences in
publisher rate and startup time do not bias one localization variant.
Quaternion command-line values use Eigen order [w, x, y, z].
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

try:
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message
except ModuleNotFoundError as exc:
    raise SystemExit(
        "ROS 2 Python modules were not found. Run:\n"
        "  source /opt/ros/humble/setup.bash\n"
        "and then run this script again."
    ) from exc

from analyze_fixer_bag import (
    EPSILON,
    centered_heading_lag,
    fit_body_fixed_position_offset,
    interpolate,
    nearest_gap,
    quaternion_array,
    quaternion_inverse,
    quaternion_multiply,
    quaternion_normalize,
    quaternion_rotation_matrix,
    quaternion_yaw,
    reference_positions_to_local,
    resolve_bag_path,
    scalar_statistics,
    stamp_seconds,
    timing_statistics,
    wrap_angle,
    yaw_quaternion,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare every /localization/pose_* branch against dual-GNSS "
            "/pose and aligned AHRS heading."
        )
    )
    parser.add_argument("bag", type=Path, help="Rosbag directory or .db3 file")
    parser.add_argument("--reference-topic", default="/pose")
    parser.add_argument("--orientation-topic", default="/ekf/imu/data")
    parser.add_argument("--propagation-topic", default="/imu/data_raw")
    parser.add_argument("--gnss-twist-topic", default="/twist_gnss")
    parser.add_argument(
        "--pose-topics",
        nargs="+",
        help=(
            "Localization pose topics to compare; otherwise every recorded "
            "/localization/pose_* topic is used"
        ),
    )
    parser.add_argument(
        "--alignment-deg",
        type=float,
        help=(
            "World-alignment yaw override; otherwise reconstructed from the "
            "first synchronized dual-GNSS/AHRS samples"
        ),
    )
    parser.add_argument(
        "--alignment-samples",
        type=int,
        default=5,
        help="Number of initial synchronized samples used to reconstruct alignment",
    )
    parser.add_argument(
        "--drift-anchor-samples",
        type=int,
        default=5,
        help=(
            "Number of initial common-timestamp samples averaged, per series, "
            "to zero that series' heading before computing error. This makes "
            "the comparison test relative drift against dual-GNSS rather than "
            "absolute heading, so it is insensitive to any constant offset in "
            "world-alignment reconstruction. Use --absolute-heading to disable."
        ),
    )
    parser.add_argument(
        "--absolute-heading",
        action="store_true",
        help=(
            "Compare absolute heading instead of drift-from-common-start. "
            "Sensitive to world-alignment reconstruction accuracy; kept for "
            "diagnosing alignment issues, not recommended for method comparison."
        ),
    )
    parser.add_argument(
        "--imu-to-body",
        nargs=4,
        type=float,
        metavar=("W", "X", "Y", "Z"),
        default=(0.0, 0.0, 0.0, 1.0),
        help="IMU-to-body quaternion in Eigen [w x y z] order",
    )
    parser.add_argument(
        "--body-to-gps",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help=(
            "Configured body-to-GPS lever arm for position checking and "
            "GNSS velocity correction"
        ),
    )
    parser.add_argument(
        "--sync-tolerance",
        type=float,
        default=0.075,
        help="Maximum nearest-message difference in seconds",
    )
    parser.add_argument(
        "--alignment-tolerance",
        type=float,
        default=0.05,
        help="Maximum pose/AHRS difference for initial alignment in seconds",
    )
    parser.add_argument(
        "--moving-speed",
        type=float,
        default=0.5,
        help="GNSS horizontal-speed threshold for moving metrics in m/s",
    )
    parser.add_argument(
        "--warmup-seconds",
        type=float,
        default=0.0,
        help=(
            "Exclude this many seconds from the start of each metric's own "
            "common-timestamp evaluation window (heading, twist, and -- since "
            "it reuses the heading window -- position) before computing any "
            "statistic. Distinct from --drift-anchor-samples: anchoring "
            "removes a constant offset but still counts early samples in the "
            "RMSE; this drops them entirely, for excluding a genuine startup "
            "transient (filter covariance still converging, GPS reference "
            "still stabilizing) rather than a fixed misalignment. Default 0 "
            "preserves prior behavior exactly."
        ),
    )
    return parser.parse_args()


def apply_warmup(indices: np.ndarray, times: np.ndarray, warmup_seconds: float):
    """Drop entries within warmup_seconds of this window's own start."""
    if warmup_seconds <= 0.0 or len(times) == 0:
        return indices, times
    keep = (times - times[0]) >= warmup_seconds
    if not np.any(keep):
        raise SystemExit(
            f"--warmup-seconds {warmup_seconds:g} excludes the entire "
            f"{times[-1] - times[0]:.1f}s common window."
        )
    return indices[keep], times[keep]


def variant_label(topic: str) -> str:
    return topic.rsplit("/", 1)[-1].removeprefix("pose_")


def corresponding_twist_topic(pose_topic: str) -> str:
    namespace, name = pose_topic.rsplit("/", 1)
    if name.startswith("pose_"):
        name = "twist_" + name[len("pose_") :]
    return f"{namespace}/{name}"


def discover_pose_topics(topic_types: dict[str, str], requested):
    if requested:
        missing = [topic for topic in requested if topic not in topic_types]
        if missing:
            raise SystemExit(
                f"Requested pose topics are absent: {', '.join(missing)}"
            )
        return requested

    topics = [
        topic
        for topic, message_type in topic_types.items()
        if topic.startswith("/localization/pose_")
        and message_type == "geometry_msgs/msg/PoseStamped"
    ]
    if not topics:
        raise SystemExit("No /localization/pose_* topics were found in the bag.")
    return sorted(topics, key=variant_label)


def load_records(bag: Path, args: argparse.Namespace):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag), storage_id="sqlite3"),
        ConverterOptions("", ""),
    )
    topic_types = {
        entry.name: entry.type for entry in reader.get_all_topics_and_types()
    }
    pose_topics = discover_pose_topics(topic_types, args.pose_topics)
    twist_topics = {
        pose_topic: corresponding_twist_topic(pose_topic)
        for pose_topic in pose_topics
        if corresponding_twist_topic(pose_topic) in topic_types
    }

    required = {
        args.reference_topic,
        args.orientation_topic,
        args.propagation_topic,
        *pose_topics,
    }
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise SystemExit(f"Required topics are absent: {', '.join(sorted(missing))}")

    requested_topics = {
        *required,
        *twist_topics.values(),
        args.gnss_twist_topic,
        "/rosout",
    }
    message_classes = {
        topic: get_message(topic_types[topic])
        for topic in requested_topics
        if topic in topic_types
    }
    records = defaultdict(list)
    logs = []

    while reader.has_next():
        topic, serialized, bag_stamp = reader.read_next()
        if topic not in message_classes:
            continue
        message = deserialize_message(serialized, message_classes[topic])
        bag_time = bag_stamp * 1e-9

        if topic == "/rosout":
            logs.append((message.level, message.name, message.msg))
        elif topic == args.reference_topic:
            pose = message.pose.pose
            covariance = message.pose.covariance
            records[topic].append(
                (
                    stamp_seconds(message.header),
                    bag_time,
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    *quaternion_array(pose.orientation),
                    covariance[0],
                    covariance[7],
                    covariance[35],
                )
            )
        elif topic in pose_topics:
            pose = message.pose
            records[topic].append(
                (
                    stamp_seconds(message.header),
                    bag_time,
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    *quaternion_array(pose.orientation),
                )
            )
        elif topic in {args.orientation_topic, args.propagation_topic}:
            records[topic].append(
                (
                    stamp_seconds(message.header),
                    bag_time,
                    *quaternion_array(message.orientation),
                    message.orientation_covariance[0],
                    message.orientation_covariance[8],
                    message.angular_velocity.x,
                    message.angular_velocity.y,
                    message.angular_velocity.z,
                )
            )
        elif topic in twist_topics.values():
            twist = message.twist
            records[topic].append(
                (
                    stamp_seconds(message.header),
                    bag_time,
                    twist.linear.x,
                    twist.linear.y,
                    twist.linear.z,
                    twist.angular.z,
                )
            )
        elif topic == args.gnss_twist_topic:
            twist = message.twist.twist
            records[topic].append(
                (
                    stamp_seconds(message.header),
                    bag_time,
                    twist.linear.x,
                    twist.linear.y,
                    twist.linear.z,
                )
            )

    arrays = {
        topic: np.asarray(values, dtype=float) for topic, values in records.items()
    }
    empty = [topic for topic in required if topic not in arrays or len(arrays[topic]) < 2]
    if empty:
        raise SystemExit(
            f"Required topics are empty or too short: {', '.join(sorted(empty))}"
        )
    return arrays, logs, pose_topics, twist_topics


def reconstruct_alignment(
    reference: np.ndarray,
    orientation: np.ndarray,
    imu_from_body: np.ndarray,
    sample_count: int,
    tolerance: float,
) -> tuple[float, np.ndarray]:
    differences = []
    orientation_time = orientation[:, 0]
    for pose in reference:
        index = int(np.searchsorted(orientation_time, pose[0]))
        candidates = [
            candidate
            for candidate in (index - 1, index)
            if 0 <= candidate < len(orientation)
        ]
        if not candidates:
            continue
        closest = min(candidates, key=lambda candidate: abs(orientation_time[candidate] - pose[0]))
        if abs(orientation_time[closest] - pose[0]) > tolerance:
            continue
        if orientation[closest, 6] < 0.0:
            continue

        pose_yaw = quaternion_yaw(pose[5:9])
        imu_body = quaternion_multiply(
            quaternion_normalize(orientation[closest, 2:6]), imu_from_body
        )
        differences.append(float(wrap_angle(pose_yaw - quaternion_yaw(imu_body))))
        if len(differences) >= sample_count:
            break

    if len(differences) < sample_count:
        raise SystemExit(
            f"Only {len(differences)} initial alignment samples matched; "
            "supply --alignment-deg or increase --alignment-tolerance."
        )
    differences = np.asarray(differences)
    alignment = circular_mean(differences)
    return alignment, differences


def circular_mean(angles: np.ndarray) -> float:
    return math.atan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))


def yaw_series(messages: np.ndarray, columns: slice) -> np.ndarray:
    return np.unwrap(
        np.array([quaternion_yaw(quaternion) for quaternion in messages[:, columns]])
    )


def common_heading_samples(
    records,
    reference_topic,
    orientation_topic,
    pose_topics,
    gnss_twist_topic,
    tolerance,
):
    reference = records[reference_topic]
    sources = [records[orientation_topic], *(records[topic] for topic in pose_topics)]
    if gnss_twist_topic in records:
        sources.append(records[gnss_twist_topic])

    start = max(source[0, 0] for source in sources)
    end = min(source[-1, 0] for source in sources)
    selected = (reference[:, 0] >= start) & (reference[:, 0] <= end)
    indices = np.flatnonzero(selected)
    query_time = reference[indices, 0]
    valid = np.ones(len(query_time), dtype=bool)
    for source in sources:
        valid &= nearest_gap(source[:, 0], query_time) <= tolerance
    indices = indices[valid]
    if len(indices) == 0:
        raise SystemExit("No timestamps are common to all localization variants.")
    return indices, reference[indices, 0]


def stats_degrees(errors: np.ndarray) -> dict[str, float]:
    return scalar_statistics(np.degrees(errors))


def output_latency(messages: np.ndarray, propagation: np.ndarray) -> np.ndarray:
    valid = (
        (messages[:, 0] >= propagation[0, 0])
        & (messages[:, 0] <= propagation[-1, 0])
    )
    query_time = messages[valid, 0]
    propagation_record_offset = interpolate(
        propagation[:, 0], propagation[:, 1] - propagation[:, 0], query_time
    )
    return (
        messages[valid, 1]
        - messages[valid, 0]
        - propagation_record_offset
    ) * 1000.0


def main() -> int:
    args = parse_args()
    if args.alignment_samples < 1:
        raise SystemExit("--alignment-samples must be positive.")

    bag = resolve_bag_path(args.bag)
    records, logs, pose_topics, twist_topics = load_records(bag, args)
    reference = records[args.reference_topic]
    orientation = records[args.orientation_topic]
    propagation = records[args.propagation_topic]

    mount = quaternion_normalize(np.asarray(args.imu_to_body, dtype=float))
    imu_from_body = quaternion_inverse(mount)
    if args.alignment_deg is None:
        alignment_rad, alignment_samples = reconstruct_alignment(
            reference,
            orientation,
            imu_from_body,
            args.alignment_samples,
            args.alignment_tolerance,
        )
        alignment_source = f"reconstructed from {len(alignment_samples)} samples"
    else:
        alignment_rad = math.radians(args.alignment_deg)
        alignment_samples = np.array([], dtype=float)
        alignment_source = "command-line override"

    alignment = yaw_quaternion(alignment_rad)
    reference_yaw = yaw_series(reference, slice(5, 9))
    orientation_yaw = np.unwrap(
        np.array(
            [
                quaternion_yaw(
                    quaternion_multiply(
                        quaternion_multiply(alignment, quaternion_normalize(q)),
                        imu_from_body,
                    )
                )
                for q in orientation[:, 2:6]
            ]
        )
    )
    pose_yaws = {
        topic: yaw_series(records[topic], slice(5, 9)) for topic in pose_topics
    }

    reference_indices, evaluation_time = common_heading_samples(
        records,
        args.reference_topic,
        args.orientation_topic,
        pose_topics,
        args.gnss_twist_topic,
        args.sync_tolerance,
    )
    reference_indices, evaluation_time = apply_warmup(
        reference_indices, evaluation_time, args.warmup_seconds
    )
    dual_yaw = reference_yaw[reference_indices]
    ahrs_yaw = interpolate(orientation[:, 0], orientation_yaw, evaluation_time)

    anchor_count = min(args.drift_anchor_samples, len(evaluation_time))
    if args.absolute_heading:
        dual_anchor = 0.0
        ahrs_anchor = 0.0
    else:
        dual_anchor = circular_mean(dual_yaw[:anchor_count])
        ahrs_anchor = circular_mean(ahrs_yaw[:anchor_count])
    ahrs_error = wrap_angle((ahrs_yaw - ahrs_anchor) - (dual_yaw - dual_anchor))
    ahrs_stats = stats_degrees(ahrs_error)

    moving = np.ones(len(evaluation_time), dtype=bool)
    if args.gnss_twist_topic in records:
        gnss_twist = records[args.gnss_twist_topic]
        speed = np.linalg.norm(gnss_twist[:, 2:4], axis=1)
        moving = (
            interpolate(gnss_twist[:, 0], speed, evaluation_time)
            > args.moving_speed
        )

    heading_results = {}
    for topic in pose_topics:
        pose_yaw = interpolate(
            records[topic][:, 0], pose_yaws[topic], evaluation_time
        )
        pose_anchor = (
            0.0 if args.absolute_heading else circular_mean(pose_yaw[:anchor_count])
        )
        dual_error = wrap_angle((pose_yaw - pose_anchor) - (dual_yaw - dual_anchor))
        ahrs_difference = wrap_angle((pose_yaw - pose_anchor) - (ahrs_yaw - ahrs_anchor))
        result = {
            "yaw": pose_yaw,
            "dual_error": dual_error,
            "dual": stats_degrees(dual_error),
            "ahrs": stats_degrees(ahrs_difference),
            "moving": stats_degrees(dual_error[moving]) if np.any(moving) else None,
        }
        result["lag_ms"], result["lag_rmse"] = centered_heading_lag(
            evaluation_time, pose_yaw, reference[:, 0], reference_yaw
        )
        heading_results[topic] = result

    print(f"\nBag: {bag}")
    print(f"Reference heading: {args.reference_topic}")
    print(f"AHRS orientation: {args.orientation_topic}")
    print(f"Propagation: {args.propagation_topic}")
    print(
        f"World alignment: {math.degrees(alignment_rad):.3f} deg "
        f"({alignment_source})"
    )
    if len(alignment_samples):
        centered = wrap_angle(alignment_samples - alignment_rad)
        print(
            "Initial alignment spread: "
            f"{np.std(np.degrees(centered)):.3f} deg standard deviation"
        )
    print(f"IMU-to-body [w x y z]: {mount.tolist()}")
    if args.warmup_seconds > 0.0:
        print(f"Warm-up excluded: first {args.warmup_seconds:g} s of each metric's own window")
    print(
        f"Common comparison: {len(evaluation_time)} dual-GNSS samples over "
        f"{evaluation_time[-1] - evaluation_time[0]:.2f} s"
    )
    if args.absolute_heading:
        print(
            "Heading reference mode: absolute (sensitive to world-alignment "
            "reconstruction; pass no --absolute-heading to compare drift instead)"
        )
    else:
        print(
            "Heading reference mode: drift relative to each series' own first "
            f"{anchor_count} common samples (world alignment does not affect "
            "the error metrics below)"
        )

    print("\nHeading comparison against dual-GNSS [degrees]")
    print(
        f"{'Variant':<25} {'mean':>8} {'RMSE':>8} {'p95|e|':>8} "
        f"{'max|e|':>8} {'moving':>8} {'vs AHRS':>8} {'gain':>8}"
    )
    print(
        f"{'aligned_AHRS':<25} {ahrs_stats['mean']:8.3f} "
        f"{ahrs_stats['rmse']:8.3f} {ahrs_stats['p95_abs']:8.3f} "
        f"{ahrs_stats['max_abs']:8.3f} {'-':>8} {'-':>8} {'baseline':>8}"
    )
    for topic in pose_topics:
        result = heading_results[topic]
        dual = result["dual"]
        moving_rmse = result["moving"]["rmse"] if result["moving"] else float("nan")
        gain = 100.0 * (ahrs_stats["rmse"] - dual["rmse"]) / ahrs_stats["rmse"]
        print(
            f"{variant_label(topic):<25} {dual['mean']:8.3f} "
            f"{dual['rmse']:8.3f} {dual['p95_abs']:8.3f} "
            f"{dual['max_abs']:8.3f} {moving_rmse:8.3f} "
            f"{result['ahrs']['rmse']:8.3f} {gain:7.1f}%"
        )
    print(
        "gain = heading RMSE reduction relative to aligned AHRS; negative is worse"
    )
    print(
        f"moving = dual-GNSS heading RMSE above {args.moving_speed:g} m/s; "
        "vs AHRS = direct heading-disagreement RMSE"
    )

    yaw_variance = reference[reference_indices, 11]
    valid_variance = yaw_variance[np.isfinite(yaw_variance) & (yaw_variance >= 0.0)]
    if len(valid_variance):
        sigma = np.degrees(np.sqrt(valid_variance))
        print(
            "Dual-GNSS reported yaw sigma: "
            f"median={np.median(sigma):.3f} deg, "
            f"p95={np.percentile(sigma, 95):.3f} deg"
        )

    print("\nHeading stability by common time quarter (RMSE deg)")
    print(f"{'Variant':<25} {'Q1':>8} {'Q2':>8} {'Q3':>8} {'Q4':>8} {'lag ms':>10}")
    edges = np.linspace(evaluation_time[0], evaluation_time[-1], 5)
    for topic in pose_topics:
        errors = heading_results[topic]["dual_error"]
        quarters = []
        for index in range(4):
            upper = edges[index + 1] + (1e-6 if index == 3 else 0.0)
            selected = (evaluation_time >= edges[index]) & (evaluation_time < upper)
            quarters.append(stats_degrees(errors[selected])["rmse"])
        print(
            f"{variant_label(topic):<25} "
            + " ".join(f"{value:8.3f}" for value in quarters)
            + f" {heading_results[topic]['lag_ms'] * 1000.0:10.1f}"
        )

    reference_position, position_description = reference_positions_to_local(reference)
    dual_position = reference_position[reference_indices]
    print(f"\nPosition consistency ({position_description})")
    print(
        f"{'Variant':<25} {'offset body x':>14} {'offset body y':>14} "
        f"{'resid RMSE':>12} {'resid p95':>10} {'lever diff':>11}"
    )
    expected_offset = None
    if args.body_to_gps is not None:
        expected_offset = -np.asarray(args.body_to_gps[:2], dtype=float)
    for topic in pose_topics:
        position = interpolate(
            records[topic][:, 0], records[topic][:, 2:4], evaluation_time
        )
        _, body_offset, residual = fit_body_fixed_position_offset(
            position, dual_position[:, :2], dual_yaw
        )
        residual_stats = scalar_statistics(residual)
        lever_difference = (
            np.linalg.norm(body_offset - expected_offset)
            if expected_offset is not None
            else float("nan")
        )
        print(
            f"{variant_label(topic):<25} {body_offset[0]:14.4f} "
            f"{body_offset[1]:14.4f} {residual_stats['rmse']:12.4f} "
            f"{residual_stats['p95_abs']:10.4f} {lever_difference:11.4f}"
        )
    if expected_offset is not None:
        print(
            "Expected fixer relative to GPS in body XY: "
            f"[{expected_offset[0]:+.4f}, {expected_offset[1]:+.4f}] m"
        )
    print(
        "Position is internal consistency when the same GNSS position feeds "
        "the filters, not independent accuracy."
    )

    available_twists = {
        pose_topic: twist_topic
        for pose_topic, twist_topic in twist_topics.items()
        if twist_topic in records
    }
    if args.gnss_twist_topic in records and available_twists:
        gnss_twist = records[args.gnss_twist_topic]
        twist_sources = [
            reference,
            propagation,
            *(records[topic] for topic in available_twists.values()),
        ]
        twist_start = max(source[0, 0] for source in twist_sources)
        twist_end = min(source[-1, 0] for source in twist_sources)
        selected = (
            (gnss_twist[:, 0] >= twist_start)
            & (gnss_twist[:, 0] <= twist_end)
        )
        twist_indices = np.flatnonzero(selected)
        twist_time = gnss_twist[twist_indices, 0]
        valid = np.ones(len(twist_time), dtype=bool)
        for source in twist_sources:
            valid &= nearest_gap(source[:, 0], twist_time) <= args.sync_tolerance
        twist_indices = twist_indices[valid]
        twist_time = gnss_twist[twist_indices, 0]
        twist_indices, twist_time = apply_warmup(
            twist_indices, twist_time, args.warmup_seconds
        )

        twist_yaw = interpolate(reference[:, 0], reference_yaw, twist_time)
        cosine = np.cos(twist_yaw)
        sine = np.sin(twist_yaw)
        gnss_velocity_enu = gnss_twist[twist_indices, 2:5]
        gnss_velocity_body = np.column_stack(
            (
                cosine * gnss_velocity_enu[:, 0]
                + sine * gnss_velocity_enu[:, 1],
                -sine * gnss_velocity_enu[:, 0]
                + cosine * gnss_velocity_enu[:, 1],
                gnss_velocity_enu[:, 2],
            )
        )
        omega_imu = interpolate(
            propagation[:, 0], propagation[:, 8:11], twist_time
        )
        omega_body = omega_imu @ quaternion_rotation_matrix(mount).T
        corrected_velocity = None
        if args.body_to_gps is not None:
            lever_arm = np.asarray(args.body_to_gps, dtype=float)
            corrected_velocity = gnss_velocity_body - np.cross(omega_body, lever_arm)

        print("\nTwist consistency on common GNSS timestamps")
        print(
            f"{'Variant':<25} {'raw RMSE':>10} {'corrected':>10} "
            f"{'corr p95':>10} {'wz RMSE':>10} {'wz corr':>10}"
        )
        for pose_topic, twist_topic in available_twists.items():
            twist = records[twist_topic]
            velocity = interpolate(twist[:, 0], twist[:, 2:5], twist_time)
            angular_z = interpolate(twist[:, 0], twist[:, 5], twist_time)
            raw_error = np.linalg.norm(
                velocity[:, :2] - gnss_velocity_body[:, :2], axis=1
            )
            raw_stats = scalar_statistics(raw_error)
            if corrected_velocity is not None:
                corrected_error = np.linalg.norm(
                    velocity[:, :2] - corrected_velocity[:, :2], axis=1
                )
                corrected_stats = scalar_statistics(corrected_error)
                corrected_rmse = corrected_stats["rmse"]
                corrected_p95 = corrected_stats["p95_abs"]
            else:
                corrected_rmse = float("nan")
                corrected_p95 = float("nan")
            angular_error = angular_z - omega_body[:, 2]
            angular_stats = scalar_statistics(angular_error)
            correlation = (
                np.corrcoef(angular_z, omega_body[:, 2])[0, 1]
                if np.std(angular_z) > EPSILON
                and np.std(omega_body[:, 2]) > EPSILON
                else float("nan")
            )
            print(
                f"{variant_label(pose_topic):<25} {raw_stats['rmse']:10.4f} "
                f"{corrected_rmse:10.4f} {corrected_p95:10.4f} "
                f"{angular_stats['rmse']:10.5f} {correlation:10.6f}"
            )
        print("Linear-velocity columns are m/s; angular-z RMSE is rad/s.")

    print("\nPublisher timing")
    print(
        f"{'Variant':<25} {'messages':>9} {'rate Hz':>9} {'p95 dt':>10} "
        f"{'>20ms':>8} {'lat med':>10} {'lat p95':>10}"
    )
    for topic in pose_topics:
        messages = records[topic]
        timing = timing_statistics(messages)
        latency = output_latency(messages, propagation)
        print(
            f"{variant_label(topic):<25} {len(messages):9d} "
            f"{timing['rate']:9.2f} {timing['p95_ms']:10.2f} "
            f"{timing['gaps_20_ms']:8d} {np.median(latency):10.2f} "
            f"{np.percentile(latency, 95):10.2f}"
        )
    print("Timing values other than rate are milliseconds.")

    warnings = [
        (name, message)
        for level, name, message in logs
        if level >= 30 and name != "rosbag2_player"
    ]
    if warnings:
        print("\nRecorded warnings/errors")
        for name, message in warnings:
            print(f"- {name}: {message}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
