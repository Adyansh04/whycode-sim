#!/usr/bin/env python3
"""Compare detector output against ground truth in a recorded bag.

Joins /whycon/poses and /marker_ground_truth on timestamp and marker id, then reports:

  * per-axis position error. Per-axis rather than Euclidean because an axis permutation
    then shows as one large component and two small ones
  * detection rate over frames ground truth says the marker was visible in
  * detection rate bucketed by apparent size and viewing angle -- the curve the benchmark
    exists to measure
  * false positives, and misses annotated with marker size, separating "too small to
    detect" from "should have worked"

    python3 compare_gt.py <bag_path> [--detections /whycon/poses]
"""

import argparse
import bisect
import math
import sys
from collections import defaultdict

SIZE_BUCKETS = ((0, 20), (20, 40), (40, 80), (80, 160), (160, 10**6))
ANGLE_BUCKETS = ((0, 15), (15, 30), (30, 45), (45, 60), (60, 90))


def read_bag(path, detection_topic, truth_topic):
    """Yield (topic, message) for the two topics of interest, in log order."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in (detection_topic, truth_topic):
        if topic not in types:
            raise SystemExit(f"{path} has no topic {topic}. Present: {sorted(types)}")

    wanted = {detection_topic, truth_topic}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in wanted:
            yield topic, deserialize_message(data, get_message(types[topic]))


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def join_nearest(left, right, tolerance_ns):
    """Pair each left stamp with the closest right stamp inside `tolerance_ns`.

    Not exact equality: the renderer stamps images on clean tick boundaries while the
    simulation clock drifts a few hundred nanoseconds, so one frame carries two
    representations. A tolerance well under the frame interval cannot pair the wrong frame.
    """
    right_stamps = sorted(right)
    pairs = []
    for stamp in sorted(left):
        index = bisect.bisect_left(right_stamps, stamp)
        best, best_delta = None, None
        for candidate in right_stamps[max(0, index - 1) : index + 2]:
            delta = abs(candidate - stamp)
            if best_delta is None or delta < best_delta:
                best, best_delta = candidate, delta
        if best is not None and best_delta <= tolerance_ns:
            pairs.append((stamp, best, best_delta))
    return pairs


def bucket_of(value, buckets):
    for low, high in buckets:
        if low <= value < high:
            return f"{low}-{high}"
    return f">{buckets[-1][1]}"


def summarise(errors):
    """Mean and RMS of a list of signed errors."""
    if not errors:
        return 0.0, 0.0
    mean = sum(errors) / len(errors)
    rms = math.sqrt(sum(e * e for e in errors) / len(errors))
    return mean, rms


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="path to the recorded bag directory")
    parser.add_argument("--detections", default="/whycon/poses")
    parser.add_argument("--truth", default="/marker_ground_truth")
    parser.add_argument(
        "--tolerance-ms",
        type=float,
        default=15.0,
        help="max stamp difference when pairing frames (default: half a 30 Hz period)",
    )
    args = parser.parse_args(argv)

    detections_by_stamp = {}
    truth_by_stamp = {}
    for topic, message in read_bag(args.bag, args.detections, args.truth):
        if topic == args.detections:
            detections_by_stamp[stamp_ns(message.header)] = {
                pose.whycode_id: pose for pose in message.poses if pose.id_valid
            }
        else:
            truth_by_stamp[stamp_ns(message.header)] = message

    tolerance_ns = int(args.tolerance_ms * 1e6)
    pairs = join_nearest(detections_by_stamp, truth_by_stamp, tolerance_ns)
    if not pairs:
        raise SystemExit(
            f"no frames matched within {args.tolerance_ms} ms. Check that the detector and "
            "the ground truth are both running on simulation time."
        )

    worst = max(delta for _, _, delta in pairs)
    print(f"bag           : {args.bag}")
    print(
        f"frames        : {len(truth_by_stamp)} truth, {len(detections_by_stamp)} detection, "
        f"{len(pairs)} matched (worst stamp delta {worst / 1e6:.3f} ms)"
    )

    axis_errors = {"x": [], "y": [], "z": []}
    by_size = defaultdict(lambda: [0, 0])
    by_angle = defaultdict(lambda: [0, 0])
    misses, false_positives = [], 0
    visible_total = matched_total = 0
    duplicate_frames = []

    for detection_stamp, truth_stamp, _ in pairs:
        detections = detections_by_stamp[detection_stamp]
        truth = truth_by_stamp[truth_stamp]
        truth_ids = {m.marker_id for m in truth.markers if m.visible}

        # A layout may place the same id twice, which is only sound while the two are
        # never visible together. If that ever holds in a recorded frame, both truth
        # entries match the one detection and the rate is overstated -- so say so rather
        # than quietly averaging it in.
        visible_ids = [m.marker_id for m in truth.markers if m.visible]
        if len(visible_ids) != len(set(visible_ids)):
            duplicate_frames.append(truth_stamp)

        for marker in truth.markers:
            if not marker.visible:
                continue
            visible_total += 1
            size_bucket = bucket_of(marker.apparent_diameter_px, SIZE_BUCKETS)
            angle_bucket = bucket_of(marker.incidence_angle_deg, ANGLE_BUCKETS)
            by_size[size_bucket][1] += 1
            by_angle[angle_bucket][1] += 1

            detection = detections.get(marker.marker_id)
            if detection is None:
                misses.append(
                    (
                        marker.marker_id,
                        marker.apparent_diameter_px,
                        marker.incidence_angle_deg,
                        marker.range_m,
                    )
                )
                continue

            matched_total += 1
            by_size[size_bucket][0] += 1
            by_angle[angle_bucket][0] += 1
            axis_errors["x"].append(detection.pose.position.x - marker.pose.position.x)
            axis_errors["y"].append(detection.pose.position.y - marker.pose.position.y)
            axis_errors["z"].append(detection.pose.position.z - marker.pose.position.z)

        false_positives += len(set(detections) - truth_ids)

    print(f"visible       : {visible_total} marker-frames")
    print(
        f"detected      : {matched_total} "
        f"({100.0 * matched_total / visible_total:.1f}%)"
        if visible_total
        else ""
    )
    print(f"false positive: {false_positives}")
    if duplicate_frames:
        print(
            f"\n  WARNING: {len(duplicate_frames)} frames show two visible markers "
            "sharing an id. Ground truth cannot say which one a detection belongs to, so "
            "the rates below are optimistic. Re-check the layout with "
            "`run_sim.py --verify-path`."
        )

    print("\nposition error, detector minus truth (metres)")
    print(f"  {'axis':<6}{'mean':>10}{'rms':>10}")
    for axis in ("x", "y", "z"):
        mean, rms = summarise(axis_errors[axis])
        print(f"  {axis:<6}{mean:>10.4f}{rms:>10.4f}")

    # One axis an order of magnitude worse than the rest is a frame-convention mistake,
    # not detector noise.
    rms_values = {axis: summarise(axis_errors[axis])[1] for axis in ("x", "y", "z")}
    worst = max(rms_values, key=rms_values.get)
    others = [v for axis, v in rms_values.items() if axis != worst]
    if rms_values[worst] > 10 * max(others + [1e-9]):
        print(
            f"\n  WARNING: {worst} error dwarfs the other axes. That is what an axis "
            "permutation looks like, not detector noise. Check the ground-truth frame "
            "convention before reading anything else here."
        )

    print("\ndetection rate by apparent diameter (px)")
    for label, (hit, total) in sorted(
        by_size.items(), key=lambda kv: int(kv[0].split("-")[0])
    ):
        print(f"  {label:<12}{hit:>6}/{total:<6}  {100.0 * hit / total:5.1f}%")

    print("\ndetection rate by incidence angle (deg)")
    for label, (hit, total) in sorted(
        by_angle.items(), key=lambda kv: int(kv[0].split("-")[0])
    ):
        print(f"  {label:<12}{hit:>6}/{total:<6}  {100.0 * hit / total:5.1f}%")

    if misses:
        print(f"\nmisses ({len(misses)}), largest first")
        for marker_id, diameter, angle, range_m in sorted(misses, key=lambda m: -m[1])[
            :15
        ]:
            print(
                f"  id={marker_id:<4} {diameter:6.1f} px  {angle:5.1f} deg  {range_m:5.2f} m"
            )

    return 0


if __name__ == "__main__":
    sys.exit(main())
