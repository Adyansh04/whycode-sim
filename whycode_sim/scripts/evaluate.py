#!/usr/bin/env python3
"""Scores a detector's replay of a benchmark bag against the ground truth.

Reads replay_detector.py's output: a bag of /marker_ground_truth and the detector's poses,
either whycode_vision/msg/WhyCodePoseArray on /whycon/poses or /tf transforms from the camera's
optical frame to tag frames named `<family>:<id>` (apriltag_ros, Isaac ROS AprilTag, the ArUco
baseline). Every marker in view in every frame becomes a row of <dir>/rows.csv; <dir>/score.json
bins them by range, incidence and apparent size for each marker size.

    python3 evaluate.py <replay_dir> --scene <scene.yaml> [--match id|position]
        [--assumed-size 0.2]   (default: the size replay_detector.py gave the detector)

A detection belongs to the marker nearest in bearing, which does not depend on the size the
detector assumed. With --assumed-size (WhyCode's one diameter for every marker), a marker of
another size has its position rescaled by its true size: a circle's pose scales exactly with it.
"""

import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from whycode_sim import scene_config

# A detection and a ground-truth frame within 5 ms are the same frame
MATCH_NS = 5_000_000
GATE_DEG = 2.0  # Bearing gate between a detection and a marker
RANGE_BIN_M = 0.5
INCIDENCE_BIN_DEG = 5.0
SIZE_BUCKETS_PX = ((0, 20), (20, 40), (40, 80), (80, math.inf))
# A loop marker counts from this apparent size, at most 70 deg from head-on
VISIBLE_PX = 15.0
VISIBLE_INCIDENCE_DEG = 70.0
RELIABLE = 0.9  # Detection rate a range bin needs to count as reliable
MIN_BIN_FRAMES = 5


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def optical_to_body(v):
    """Optical (X right, Y down, Z forward) to body (X forward, Y left, Z up)."""
    return np.array([v[2], -v[0], -v[1]])


def read(bag):
    """Ground-truth frames, detections by stamp, and the stamps the detector processed."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    truth, detections, processed = [], defaultdict(list), set()
    while reader.has_next():
        topic, data, _ = reader.read_next()
        message = deserialize_message(data, types[topic])
        if topic == "/marker_ground_truth":
            truth.append((stamp_ns(message.header.stamp), message))
        elif topic == "/whycon/poses":
            stamp = stamp_ns(message.header.stamp)
            processed.add(stamp)
            for pose in message.poses:
                position = pose.pose.position
                # WhyCode's normal is its local X
                normal = rotation(pose.pose.orientation)[:, 0]
                detections[stamp].append(
                    (
                        pose.whycode_id if pose.id_valid else -1,
                        np.array([position.x, position.y, position.z]),
                        normal,
                    )
                )
        elif topic == "/detections":
            processed.add(stamp_ns(message.header.stamp))
        elif topic == "/tf":
            for transform in message.transforms:
                frame, _, tag = transform.child_frame_id.rpartition(":")
                if not frame or not tag.isdigit():
                    continue  # The robot's own frames
                t = transform.transform.translation
                detections[stamp_ns(transform.header.stamp)].append(
                    (
                        int(tag),
                        optical_to_body([t.x, t.y, t.z]),
                        optical_to_body(rotation(transform.transform.rotation)[:, 2]),
                    )
                )
    truth.sort(key=lambda entry: entry[0])
    return truth, detections, processed


def nearest(stamps, stamp):
    index = np.searchsorted(stamps, stamp)
    best = None
    for candidate in (index - 1, index):
        if 0 <= candidate < len(stamps) and abs(stamps[candidate] - stamp) <= MATCH_NS:
            if best is None or abs(stamps[candidate] - stamp) < abs(best - stamp):
                best = stamps[candidate]
    return best


def angle_deg(a, b):
    cosine = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def score(directory, scene, match, assumed_size):
    config = scene_config.load(scene)
    family = getattr(config.marker.families, config.marker.family)
    fraction = getattr(family, "black_fraction", 1.0)
    sizes = [
        marker.size_m * fraction for marker in scene_config.resolved_markers(config)
    ]

    cpu = (
        json.loads((directory / "cpu.json").read_text())
        if (directory / "cpu.json").exists()
        else {}
    )
    assumed_size = assumed_size or cpu.get("assumed_size_m")
    truth, detections, processed = read(directory / "replay")
    detection_stamps = np.array(sorted(detections), dtype=np.int64)
    processed_stamps = np.array(sorted(processed), dtype=np.int64)
    rows, false_positives, false_with_id, duplicates, hidden = [], 0, 0, 0, 0
    for stamp, message in truth:
        # A frame the detector dropped is no miss of its own; the replay's rate keeps them few
        if processed and nearest(processed_stamps, stamp) is None:
            continue
        in_view = [
            (slot, marker)
            for slot, marker in enumerate(message.markers)
            if marker.in_frustum
            and not marker.occluded
            and marker.incidence_angle_deg < 90.0
        ]
        found_stamp = nearest(detection_stamps, stamp)
        frame = detections[found_stamp] if found_stamp is not None else []

        # Each detection to the marker nearest in bearing; one detection per marker
        pairs = []
        for index, (_, position, _) in enumerate(frame):
            for slot, marker in in_view:
                p = marker.pose.position
                bearing = angle_deg(position, [p.x, p.y, p.z])
                if bearing < GATE_DEG:
                    pairs.append((bearing, index, slot))
        pairs.sort()
        taken, used = {}, set()
        for _, index, slot in pairs:
            if index in used:
                continue
            if slot in taken:
                duplicates += 1
            else:
                taken[slot] = index
            used.add(index)
        # A marker the ground truth calls hidden can still be seen: cut by the image edge, or
        # behind an occluding ray. Only a detection of no marker, or with another's ID, is false.
        for k, (marker_id, position, _) in enumerate(frame):
            if k in used:
                continue
            near = [
                m.marker_id
                for m in message.markers
                if angle_deg(
                    position, [m.pose.position.x, m.pose.position.y, m.pose.position.z]
                )
                < GATE_DEG
            ]
            if near and (match == "position" or marker_id < 0 or marker_id in near):
                hidden += 1
                continue
            false_positives += 1
            # The ones that would mislead: an ID on something that is not that marker
            false_with_id += int(marker_id >= 0)

        for slot, marker in in_view:
            p = marker.pose.position
            truth_position = np.array([p.x, p.y, p.z])
            row = {
                "stamp_s": stamp * 1e-9,
                "slot": slot,
                "marker_id": marker.marker_id,
                "size_m": round(sizes[slot], 4),
                "range_m": marker.range_m,
                "incidence_deg": marker.incidence_angle_deg,
                "apparent_px": marker.apparent_diameter_px * fraction,
                "detected": 0,
                "found": 0,
                "wrong_id": 0,
                "no_id": 0,
                "range_error_m": "",
                "position_error_m": "",
                "normal_error_deg": "",
            }
            if slot in taken:
                marker_id, position, normal = frame[taken[slot]]
                if assumed_size:
                    position = position * sizes[slot] / assumed_size
                row["detected"] = 1
                if match == "position":
                    row["found"] = 1
                else:
                    row["found"] = int(marker_id == marker.marker_id)
                    row["wrong_id"] = int(
                        marker_id >= 0 and marker_id != marker.marker_id
                    )
                    row["no_id"] = int(marker_id < 0)
                row["range_error_m"] = np.linalg.norm(position) - marker.range_m
                row["position_error_m"] = np.linalg.norm(position - truth_position)
                truth_normal = rotation(marker.pose.orientation)[:, 2]
                row["normal_error_deg"] = min(
                    angle_deg(normal, truth_normal), angle_deg(-normal, truth_normal)
                )
            rows.append(row)

    with (directory / "rows.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=list(rows[0]) if rows else ["stamp_s"]
        )
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "frames": len(truth),
        "frames_processed": len(processed) if processed else None,
        "detections": int(sum(len(v) for v in detections.values())),
        "false_positives": false_positives,
        "false_positives_with_id": false_with_id,
        "duplicates": duplicates,
        "detections_of_hidden_markers": hidden,
        "sizes": {},
    }
    if cpu:
        frames = len(processed) or cpu["frames"]
        cpu["cpu_us_per_frame"] = round(1e6 * cpu["cpu_s"] / frames, 1)
        cpu["cpu_percent_at_30hz"] = round(100.0 * cpu["cpu_s"] * 30.0 / frames, 1)
        summary["cpu"] = cpu
    for size in sorted(set(round(s, 4) for s in sizes)):
        summary["sizes"][f"{size:g}"] = summarise(
            [r for r in rows if r["size_m"] == size]
        )
    (directory / "score.json").write_text(json.dumps(summary, indent=2))
    return summary


def errors(rows):
    """Median and 90th percentile of the relative range error, and of the normal's error."""
    hits = [r for r in rows if r["detected"]]
    if not hits:
        return {}
    relative = np.abs([r["range_error_m"] / r["range_m"] for r in hits]) * 100.0
    normal = np.array([r["normal_error_deg"] for r in hits])
    position = np.array([r["position_error_m"] for r in hits])
    return {
        "range_error_pct_median": round(float(np.median(relative)), 3),
        "range_error_pct_p90": round(float(np.percentile(relative, 90)), 3),
        "position_error_m_median": round(float(np.median(position)), 4),
        "normal_error_deg_median": round(float(np.median(normal)), 2),
        "normal_error_deg_p90": round(float(np.percentile(normal, 90)), 2),
    }


def binned(rows, key, width):
    bins = defaultdict(list)
    for row in rows:
        bins[math.floor(row[key] / width)].append(row)
    table = []
    for index in sorted(bins):
        members = bins[index]
        table.append(
            {
                "from": round(index * width, 3),
                "to": round((index + 1) * width, 3),
                "frames": len(members),
                "found": round(sum(r["found"] for r in members) / len(members), 4),
                "detected": round(
                    sum(r["detected"] for r in members) / len(members), 4
                ),
                "wrong_id": sum(r["wrong_id"] for r in members),
                **errors(members),
            }
        )
    return table


def summarise(rows):
    by_range = binned(rows, "range_m", RANGE_BIN_M)
    # Reliable: every bin from the nearest out to here finds at least 90%
    reliable = None
    for entry in by_range:
        if entry["frames"] < MIN_BIN_FRAMES:
            continue
        if entry["found"] < RELIABLE:
            break
        reliable = entry["to"]
    farthest = [
        e["to"] for e in by_range if e["frames"] >= MIN_BIN_FRAMES and e["found"] >= 0.5
    ]
    visible = [
        r
        for r in rows
        if r["apparent_px"] >= VISIBLE_PX
        and r["incidence_deg"] <= VISIBLE_INCIDENCE_DEG
    ]
    by_size = []
    for low, high in SIZE_BUCKETS_PX:
        members = [r for r in visible if low <= r["apparent_px"] < high]
        if members:
            by_size.append(
                {
                    "from_px": low,
                    "to_px": None if math.isinf(high) else high,
                    "frames": len(members),
                    "found": round(sum(r["found"] for r in members) / len(members), 4),
                    **errors(members),
                }
            )
    return {
        "frames": len(rows),
        "visible_frames": len(visible),
        "found_visible": round(
            sum(r["found"] for r in visible) / max(len(visible), 1), 4
        ),
        "wrong_id": sum(r["wrong_id"] for r in rows),
        "reliable_range_m": reliable,
        "max_range_m": max(farthest) if farthest else None,
        **errors(visible),
        "by_range": by_range,
        "by_incidence": binned(rows, "incidence_deg", INCIDENCE_BIN_DEG),
        "by_apparent_size": by_size,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("directory", type=Path)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--match", choices=("id", "position"), default="id")
    parser.add_argument("--assumed-size", type=float, default=0.0)
    args = parser.parse_args(argv)
    summary = score(args.directory, args.scene, args.match, args.assumed_size)
    for size, entry in summary["sizes"].items():
        print(
            f"{args.directory.name} {size} m: found {100 * entry['found_visible']:.1f}% of "
            f"{entry['visible_frames']} visible, reliable to {entry['reliable_range_m']} m, "
            f"max {entry['max_range_m']} m, wrong ids {entry['wrong_id']}, "
            f"range error {entry.get('range_error_pct_median')}% median"
        )
    print(
        f"false positives {summary['false_positives']} "
        f"({summary['false_positives_with_id']} with an ID), duplicates {summary['duplicates']}, "
        f"detections of hidden markers {summary['detections_of_hidden_markers']}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
