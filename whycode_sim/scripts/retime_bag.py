#!/usr/bin/env python3
"""Rewrite a bag's log times from each message's own header stamp.

Playback schedules by log time. Even when recorded with `--use-sim-time`, an image's log time is
whichever /clock tick the recorder saw last, and the render pipeline delivers frames one or two
ticks late, so 13% of frames land on the same tick as their neighbour and 13% skip one. Header
stamps are exact, so taking log time from them makes playback as even as the camera.

The bag also begins at the first ground-truth frame: the recorder starts capturing a few frames
before ground_truth_node's inputs are discovered, and a frame without ground truth cannot be
scored.

    python3 retime_bag.py /root/data/raw_whycode_1lap /root/data/whycode_1lap
"""

import argparse
import heapq
import struct
import sys
from pathlib import Path

import rosbag2_py

MCAP_CONFIG = Path(__file__).resolve().parent.parent / "config" / "mcap_zstd.yaml"
# Frames arrive at most a few ticks late, so a 1 s sorting window is ample.
REORDER_WINDOW_NS = 1_000_000_000
# A stamp further than this from its log time is not trusted.
MAX_SHIFT_NS = 1_000_000_000
TRUTH_TOPIC = "/marker_ground_truth"
# Ground truth must begin this soon after the bag does.
TRUTH_DEADLINE_NS = 5_000_000_000
# Image and ground-truth stamps agree to microseconds.
MATCH_NS = 1_000_000

# Where the stamp sits in the CDR payload, after the 4-byte encapsulation header.
STAMP_OFFSET = {
    "sensor_msgs/msg/Image": 4,
    "sensor_msgs/msg/CameraInfo": 4,
    "nav_msgs/msg/Odometry": 4,
    "rosgraph_msgs/msg/Clock": 4,
    "whycode_sim_msgs/msg/MarkerGroundTruthArray": 4,
    "whycode_sim_msgs/msg/MarkerOcclusion": 4,
    "tf2_msgs/msg/TFMessage": 8,  # first transform's header, past the sequence length
}


def stamp_ns(data, offset):
    sec, nanosec = struct.unpack_from("<iI", data, offset)
    return sec * 1_000_000_000 + nanosec


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source")
    parser.add_argument("output")
    args = parser.parse_args(argv)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.source, storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=args.output, storage_id="mcap", storage_config_uri=str(MCAP_CONFIG)
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {}
    for topic in reader.get_all_topics_and_types():
        writer.create_topic(topic)
        types[topic.name] = topic.type

    # Retiming moves messages by a few ticks, so emit through a short sorting window to keep
    # the output in log-time order.
    pending, sequence, retimed, kept, dropped = [], 0, 0, 0, 0
    first_log, truth_start = None, None
    while reader.has_next():
        topic, data, log_ns = reader.read_next()
        offset = STAMP_OFFSET.get(types[topic])
        # A latched topic is written once and must stay at the start of the bag.
        if offset is not None and topic != "/tf_static" and len(data) >= offset + 8:
            stamp = stamp_ns(data, offset)
            if abs(stamp - log_ns) <= MAX_SHIFT_NS:
                log_ns, retimed = stamp, retimed + 1
            else:
                kept += 1
        else:
            kept += 1
        # Hold everything until ground truth appears, then drop what is stamped before it.
        # Frames can reach the bag ticks after their stamp, so the cut applies to arrivals
        # after the first ground truth too, not only to what is already held.
        first_log = log_ns if first_log is None else first_log
        if truth_start is None and topic == TRUTH_TOPIC:
            truth_start = log_ns - MATCH_NS
            before = len(pending)
            pending = [
                m for m in pending if m[0] >= truth_start or m[2] == "/tf_static"
            ]
            heapq.heapify(pending)
            dropped += before - len(pending)
        elif truth_start is None and log_ns - first_log > TRUTH_DEADLINE_NS:
            raise SystemExit(f"no {TRUTH_TOPIC} within 5 s of the bag's start")
        elif truth_start is not None and log_ns < truth_start and topic != "/tf_static":
            dropped += 1
            continue

        heapq.heappush(pending, (log_ns, sequence, topic, data))
        sequence += 1
        if truth_start is None:
            continue

        while pending and pending[0][0] < log_ns - REORDER_WINDOW_NS:
            out_ns, _, out_topic, out_data = heapq.heappop(pending)
            writer.write(out_topic, out_data, out_ns)
    while pending:
        out_ns, _, out_topic, out_data = heapq.heappop(pending)
        writer.write(out_topic, out_data, out_ns)

    del writer  # closing writes the summary and metadata.yaml
    print(
        f"{retimed} messages retimed from their header stamp, {kept} kept as logged, "
        f"{dropped} dropped from before the first ground truth"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
