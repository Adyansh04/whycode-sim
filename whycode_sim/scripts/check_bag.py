#!/usr/bin/env python3
"""Audit a recorded bag before trusting it.

Every failure this catches produced a bag that looked well formed: the right topics, rate and
size. Checks that frames carry pixels, that header stamps are evenly spaced, that log times are
sim time (so playback runs at the recorded rate rather than re-enacting a slow session), that
every frame has ground truth, that the robot drove whole laps from the spawn (an open path to
its end), and that no two visible markers share an id.

    python3 check_bag.py /root/data/whycode_1lap [--laps 1]
"""

import argparse
import struct
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from whycode_sim import scene_config
from whycode_sim.geometry import project_onto_path, resample_path

DEFAULT_SCENE = "/root/workspace/src/whycode_sim/whycode_sim/config/scene.yaml"
FRAME_SAMPLE_EVERY = 30  # decode one frame per sim second for the pixel check
EPOCH_THRESHOLD_S = 1e8  # log times beyond this are wall clock, not sim time
EDGE_S = 0.1  # images without ground truth are tolerated only this close to either end


def header_stamp(data):
    """Header stamp from raw CDR, skipping a full decode of a 2.8 MB image."""
    sec, nanosec = struct.unpack_from("<iI", data, 4)
    return sec + nanosec * 1e-9


def unwrapped_progress(waypoints, xy, loop_length):
    """Distance travelled along the loop, unwrapped across its seam."""
    arcs = [project_onto_path(waypoints, np.asarray(p), True) for p in xy]
    total = 0.0
    for a, b in zip(arcs, arcs[1:]):
        step = b - a
        if step < -loop_length / 2.0:
            step += loop_length
        elif step > loop_length / 2.0:
            step -= loop_length
        total += step
    return arcs[0], arcs[-1], total


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--scene", default=DEFAULT_SCENE)
    parser.add_argument(
        "--laps", type=int, help="fail unless exactly this many laps were driven"
    )
    args = parser.parse_args(argv)

    config = scene_config.load(args.scene)
    waypoints = [list(p) for p in config.path.waypoints]
    loop = getattr(config.path, "loop", True)
    _, loop_length = resample_path(waypoints, True)
    _, path_length = resample_path(waypoints, loop)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    header, logged, dark, frozen, odom = [], [], 0, 0, []
    previous, clashes, truth = None, 0, []
    while reader.has_next():
        topic, data, log_ns = reader.read_next()
        if topic == "/camera/image_raw":
            header.append(header_stamp(data))
            logged.append(log_ns * 1e-9)
            if len(header) % FRAME_SAMPLE_EVERY == 1:
                image = deserialize_message(data, get_message(types[topic]))
                pixels = np.frombuffer(image.data, np.uint8)
                dark += int(pixels.max() == 0)
                frozen += int(previous is not None and np.array_equal(pixels, previous))
                previous = pixels
        elif topic == "/odom_ground_truth":
            pose = deserialize_message(
                data, get_message(types[topic])
            ).pose.pose.position
            odom.append((pose.x, pose.y))
        elif topic == "/marker_ground_truth":
            ids = [
                m.marker_id
                for m in deserialize_message(data, get_message(types[topic])).markers
                if m.visible
            ]
            truth.append(header_stamp(data))
            clashes += int(len(ids) != len(set(ids)))

    if len(header) < 2 or len(odom) < 2:
        print(f"FAIL: {len(header)} images and {len(odom)} odometry messages")
        return 1

    header, logged = np.asarray(header), np.asarray(logged)
    header_dt, log_dt = np.diff(header), np.diff(logged)
    period = float(np.median(header_dt))
    sampled = (len(header) + FRAME_SAMPLE_EVERY - 1) // FRAME_SAMPLE_EVERY
    first_arc, last_arc, travelled = unwrapped_progress(waypoints, odom, loop_length)
    laps = travelled / loop_length
    sim_logged = logged[0] < EPOCH_THRESHOLD_S

    print(f"bag        : {args.bag}")
    print(
        f"images     : {len(header)} over {header[-1] - header[0]:.2f} s sim, {1.0 / period:.2f} Hz"
    )
    print(
        f"header dt  : median {period * 1e3:.2f} ms, spread "
        f"{(header_dt.max() - header_dt.min()) * 1e3:.2f} ms"
    )
    print(
        f"log dt     : median {np.median(log_dt) * 1e3:.2f} ms, p99 "
        f"{np.percentile(log_dt, 99) * 1e3:.2f}, max {log_dt.max() * 1e3:.2f}  "
        f"({'sim time' if sim_logged else 'WALL CLOCK'})"
    )
    print(f"pixels     : {sampled} frames sampled, {dark} black, {frozen} unchanged")
    print(
        f"robot      : arclength {first_arc:.2f} m -> {last_arc:.2f} m, "
        f"{travelled:.2f} m travelled = {laps:.3f} laps"
    )
    # A frame without ground truth cannot be scored. A startup race may leave the very first
    # frames without it; anywhere else it is a gap in the benchmark.
    stamps = np.sort(np.asarray(truth)) if truth else np.asarray([np.inf])
    after = np.clip(np.searchsorted(stamps, header), 0, len(stamps) - 1)
    before = np.clip(after - 1, 0, len(stamps) - 1)
    nearest = np.minimum(abs(stamps[after] - header), abs(stamps[before] - header))
    untruthed = header[nearest > period / 2]
    inside = untruthed[
        (untruthed > header[0] + EDGE_S) & (untruthed < header[-1] - EDGE_S)
    ]
    print(
        f"truth      : {len(truth)} frames, {len(untruthed)} images without one, "
        f"{clashes} with two visible markers sharing an id"
    )

    failures = []
    if dark or frozen:
        failures.append("frames are black or frozen: the render product stalled")
    if header_dt.max() - header_dt.min() > period * 0.05:
        failures.append("header stamps are unevenly spaced: frames were dropped")
    if not sim_logged:
        failures.append("log times are wall clock: record with --use-sim-time")
    elif np.percentile(log_dt, 99) > period * 1.5:
        failures.append("log times jitter: playback will stutter")
    if travelled < 0.5:
        failures.append("the camera barely moved: the robot is stuck")
    if args.laps is not None and abs(laps - args.laps) > 0.05:
        failures.append(f"drove {laps:.3f} laps, expected {args.laps}")
    # An open path ends where the follower stops, within its goal tolerance
    if not loop and travelled < path_length - 1.0:
        failures.append(f"drove {travelled:.2f} m of the {path_length:.2f} m path")
    if clashes:
        failures.append("a frame shows two visible markers with one id")
    if len(inside):
        failures.append(
            f"{len(inside)} images away from the bag's ends have no ground truth"
        )

    print()
    for failure in failures:
        print(f"FAIL: {failure}")
    if not failures:
        print("ok: the bag is sound")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
