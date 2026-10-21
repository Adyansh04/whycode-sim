#!/usr/bin/env python3
"""Replays a benchmark bag through one detector and records what it reports.

    python3 replay_detector.py <bag> --detector <name> --scene <scene.yaml> --out <dir>
        [--rate 1.0]

Detectors: whycode and whycon (whycode_vision, the Isaac config with this camera's size and
intrinsics, identification off for whycon), apriltag and apriltag_full (apriltag_ros on
tag36h11, quads found at half and at full resolution), aruco (whycode_baselines' OpenCV
ArUco node), isaac_apriltag (Isaac ROS on the GPU, started beforehand in the isaac-ros
container by launch/isaac_ros_apriltag.launch.py). Tag sizes come from the scene; the camera from
the bag's camera_info.

Writes <dir>/replay, a bag of /marker_ground_truth and the detector's output, and
<dir>/cpu.json: the detector's CPU time (every thread, its child processes included) per
frame it processed. Refuses to start while another ROS session is running.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo

from whycode_sim import scene_config

INSTALL = Path(os.environ.get("INSTALL", "/root/workspace/install"))
VISION_SHARE = INSTALL / "whycode_vision" / "share" / "whycode_vision"
PLAYED = ["/camera/image_raw", "/camera/camera_info", "/clock", "/marker_ground_truth"]
OUTPUTS = {
    "whycode": ["/whycon/poses"],
    "whycon": ["/whycon/poses"],
    "apriltag": ["/tf", "/detections"],
    "apriltag_full": ["/tf", "/detections"],
    "aruco": ["/tf"],
    "isaac_apriltag": ["/tf"],
}
# Started outside, in the isaac-ros container; found by its process name, which ros-dev sees
# through its host PID namespace
EXTERNAL = {"isaac_apriltag": "isaac_apriltag_container"}
TICKS = os.sysconf("SC_CLK_TCK")


def camera_info(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/camera/camera_info"]))
    while reader.has_next():
        _, data, _ = reader.read_next()
        return deserialize_message(data, CameraInfo)
    raise RuntimeError(f"{bag} has no /camera/camera_info")


def image_count(bag):
    info = rosbag2_py.Info().read_metadata(str(bag), "")
    return next(
        topic.message_count
        for topic in info.topics_with_message_count
        if topic.topic_metadata.name == "/camera/image_raw"
    )


def tag_sizes(scene):
    """Each marker id's black edge (m)."""
    config = scene_config.load(scene)
    family = getattr(config.marker.families, config.marker.family)
    fraction = getattr(family, "black_fraction", 1.0)
    return {
        marker.id: round(marker.size_m * fraction, 6)
        for marker in scene_config.resolved_markers(config)
    }


def whycode_command(out, info, sizes, identify):
    """The Isaac config with this camera, and the most common marker size as the diameter.

    The detector takes one diameter; evaluate.py rescales a marker of another size, whose
    position scales exactly with it.
    """
    intrinsics = {
        "image_width": info.width,
        "image_height": info.height,
        "camera_name": "replay",
        "camera_matrix": {"rows": 3, "cols": 3, "data": [float(v) for v in info.k]},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {"rows": 1, "cols": 5, "data": [0.0] * 5},
    }
    intrinsics_path = out / "camera.yaml"
    intrinsics_path.write_text(yaml.safe_dump(intrinsics))
    config = yaml.safe_load(
        (VISION_SHARE / "config" / "whycon_config_isaac.yaml").read_text()
    )
    config["camera"].update(
        {
            "image_width": info.width,
            "image_height": info.height,
            "frame_rate": 30.0,
            "package_name": "whycode_vision",
            # Resolved inside the package share, so relative to it
            "config_path": os.path.relpath(intrinsics_path.resolve(), VISION_SHARE),
        }
    )
    diameters = list(sizes.values())
    config["detector"]["outer_diameter"] = max(
        set(diameters), key=lambda size: (diameters.count(size), size)
    )
    config["identification"]["enabled"] = identify
    config["ros_interface"].update(
        {
            "publish_images": False,
            "publish_debug_images": False,
            "publish_tf": False,
            "publish_visualization_markers": False,
        }
    )
    config_path = out / "whycon_config.yaml"
    config_path.write_text(yaml.safe_dump(config, sort_keys=False))
    (out / "assumed_size").write_text(str(config["detector"]["outer_diameter"]))
    return [
        "ros2", "run", "whycode_vision", "whycon", "--ros-args",
        "-p", f"config_file:={config_path}", "-p", "use_sim_time:=true",
    ]  # fmt: skip


def apriltag_command(out, sizes, decimate):
    ids = sorted(sizes)
    params = {
        "apriltag": {
            "ros__parameters": {
                "use_sim_time": True,
                "image_transport": "raw",
                "qos_profile": "default",  # Reliable, as the bag plays: every frame arrives
                "family": "36h11",
                "size": sizes[ids[0]],
                "max_hamming": 0,
                "detector": {
                    "threads": 1,
                    "decimate": decimate,
                    "blur": 0.0,
                    "refine": True,
                    "sharpening": 0.25,
                    "debug": False,
                },
                "tag": {
                    "ids": ids,
                    "frames": [f"tag36h11:{i}" for i in ids],
                    "sizes": [sizes[i] for i in ids],
                },
            }
        }
    }
    params_path = out / "apriltag.yaml"
    params_path.write_text(yaml.safe_dump(params))
    return [
        "ros2", "run", "apriltag_ros", "apriltag_node", "--ros-args",
        "-r", "__node:=apriltag", "-r", "image_rect:=/camera/image_raw",
        "-r", "camera_info:=/camera/camera_info", "--params-file", str(params_path),
    ]  # fmt: skip


def aruco_command(sizes):
    ids = sorted(sizes)
    return [
        "ros2", "run", "whycode_baselines", "aruco_node", "--ros-args",
        "-r", "image:=/camera/image_raw", "-r", "camera_info:=/camera/camera_info",
        "-p", f"ids:={ids}", "-p", f"sizes:={[sizes[i] for i in ids]}",
        "-p", "use_sim_time:=true",
    ]  # fmt: skip


def detector_command(name, out, info, sizes):
    if name in ("whycode", "whycon"):
        return whycode_command(out, info, sizes, identify=name == "whycode")
    if name == "apriltag":
        return apriltag_command(out, sizes, decimate=2.0)
    if name == "apriltag_full":
        return apriltag_command(out, sizes, decimate=1.0)
    if name == "aruco":
        return aruco_command(sizes)
    if name in EXTERNAL:
        # One size for every tag: the most common, as for WhyCode
        diameters = list(sizes.values())
        size = max(set(diameters), key=lambda value: (diameters.count(value), value))
        (out / "assumed_size").write_text(str(size))
        return None
    raise ValueError(f"unknown detector {name}")


def find_process(pattern, timeout=120.0):
    """The pid whose command line holds `pattern`, once it is running."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for entry in os.listdir("/proc"):
            if not entry.isdigit():
                continue
            try:
                command = (
                    Path(f"/proc/{entry}/cmdline").read_bytes().replace(b"\0", b" ")
                )
            except OSError:
                continue
            if pattern.encode() in command and b"ros2 launch" not in command:
                return int(entry)
        time.sleep(1.0)
    raise RuntimeError(f"no process matching {pattern} after {timeout:.0f} s")


def process_tree(pid):
    """pid and every descendant: `ros2 run` execs the detector as its child."""
    pids, frontier = [pid], [pid]
    while frontier:
        parent = frontier.pop()
        try:
            for task in os.listdir(f"/proc/{parent}/task"):
                children = (
                    Path(f"/proc/{parent}/task/{task}/children").read_text().split()
                )
                pids.extend(int(c) for c in children)
                frontier.extend(int(c) for c in children)
        except FileNotFoundError:
            pass
    return pids


def cpu_seconds(pids):
    total = 0
    for pid in pids:
        try:
            fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
            total += int(fields[11]) + int(fields[12])  # utime, stime
        except FileNotFoundError:
            pass
    return total / TICKS


def stop(process, timeout=20):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def ros_session_running():
    nodes = subprocess.run(
        ["ros2", "node", "list", "--no-daemon"],
        capture_output=True,
        text=True,
        timeout=30,
    ).stdout.split()
    return [node for node in nodes if node]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag", type=Path)
    parser.add_argument("--detector", required=True, choices=sorted(OUTPUTS))
    parser.add_argument("--scene", required=True)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--rate", type=float, default=1.0)
    args = parser.parse_args(argv)

    # An external detector's own nodes are expected, its launch's node among them
    running = [
        n
        for n in ros_session_running()
        if args.detector not in EXTERNAL
        or not ("apriltag" in n or n.startswith("/launch_ros_"))
    ]
    if running:
        print(
            f"a ROS session is running ({', '.join(running)}); not starting",
            file=sys.stderr,
        )
        return 1
    args.out.mkdir(parents=True, exist_ok=True)
    replay = args.out / "replay"
    subprocess.run(
        ["rm", "-rf", str(replay), str(args.out / "assumed_size")], check=True
    )

    info = camera_info(args.bag)
    sizes = tag_sizes(args.scene)
    log = (args.out / "detector.log").open("w")
    command = detector_command(args.detector, args.out, info, sizes)
    if command is None:
        detector, detector_pid = None, find_process(EXTERNAL[args.detector])
    else:
        detector = subprocess.Popen(
            command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
        )
        detector_pid = detector.pid
    recorder = player = None
    try:
        time.sleep(3.0)
        recorder = subprocess.Popen(
            ["ros2", "bag", "record", "--use-sim-time", "-o", str(replay),
             "/marker_ground_truth", *OUTPUTS[args.detector]],
            stdout=(args.out / "record.log").open("w"), stderr=subprocess.STDOUT,
            start_new_session=True,
        )  # fmt: skip
        time.sleep(3.0)
        before = cpu_seconds(process_tree(detector_pid))
        started = time.monotonic()
        player = subprocess.run(
            ["ros2", "bag", "play", str(args.bag), "--clock", "--rate", str(args.rate),
             "--topics", *PLAYED],
            stdout=(args.out / "play.log").open("w"), stderr=subprocess.STDOUT,
        )  # fmt: skip
        # Let the detector drain its queue before reading its CPU time
        time.sleep(3.0)
        cpu = cpu_seconds(process_tree(detector_pid)) - before
        wall = time.monotonic() - started
    finally:
        if recorder is not None:
            stop(recorder)
        if detector is not None:
            stop(detector)

    frames = image_count(args.bag)
    assumed = args.out / "assumed_size"
    result = {
        "bag": str(args.bag),
        "detector": args.detector,
        "rate": args.rate,
        "frames": frames,
        "width": info.width,
        "height": info.height,
        "cpu_s": round(cpu, 3),
        "wall_s": round(wall, 1),
        "cpu_us_per_frame": round(1e6 * cpu / frames, 1),
        "cpu_percent_at_30hz": round(100.0 * cpu * 30.0 / frames, 1),
        "player_exit": player.returncode if player else None,
        # The one marker size the detector was given, for evaluate.py to rescale the others
        "assumed_size_m": float(assumed.read_text()) if assumed.exists() else None,
    }
    (args.out / "cpu.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result))
    return 0


if __name__ == "__main__":
    sys.exit(main())
