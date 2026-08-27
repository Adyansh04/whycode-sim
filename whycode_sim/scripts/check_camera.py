#!/usr/bin/env python3
"""Confirm the simulator is rendering before committing to a long recording.

A stalled render product still publishes /camera/image_raw at the right rate, encoding and
size, with every pixel zero. Ground truth, odometry and TF stay correct, and zstd packs the
result small enough that the bag size looks ordinary, so nothing catches it except opening
an image.

Exits non-zero if the frames are black or frozen, so it can gate a recording:

    python3 check_camera.py && ros2 launch whycode_sim sim.launch.py record:=true ...
"""

import argparse
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class FrameSampler(Node):
    def __init__(self, topic):
        super().__init__("camera_check")
        self.frames = []
        self.create_subscription(
            Image,
            topic,
            self._on_image,
            QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )

    def _on_image(self, msg):
        self.frames.append(
            np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/camera/image_raw")
    parser.add_argument("--frames", type=int, default=12)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument(
        "--min-mean",
        type=float,
        default=1.0,
        help="reject if every sampled frame is darker than this",
    )
    args = parser.parse_args(argv)

    rclpy.init()
    node = FrameSampler(args.topic)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    deadline = time.time() + args.timeout
    while len(node.frames) < args.frames and time.time() < deadline:
        time.sleep(0.2)

    frames = list(node.frames)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    if len(frames) < 2:
        print(
            f"FAIL: only {len(frames)} frames on {args.topic} in {args.timeout:.0f} s"
        )
        return 1

    means = [float(f.mean()) for f in frames]
    peak = max(int(f.max()) for f in frames)
    # Frames are compared against the first rather than pairwise: a renderer that latched
    # one good frame and stopped updating also needs to fail here.
    motion = max(
        float(np.abs(f.astype(int) - frames[0].astype(int)).mean()) for f in frames
    )

    print(
        f"frames sampled : {len(frames)} at {frames[0].shape[1]}x{frames[0].shape[0]}"
    )
    print(f"mean brightness: {min(means):.2f} .. {max(means):.2f}   peak pixel {peak}")
    print(f"largest change : {motion:.3f} (0 means nothing moved)")

    if peak == 0 or max(means) < args.min_mean:
        print(
            "\nFAIL: every frame is black. The render product is not producing pixels."
        )
        print("Restart the simulator container and wait for it to report healthy.")
        return 1
    if motion < 1e-6:
        print(
            "\nFAIL: every frame is identical. The renderer has stalled on one image."
        )
        return 1

    print("\nok: the camera is rendering")
    return 0


if __name__ == "__main__":
    sys.exit(main())
