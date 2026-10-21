#!/usr/bin/env python3
"""Draw the loop, the markers and the driven trajectory over the warehouse floor plan.

For reviewing a path change without the GUI: shelving is dark, the configured loop blue, the
spawn green, side markers orange and end-of-lane markers red, each with its facing direction.
With --bag, the trajectory actually driven is drawn over the plan in magenta. A scene that
places the warehouse elsewhere (scene_range.yaml, scene_angle.yaml) is drawn on a map saved with
another through --map-scene.

    run_sim.py --verify-path --save-map /root/data/occupancy.npz       # in the isaac container
    python3 draw_loop_map.py /root/data/occupancy.npz --bag /root/data/whycode_1lap -o map.png
    python3 draw_loop_map.py /root/data/occupancy.npz --scene .../scene_range.yaml -o range.png
"""

import argparse
import math
import sys

import numpy as np
from PIL import Image, ImageDraw

from whycode_sim import scene_config
from whycode_sim.geometry import frame_change

DEFAULT_SCENE = "/root/workspace/src/whycode_sim/whycode_sim/config/scene.yaml"
PX_PER_M = 14


def read_trajectory(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/odom_ground_truth"]))
    points = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        p = deserialize_message(data, get_message(types[topic])).pose.pose.position
        points.append((p.x, p.y))
    return points


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("occupancy")
    parser.add_argument("--scene", default=DEFAULT_SCENE)
    parser.add_argument(
        "--map-scene", default=DEFAULT_SCENE, help="the scene the map was saved in"
    )
    parser.add_argument("--bag", help="overlay the trajectory driven in this bag")
    parser.add_argument("-o", "--output", default="loop_map.png")
    args = parser.parse_args(argv)

    plan = np.load(args.occupancy)
    grid, origin, resolution = plan["grid"], plan["origin"], float(plan["resolution"])
    # Maps saved with the robot's height mark what actually blocks it; the rest is overhead.
    blocking = plan["blocking"] if "blocking" in plan else grid
    config = scene_config.load(args.scene)
    to_map = frame_change(config.world, scene_config.load(args.map_scene).world)
    loop = [to_map(p) for p in config.path.waypoints]
    closed = getattr(config.path, "loop", True)
    markers = scene_config.resolved_markers(config)

    # The whole building, so the loop can be judged against the shelving and the open floor.
    x0, y0 = origin
    x1 = origin[0] + grid.shape[1] * resolution
    y1 = origin[1] + grid.shape[0] * resolution
    width, height = int((x1 - x0) * PX_PER_M), int((y1 - y0) * PX_PER_M)

    def px(x, y):  # world metres to image pixels, north up
        return ((x - x0) * PX_PER_M, (y1 - y) * PX_PER_M)

    # Floor plan: sample the occupancy grid at each pixel centre.
    cols = ((np.arange(width) + 0.5) / PX_PER_M + x0 - origin[0]) / resolution
    rows = ((y1 - (np.arange(height) + 0.5) / PX_PER_M) - origin[1]) / resolution
    cols = np.clip(cols.astype(int), 0, grid.shape[1] - 1)
    rows = np.clip(rows.astype(int), 0, grid.shape[0] - 1)
    blocked = blocking[rows[:, None], cols[None, :]]
    overhead = grid[rows[:, None], cols[None, :]] & ~blocked
    floor = np.full(blocked.shape + (3,), [236, 238, 241], dtype=np.uint8)
    floor[overhead] = [196, 200, 208]
    floor[blocked] = [70, 74, 82]
    image = Image.fromarray(floor, "RGB")
    draw = ImageDraw.Draw(image)

    for gx in range(math.ceil(x0), int(x1) + 1, 5):  # 5 m grid
        draw.line([px(gx, y0), px(gx, y1)], fill=(214, 218, 224))
    for gy in range(math.ceil(y0), int(y1) + 1, 5):
        draw.line([px(x0, gy), px(x1, gy)], fill=(214, 218, 224))

    draw.line([px(*p) for p in loop + loop[:1] * closed], fill=(40, 110, 230), width=4)
    if args.bag:
        driven = [to_map(p) for p in read_trajectory(args.bag)]
        draw.line([px(*p) for p in driven], fill=(220, 40, 160), width=2)

    for index, marker in enumerate(markers):
        x, y = marker.position[0], marker.position[1]
        end_of_lane = getattr(config.markers[index], "position", None) is not None
        colour = (220, 50, 50) if end_of_lane else (240, 150, 30)
        tip = to_map(
            (x + 1.2 * math.cos(marker.yaw_rad), y + 1.2 * math.sin(marker.yaw_rad))
        )
        x, y = to_map((x, y))
        draw.line([px(x, y), px(*tip)], fill=colour, width=2)
        cx, cy = px(x, y)
        draw.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], fill=colour)
        draw.text((cx + 7, cy - 7), f"#{index} id{marker.id}", fill=(20, 20, 20))

    sx, sy = px(*loop[0])
    draw.ellipse([sx - 7, sy - 7, sx + 7, sy + 7], fill=(30, 170, 80))
    draw.text((sx + 9, sy + 4), "spawn", fill=(30, 120, 60))
    draw.text((8, 8), "north up, 5 m grid", fill=(20, 20, 20))

    image.save(args.output)
    print(f"wrote {args.output} ({width}x{height})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
