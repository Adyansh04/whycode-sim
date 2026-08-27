"""Renders MarkerGroundTruthArray as RViz markers.

RViz cannot display a custom message, so this converts ground truth into
visualization_msgs. A replay-time debugging aid, not part of the recorded contract.

Two views, answering different questions:

  camera frame  where the detector should find each marker. If these do not sit on the
                markers in the camera image, the frame convention is wrong.
  world frame   the static layout and the robot's path through it, for checking the loop
                and placement against the config.

Colour says why a marker is not counted visible, which is the question a missing detection
raises.
"""

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from whycode_sim_msgs.msg import MarkerGroundTruthArray

VISIBLE = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.9)  # detector should see this
OCCLUDED = ColorRGBA(r=0.9, g=0.2, b=0.2, a=0.9)  # in view but blocked
TOO_SMALL = ColorRGBA(r=0.9, g=0.7, b=0.1, a=0.8)  # in view, too few pixels
EDGE_ON = ColorRGBA(r=0.6, g=0.3, b=0.9, a=0.8)  # in view, too oblique
OUT_OF_VIEW = ColorRGBA(r=0.4, g=0.4, b=0.4, a=0.25)  # not in frame at all


def colour_for(marker):
    if marker.visible:
        return VISIBLE
    if not marker.in_frustum:
        return OUT_OF_VIEW
    if marker.occluded:
        return OCCLUDED
    if not marker.facing_camera:
        return EDGE_ON
    return TOO_SMALL


class GroundTruthVizNode(Node):
    def __init__(self):
        super().__init__("marker_ground_truth_viz")

        self.declare_parameter("marker_size_m", 0.2)
        # Isaac names the stage root "World"; this must match or the world-frame markers
        # are published into a frame that does not exist.
        self.declare_parameter("world_frame", "World")
        self.declare_parameter("trail_length", 2000)

        self.size = self.get_parameter("marker_size_m").value
        self.world_frame = self.get_parameter("world_frame").value
        self.trail_length = self.get_parameter("trail_length").value
        self.trail = []

        # Two topics: RViz's Camera display can hide a display but not a namespace, so
        # world-frame geometry on the same topic would draw through the image overlay.
        self.publisher = self.create_publisher(
            MarkerArray, "/marker_ground_truth_viz", 1
        )
        self.layout_publisher = self.create_publisher(
            MarkerArray, "/marker_layout_viz", 1
        )
        self.create_subscription(
            MarkerGroundTruthArray, "/marker_ground_truth", self._on_ground_truth, 10
        )
        self.get_logger().info("publishing RViz markers on /marker_ground_truth_viz")

    def _on_ground_truth(self, msg):
        overlay = MarkerArray()
        layout = MarkerArray()
        for index, entry in enumerate(msg.markers):
            overlay.markers.extend(self._camera_frame(msg, entry, index))
            layout.markers.append(self._world_frame(msg, entry, index))
        layout.markers.append(self._robot_trail(msg))
        self.publisher.publish(overlay)
        self.layout_publisher.publish(layout)

    def _camera_frame(self, msg, entry, index):
        """The marker plate and its label, at the pose ground truth reports.

        Stamped zero, meaning "use the latest transform". The ground-truth stamp can sit
        a few milliseconds ahead of the newest TF in RViz's buffer, and the lookup then
        fails as an extrapolation with nothing drawn. The cost is a few centimetres of
        pose lag at walking pace, which is fine for a viewer.
        """
        plate = Marker()
        plate.header.frame_id = msg.header.frame_id
        plate.header.stamp = Time().to_msg()
        plate.ns = "gt_camera"
        plate.id = index
        plate.type = Marker.CUBE
        plate.action = Marker.ADD
        plate.pose = entry.pose
        # Thin in the marker's own normal direction, which is local Z.
        plate.scale.x = self.size
        plate.scale.y = self.size
        plate.scale.z = 0.01
        plate.color = colour_for(entry)

        label = Marker()
        label.header.frame_id = msg.header.frame_id
        label.header.stamp = Time().to_msg()
        label.ns = "gt_camera_label"
        label.id = index
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = entry.pose.position.x
        label.pose.position.y = entry.pose.position.y
        label.pose.position.z = entry.pose.position.z + self.size
        label.pose.orientation.w = 1.0
        label.scale.z = 0.12
        label.color = colour_for(entry)
        label.text = (
            f"id{entry.marker_id} {entry.range_m:.1f}m "
            f"{entry.apparent_diameter_px:.0f}px {entry.incidence_angle_deg:.0f}deg"
        )
        return [plate, label]

    def _world_frame(self, msg, entry, index):
        """The static layout, so the configured loop and placement can be eyeballed."""
        plate = Marker()
        plate.header.stamp = Time().to_msg()
        plate.header.frame_id = self.world_frame
        plate.ns = "gt_world"
        plate.id = index
        plate.type = Marker.CUBE
        plate.action = Marker.ADD
        plate.pose = entry.world_pose
        plate.scale.x = 0.01
        plate.scale.y = self.size
        plate.scale.z = self.size
        plate.color = colour_for(entry)
        return plate

    def _robot_trail(self, msg):
        """Where the camera has been, drawn in the world frame."""
        position = msg.camera_world_pose.position
        point = Point(x=position.x, y=position.y, z=position.z)
        if (
            not self.trail
            or math.dist((point.x, point.y), (self.trail[-1].x, self.trail[-1].y))
            > 0.05
        ):
            self.trail.append(point)
            del self.trail[: max(0, len(self.trail) - self.trail_length)]

        trail = Marker()
        trail.header.stamp = Time().to_msg()
        trail.header.frame_id = self.world_frame
        trail.ns = "camera_trail"
        trail.id = 0
        trail.type = Marker.LINE_STRIP
        trail.action = Marker.ADD
        trail.pose.orientation.w = 1.0
        trail.scale.x = 0.04
        trail.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.9)
        trail.points = list(self.trail)
        return trail


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
