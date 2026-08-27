"""Publishes exact marker poses relative to the camera, for comparison against the detector.

Runs in the ROS container rather than inside Isaac so it can be re-run against a recorded
bag: fixing the visibility maths or tightening a threshold costs one replay rather than a
fresh simulation that would not be bit-identical.

Poses use the frame whycode_vision reports (X forward, Y left, Z up), so
/marker_ground_truth and /whycon/poses can be differenced field by field.

Camera pose and occlusion flags are published by the simulator in one call with one stamp,
so they join on exact equality every frame. The image is deliberately not in that join: it
is rendered asynchronously and can lag by seconds, and buffering enough 1280x720 frames to
bridge that would cost hundreds of megabytes to gain nothing but a timestamp. Both streams
are stamped from simulation time, so ground truth lands within a microsecond of the image
it describes -- far inside one frame period.
"""

import message_filters
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

from whycode_sim import scene_config
from whycode_sim.geometry import (
    evaluate_marker,
    matrix_to_quaternion,
    quaternion_to_matrix,
)
from whycode_sim_msgs.msg import (
    MarkerGroundTruth,
    MarkerGroundTruthArray,
    MarkerOcclusion,
)


class GroundTruthNode(Node):
    def __init__(self):
        super().__init__("marker_ground_truth")

        self.declare_parameter("scene_config", "")
        self.declare_parameter("sync_queue_size", 30)

        config_path = self.get_parameter("scene_config").value
        if not config_path:
            raise RuntimeError("the scene_config parameter is required")

        self.config = scene_config.load(config_path)
        self.markers = scene_config.resolved_markers(self.config)
        self.limits = self.config.visibility
        self.frame_id = self.config.camera.frame_id

        # Intrinsics come from the live camera_info rather than the scene file, so a
        # resolution change cannot leave the projection and the renderer disagreeing.
        self.intrinsics = None
        self.image_size = None
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self._on_camera_info, 1
        )

        self.publisher = self.create_publisher(
            MarkerGroundTruthArray, "/marker_ground_truth", 10
        )

        queue_size = self.get_parameter("sync_queue_size").value
        synchronizer = message_filters.TimeSynchronizer(
            [
                message_filters.Subscriber(self, Odometry, "/camera_ground_truth"),
                message_filters.Subscriber(self, MarkerOcclusion, "/marker_occlusion"),
            ],
            queue_size,
        )
        synchronizer.registerCallback(self._on_frame)

        self.frames_published = 0
        self.get_logger().info(
            f"ground truth ready for {len(self.markers)} {self.config.marker.family} markers"
        )

    def _on_camera_info(self, msg):
        intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        if self.intrinsics is None:
            self.get_logger().info(
                f"intrinsics fx={intrinsics[0]:.4f} fy={intrinsics[1]:.4f} "
                f"cx={intrinsics[2]:.1f} cy={intrinsics[3]:.1f} at {msg.width}x{msg.height}"
            )
        self.intrinsics = intrinsics
        self.image_size = (msg.width, msg.height)

    def _on_frame(self, camera_pose, occlusion):
        if self.intrinsics is None:
            return

        position = camera_pose.pose.pose.position
        camera_position = np.array([position.x, position.y, position.z])
        rotation = camera_pose.pose.pose.orientation
        camera_rotation = quaternion_to_matrix(
            rotation.x, rotation.y, rotation.z, rotation.w
        )

        # Indexed positionally, not keyed by id: the layout may carry the same id twice,
        # and a dict would hand both markers the second one's flag. The message declares
        # its arrays parallel to the scene config order.
        if len(occlusion.occluded) != len(self.markers):
            self.get_logger().warn(
                f"occlusion array has {len(occlusion.occluded)} entries for "
                f"{len(self.markers)} markers; skipping frame"
            )
            return

        array = MarkerGroundTruthArray()
        array.header.stamp = camera_pose.header.stamp
        array.header.frame_id = self.frame_id
        array.camera_world_pose = camera_pose.pose.pose

        visible_count = 0
        for index, marker in enumerate(self.markers):
            result = evaluate_marker(
                marker,
                camera_position,
                camera_rotation,
                self.intrinsics,
                self.image_size,
                self.limits,
            )
            entry = self._to_message(marker, result, bool(occlusion.occluded[index]))
            visible_count += int(entry.visible)
            array.markers.append(entry)

        array.visible_marker_count = visible_count
        self.publisher.publish(array)
        self.frames_published += 1

    def _to_message(self, marker, result, occluded):
        entry = MarkerGroundTruth()
        entry.marker_id = int(marker.id)
        entry.family = marker.family
        entry.texture = marker.texture

        offset = result["position_camera"]
        entry.pose.position.x = float(offset[0])
        entry.pose.position.y = float(offset[1])
        entry.pose.position.z = float(offset[2])
        qx, qy, qz, qw = matrix_to_quaternion(result["rotation_camera"])
        entry.pose.orientation.x = float(qx)
        entry.pose.orientation.y = float(qy)
        entry.pose.orientation.z = float(qz)
        entry.pose.orientation.w = float(qw)

        entry.world_pose.position.x = float(marker.position[0])
        entry.world_pose.position.y = float(marker.position[1])
        entry.world_pose.position.z = float(marker.position[2])
        entry.world_pose.orientation.z = float(np.sin(marker.yaw_rad / 2.0))
        entry.world_pose.orientation.w = float(np.cos(marker.yaw_rad / 2.0))

        entry.range_m = float(result["range_m"])
        entry.incidence_angle_deg = float(result["incidence_angle_deg"])
        entry.bearing_u_px = float(result["bearing_u_px"])
        entry.bearing_v_px = float(result["bearing_v_px"])
        entry.apparent_diameter_px = float(result["apparent_diameter_px"])

        entry.in_frustum = bool(result["in_frustum"])
        entry.facing_camera = bool(result["facing_camera"])
        entry.large_enough = bool(result["large_enough"])
        entry.occluded = occluded
        entry.visible = (
            entry.in_frustum
            and entry.facing_camera
            and entry.large_enough
            and not occluded
        )
        return entry


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthNode()
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
