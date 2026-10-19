"""Per-frame publishers needing direct stage access.

Things the OmniGraph bridge cannot provide:

  /camera_ground_truth  the camera's exact world pose, body convention. Deriving it from
                        /tf would mean interpolation, and one tick of misalignment at 30 Hz
                        and 1 m/s is ~33 mm, the same order as the error being measured.
  /marker_occlusion     a line-of-sight raycast per marker; only the simulator sees the
                        collision geometry.
  /odom_ground_truth    the chassis world pose. IsaacComputeOdometry reports relative to the
                        starting pose, which is not world once the robot spawns facing the path.

All use plain rclpy rather than OmniGraph, which keeps the timestamp under our control.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from pxr import Usd, UsdGeom
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from whycode_sim.geometry import R_USD_TO_GT, matrix_to_quaternion
from whycode_sim_msgs.msg import MarkerOcclusion

# Markers have no colliders, so a ray stopping short hit real scene geometry. Pull the
# hit back from the face so a ray landing on the marker is not counted as a blocker.
OCCLUSION_EPSILON_M = 0.05


class SimPublishers(Node):
    def __init__(self, config, scene):
        super().__init__("whycode_sim_publishers")
        self.config = config
        self.markers = scene["markers"]
        self.camera_path = scene["camera_path"]
        self.chassis_path = scene["chassis_path"]
        self.frame_id = config.camera.frame_id
        self._last_odometry = None

        self.camera_pose_publisher = self.create_publisher(
            Odometry, "/camera_ground_truth", 10
        )
        self.occlusion_publisher = self.create_publisher(
            MarkerOcclusion, "/marker_occlusion", 10
        )
        self.odometry_publisher = self.create_publisher(
            Odometry, "/odom_ground_truth", 10
        )

        self._publish_static_frames()
        self._sim_time_interface = self._acquire_sim_time_interface()
        self._raycast = self._acquire_raycast()

    def _publish_static_frames(self):
        """Tie the odometry frame names into the TF tree Isaac publishes.

        Isaac names frames after prims (World, chassis_link) while the odometry message
        keeps the ROS convention (odom -> base_link), so without these the odometry
        frames are absent from TF. Both are exact: the robot spawns at the world origin,
        so odom and World coincide, and base_link is the chassis.
        """
        self._static_broadcaster = StaticTransformBroadcaster(self)
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for parent, child in (("World", "odom"), ("chassis_link", "base_link")):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self._static_broadcaster.sendTransform(transforms)
        self.get_logger().info(
            "published static World->odom and chassis_link->base_link"
        )

    def sim_time(self, fallback):
        """The simulation time the OmniGraph publishers stamp with.

        IsaacReadSimulationTime reads the same interface, so values land on the tick
        boundaries the camera helper uses; SimulationContext.current_time drifts from it.

        Do not round. Tick boundaries at 30 Hz are 33333335 ns apart, not whole
        microseconds, so quantising moves the stamp off the renderer's value.
        """
        if self._sim_time_interface is not None:
            return self._sim_time_interface.get_sim_time()
        return fallback

    def _acquire_sim_time_interface(self):
        try:
            from isaacsim.core.nodes.bindings import _isaacsim_core_nodes

            interface = _isaacsim_core_nodes.acquire_interface()
        except Exception as error:  # noqa: BLE001 - fall back rather than fail the run
            self.get_logger().warning(
                f"simulation-time interface unavailable ({error}); stamps fall back to "
                "the simulation context clock, which drifts sub-microsecond from the image"
            )
            return None

        # The binding stub declares no methods; confirm the accessor exists rather than
        # discovering it missing once per frame.
        if not hasattr(interface, "get_sim_time"):
            self.get_logger().warning(
                "core-nodes interface has no get_sim_time; falling back to the "
                f"simulation context clock. Available: {sorted(dir(interface))[:12]}"
            )
            return None
        return interface

    def _acquire_raycast(self):
        """PhysX scene query interface, or None when physics is not available."""
        try:
            from omni.physx import get_physx_scene_query_interface

            return get_physx_scene_query_interface()
        except Exception as error:  # noqa: BLE001 - absence is reported, not fatal
            self.get_logger().warning(
                f"scene queries unavailable ({error}); occlusion will report false"
            )
            return None

    def camera_world_transform(self, stage):
        """Camera position and USD-convention rotation, read straight off the stage."""
        return self._world_pose(stage, self.camera_path)

    def _world_pose(self, stage, path):
        prim = stage.GetPrimAtPath(path)
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        translation = matrix.ExtractTranslation()
        rotation = matrix.ExtractRotationMatrix()
        position = np.array([translation[0], translation[1], translation[2]])
        basis = np.array(
            [[rotation[row][col] for col in range(3)] for row in range(3)]
        ).T
        return position, basis

    def publish(self, stage, sim_time):
        """Publish every topic for the frame rendered at `sim_time` (seconds)."""
        seconds = self.sim_time(sim_time)
        stamp = rclpy.time.Time(seconds=seconds).to_msg()
        position, rotation = self.camera_world_transform(stage)

        # The child frame is camera_link, the body convention, so convert from USD's.
        body = rotation @ R_USD_TO_GT.T
        self.camera_pose_publisher.publish(
            _pose_message(stamp, "World", self.frame_id, position, body)
        )
        self._publish_occlusion(stamp, position)
        self._publish_odometry(stage, stamp, seconds)

    def _publish_odometry(self, stage, stamp, seconds):
        position, rotation = self._world_pose(stage, self.chassis_path)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        message = _pose_message(stamp, "odom", "base_link", position, rotation)

        # Twist in the body frame, from the change since the previous tick.
        if self._last_odometry is not None:
            last_seconds, last_position, last_yaw = self._last_odometry
            elapsed = seconds - last_seconds
            if elapsed > 0.0:
                linear = rotation.T @ ((position - last_position) / elapsed)
                turn = math.remainder(yaw - last_yaw, math.tau)
                twist = message.twist.twist
                twist.linear.x, twist.linear.y, twist.linear.z = (
                    float(v) for v in linear
                )
                twist.angular.z = turn / elapsed
        self._last_odometry = (seconds, position, yaw)
        self.odometry_publisher.publish(message)

    def _publish_occlusion(self, stamp, camera_position):
        message = MarkerOcclusion()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id

        for marker in self.markers:
            target = np.asarray(marker.position)
            direction = target - camera_position
            distance = float(np.linalg.norm(direction))
            occluded, hit_distance = False, distance

            if self._raycast is not None and distance > 1e-6:
                direction = direction / distance
                hit = self._raycast.raycast_closest(
                    camera_position.tolist(),
                    direction.tolist(),
                    distance - OCCLUSION_EPSILON_M,
                )
                if hit and hit.get("hit", False):
                    occluded = True
                    hit_distance = float(hit.get("distance", distance))

            message.marker_ids.append(int(marker.id))
            message.occluded.append(bool(occluded))
            message.hit_distance_m.append(float(hit_distance))

        self.occlusion_publisher.publish(message)

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.0)


def _pose_message(stamp, frame_id, child_frame_id, position, rotation):
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.child_frame_id = child_frame_id
    pose = message.pose.pose
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
    x, y, z, w = matrix_to_quaternion(rotation)
    pose.orientation.x, pose.orientation.y = float(x), float(y)
    pose.orientation.z, pose.orientation.w = float(z), float(w)
    return message
