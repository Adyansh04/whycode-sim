"""Drives the robot around the configured loop with a pure-pursuit controller.

Pure pursuit rather than drive-to-point: the approach profile past each marker is what the
recording is for, and drive-to-point stops and rotates in place at every waypoint, giving
motion-blur spikes at the corners and stretches of zero forward motion.

Reads the same scene config as the scene builder and the ground-truth node, so the driven
path cannot drift from the one the markers were placed against.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from whycode_sim import scene_config
from whycode_sim.geometry import (
    point_at_arclength,
    project_onto_path,
    pure_pursuit_curvature,
    quaternion_to_matrix,
    resample_path,
)


class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__("waypoint_follower")

        self.declare_parameter("scene_config", "")
        self.declare_parameter("control_rate_hz", 20.0)

        config_path = self.get_parameter("scene_config").value
        if not config_path:
            raise RuntimeError("the scene_config parameter is required")

        config = scene_config.load(config_path)
        self.path = config.path
        self.waypoints = [list(point) for point in config.path.waypoints]
        self.loop = getattr(self.path, "loop", True)
        _, self.total_length = resample_path(self.waypoints, self.loop)

        self.pose = None
        self.finished = False
        self.laps = 0
        self.last_arclength = 0.0

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom_ground_truth", self._on_odom, 10)

        rate = self.get_parameter("control_rate_hz").value
        self.create_timer(1.0 / rate, self._on_tick)

        self.get_logger().info(
            f"following {len(self.waypoints)} waypoints, {self.total_length:.1f} m, "
            f"loop={self.loop}, at {self.path.speed_mps} m/s"
        )

    def _on_odom(self, msg):
        position = msg.pose.pose.position
        rotation = msg.pose.pose.orientation
        self.pose = (
            np.array([position.x, position.y]),
            quaternion_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _on_tick(self):
        if self.pose is None or self.finished:
            return

        position, rotation = self.pose
        arclength = project_onto_path(self.waypoints, position, self.loop)

        if self.loop:
            # A large backwards jump in arclength means the projection wrapped past the
            # seam, which is one completed lap.
            if self.last_arclength - arclength > self.total_length / 2.0:
                self.laps += 1
                self.get_logger().info(f"completed lap {self.laps}")
        elif self.total_length - arclength < self.path.goal_tolerance_m:
            self.get_logger().info("reached the end of the path")
            self.finished = True
            self.publisher.publish(Twist())
            return
        self.last_arclength = arclength

        target = point_at_arclength(
            self.waypoints, arclength + self.path.lookahead_m, self.loop
        )

        # Into the body frame. Only yaw matters on a planar path, and the first rotation
        # column gives it without a euler conversion.
        heading = math.atan2(rotation[1, 0], rotation[0, 0])
        delta = np.asarray(target) - position
        cos_heading, sin_heading = math.cos(-heading), math.sin(-heading)
        lookahead_body = (
            cos_heading * delta[0] - sin_heading * delta[1],
            sin_heading * delta[0] + cos_heading * delta[1],
        )

        curvature = pure_pursuit_curvature(lookahead_body)
        speed = self.path.speed_mps / (1.0 + self.path.corner_slowdown * abs(curvature))
        yaw_rate = max(
            -self.path.max_yaw_rate_rps,
            min(self.path.max_yaw_rate_rps, curvature * speed),
        )

        command = Twist()
        command.linear.x = speed
        command.angular.z = yaw_rate
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())  # do not leave the robot driving
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
