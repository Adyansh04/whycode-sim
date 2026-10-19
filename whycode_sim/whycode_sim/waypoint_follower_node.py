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
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future

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
        # 0 drives the loop until stopped. N drives exactly N laps from the spawn, stops there
        # and completes `done`, so a recording covers every marker the same number of times.
        self.declare_parameter("laps", 0)
        # Optional per-lap speeds, comma separated, one per lap ("0.5,0.8,1.2"). Empty uses
        # path.speed_mps throughout.
        self.declare_parameter("lap_speeds", "")

        config_path = self.get_parameter("scene_config").value
        if not config_path:
            raise RuntimeError("the scene_config parameter is required")

        config = scene_config.load(config_path)
        self.path = config.path
        self.waypoints = [list(point) for point in config.path.waypoints]
        self.loop = getattr(self.path, "loop", True)
        _, self.total_length = resample_path(self.waypoints, self.loop)

        laps = self.get_parameter("laps").value
        self.target_m = laps * self.total_length if self.loop and laps > 0 else None
        speeds = self.get_parameter("lap_speeds").value
        self.lap_speeds = (
            [float(v) for v in speeds.split(",")] if speeds.strip() else []
        )
        if self.lap_speeds and len(self.lap_speeds) != laps:
            raise RuntimeError(
                f"lap_speeds lists {len(self.lap_speeds)} speeds for laps={laps}; give one per lap"
            )
        self.start_delay = Duration(seconds=getattr(self.path, "start_delay_s", 0.0))

        self.pose = None
        self.started_at = None
        self.last_arclength = None
        self.progress_m = 0.0
        self.laps_logged = 0
        self.done = Future()

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom_ground_truth", self._on_odom, 10)

        rate = self.get_parameter("control_rate_hz").value
        self.create_timer(1.0 / rate, self._on_tick)

        self.get_logger().info(
            f"following {len(self.waypoints)} waypoints, {self.total_length:.1f} m, "
            f"loop={self.loop}, at {self.lap_speeds or self.path.speed_mps} m/s, "
            f"laps={laps or 'unbounded'}"
        )

    def _on_odom(self, msg):
        position = msg.pose.pose.position
        rotation = msg.pose.pose.orientation
        self.pose = (
            np.array([position.x, position.y]),
            quaternion_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _on_tick(self):
        if self.pose is None or self.done.done():
            return

        # Hold at the spawn first, so a recording opens on the robot at rest.
        now = self.get_clock().now()
        if self.started_at is None:
            self.started_at = now
        if now - self.started_at < self.start_delay:
            return

        position, rotation = self.pose
        arclength = project_onto_path(self.waypoints, position, self.loop)
        self._advance(arclength)

        if self.target_m is not None and self.progress_m >= self.target_m:
            self._finish(f"completed {self.laps_logged} laps")
            return
        if not self.loop and self.total_length - arclength < self.path.goal_tolerance_m:
            self._finish("reached the end of the path")
            return

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
        speed = self._cruise_speed() / (
            1.0 + self.path.corner_slowdown * abs(curvature)
        )
        yaw_rate = max(
            -self.path.max_yaw_rate_rps,
            min(self.path.max_yaw_rate_rps, curvature * speed),
        )

        command = Twist()
        command.linear.x = speed
        command.angular.z = yaw_rate
        self.publisher.publish(command)

    def _advance(self, arclength):
        """Accumulate distance along the path, unwrapped across the loop's seam.

        The spawn sits on the seam, where the projection flips between 0 and the full loop
        length. Counting laps by seam crossings therefore fires on the first tick; summing
        signed steps does not.
        """
        if self.last_arclength is not None:
            step = arclength - self.last_arclength
            if self.loop and step < -self.total_length / 2.0:
                step += self.total_length
            elif self.loop and step > self.total_length / 2.0:
                step -= self.total_length
            self.progress_m += step
            laps = int(max(self.progress_m, 0.0) // self.total_length)
            if laps > self.laps_logged:
                self.laps_logged = laps
                upcoming = (
                    f", next at {self._cruise_speed():.2f} m/s"
                    if self.target_m is None or self.progress_m < self.target_m
                    else ""
                )
                self.get_logger().info(f"completed lap {laps}{upcoming}")
        self.last_arclength = arclength

    def _cruise_speed(self):
        if not self.lap_speeds:
            return self.path.speed_mps
        return self.lap_speeds[min(self.laps_logged, len(self.lap_speeds) - 1)]

    def _finish(self, reason):
        self.publisher.publish(Twist())
        self.get_logger().info(f"{reason}; stopped")
        self.done.set_result(True)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin_until_future_complete(node, node.done)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())  # do not leave the robot driving
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
