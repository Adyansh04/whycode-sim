"""Ground truth, path following, and optional bag recording for the Isaac Sim scene.

Isaac Sim itself is a separate container and is not started from here; bring it up with
`./scripts/manage.sh start isaac`. This launch file runs the ROS-side half that consumes
what Isaac publishes.

With `laps:=N` the robot drives exactly N laps from the spawn and stops there, and the launch
ends with it, so a recording finalises on its own.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

RECORDED_TOPICS = [
    "/camera/image_raw",
    "/camera/camera_info",
    "/clock",
    "/odom_ground_truth",
    "/camera_ground_truth",
    "/marker_occlusion",
    "/marker_ground_truth",
    "/tf",
    "/tf_static",
]


def generate_launch_description():
    package_share = FindPackageShare("whycode_sim")

    scene_config_arg = DeclareLaunchArgument(
        "scene_config",
        default_value=PathJoinSubstitution([package_share, "config", "scene.yaml"]),
        description="Scene layout; must be the same file the Isaac container loaded",
    )

    follow_arg = DeclareLaunchArgument(
        "follow",
        default_value="true",
        description="Drive the configured loop (true/false)",
    )

    record_arg = DeclareLaunchArgument(
        "record",
        default_value="false",
        description="Record the benchmark topics to a bag (true/false)",
    )

    bag_prefix_arg = DeclareLaunchArgument(
        "bag_prefix",
        default_value="/root/data/whycode_loop",
        description="Output path prefix for the recorded bag",
    )

    laps_arg = DeclareLaunchArgument(
        "laps",
        default_value="1",
        description="Laps to drive before stopping and ending the launch; 0 drives until stopped",
    )

    lap_speeds_arg = DeclareLaunchArgument(
        "lap_speeds",
        default_value="",
        description="Per-lap speeds in m/s, one per lap, e.g. 0.5,0.8,1.2; empty uses the scene's",
    )

    # Isaac publishes /clock, so everything downstream must follow simulation time.
    use_sim_time = SetParameter(name="use_sim_time", value=True)

    ground_truth_node = Node(
        package="whycode_sim",
        executable="ground_truth_node",
        name="marker_ground_truth",
        output="screen",
        parameters=[{"scene_config": LaunchConfiguration("scene_config")}],
    )

    waypoint_follower_node = Node(
        condition=IfCondition(LaunchConfiguration("follow")),
        package="whycode_sim",
        executable="waypoint_follower_node",
        name="waypoint_follower",
        output="screen",
        parameters=[
            {
                "scene_config": LaunchConfiguration("scene_config"),
                "laps": LaunchConfiguration("laps"),
                # A single speed would otherwise be parsed as a float and rejected.
                "lap_speeds": ParameterValue(
                    LaunchConfiguration("lap_speeds"), value_type=str
                ),
            }
        ],
    )

    # The follower exits only after its last lap, or on a failure: either way the run is over.
    end_with_follower = RegisterEventHandler(
        OnProcessExit(
            target_action=waypoint_follower_node,
            on_exit=[EmitEvent(event=Shutdown(reason="waypoint follower finished"))],
        )
    )

    # MCAP's own zstd chunks rather than rosbag2's --compression-mode, which wraps the
    # file in a layer a plain SequentialReader cannot open -- and that is how the analysis
    # reads it. Measured on a driving loop: 6.5 GiB of chunk payload becomes 3.5 GiB.
    bag_recorder = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("record")),
        cmd=[
            "ros2",
            "bag",
            "record",
            # Log on sim time: playback schedules by log time, and a wall-clock bag replays
            # the recording session's pace on a /clock unrelated to the headers.
            "--use-sim-time",
            "--storage-config-file",
            PathJoinSubstitution([package_share, "config", "mcap_zstd.yaml"]),
            "-o",
            LaunchConfiguration("bag_prefix"),
            *RECORDED_TOPICS,
        ],
        output="screen",
        # Room to flush the last chunk and write metadata.yaml before a SIGTERM.
        sigterm_timeout="30",
    )

    return LaunchDescription(
        [
            scene_config_arg,
            follow_arg,
            record_arg,
            bag_prefix_arg,
            laps_arg,
            lap_speeds_arg,
            use_sim_time,
            ground_truth_node,
            waypoint_follower_node,
            end_with_follower,
            bag_recorder,
        ]
    )
