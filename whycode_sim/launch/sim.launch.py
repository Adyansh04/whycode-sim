"""Ground truth, path following, and optional bag recording for the Isaac Sim scene.

Isaac Sim itself is a separate container and is not started from here; bring it up with
`./scripts/manage.sh start isaac`. This launch file runs the ROS-side half that consumes
what Isaac publishes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
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
        parameters=[{"scene_config": LaunchConfiguration("scene_config")}],
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
            "--storage-config-file",
            PathJoinSubstitution([package_share, "config", "mcap_zstd.yaml"]),
            "-o",
            LaunchConfiguration("bag_prefix"),
            *RECORDED_TOPICS,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            scene_config_arg,
            follow_arg,
            record_arg,
            bag_prefix_arg,
            use_sim_time,
            ground_truth_node,
            waypoint_follower_node,
            bag_recorder,
        ]
    )
