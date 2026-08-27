"""Replays a recorded bag and shows it in RViz.

Reads the bag only, with Isaac stopped, so it also tests whether a recording stands on
its own.

    ros2 launch whycode_sim replay.launch.py bag:=/root/data/warehouse_loop

ground_truth_viz_node converts the custom ground-truth message into markers RViz can
render. What to look for: green plates sitting on the printed markers in the Camera view.
Offset, or on a different axis, means the frame convention is wrong.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("whycode_sim")

    bag_arg = DeclareLaunchArgument(
        "bag",
        default_value="/root/data/warehouse_loop",
        description="Bag directory to replay",
    )

    rate_arg = DeclareLaunchArgument(
        "rate",
        default_value="1.0",
        description="Playback rate",
    )

    loop_arg = DeclareLaunchArgument(
        "loop",
        default_value="true",
        description="Restart the bag when it ends (true/false)",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Open RViz (true/false)",
    )

    marker_size_arg = DeclareLaunchArgument(
        "marker_size_m",
        default_value="0.2",
        description="Marker edge length, for drawing the plates at the right size",
    )

    # The bag carries /clock; without this the markers and the image are placed against
    # wall time and drift apart.
    use_sim_time = SetParameter(name="use_sim_time", value=True)

    player = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("bag"),
            "--clock",
            "--rate",
            LaunchConfiguration("rate"),
            "--loop",
        ],
        condition=IfCondition(LaunchConfiguration("loop")),
        output="screen",
    )

    player_once = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("bag"),
            "--clock",
            "--rate",
            LaunchConfiguration("rate"),
        ],
        condition=UnlessCondition(LaunchConfiguration("loop")),
        output="screen",
    )

    viz = Node(
        package="whycode_sim",
        executable="ground_truth_viz_node",
        name="marker_ground_truth_viz",
        output="screen",
        parameters=[{"marker_size_m": LaunchConfiguration("marker_size_m")}],
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration("rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([package_share, "config", "replay.rviz"]),
        ],
    )

    return LaunchDescription(
        [
            bag_arg,
            rate_arg,
            loop_arg,
            rviz_arg,
            marker_size_arg,
            use_sim_time,
            player,
            player_once,
            viz,
            rviz,
        ]
    )
