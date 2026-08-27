"""The simulation half plus the WhyCode detector, for watching detection live.

Replaces the Gazebo-era whycode_demo.launch.py. Isaac Sim runs in its own container and
is started separately with `./scripts/manage.sh start isaac`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_share = FindPackageShare("whycode_sim")
    vision_share = FindPackageShare("whycode_vision")

    scene_config_arg = DeclareLaunchArgument(
        "scene_config",
        default_value=PathJoinSubstitution([sim_share, "config", "scene.yaml"]),
        description="Scene layout; must be the same file the Isaac container loaded",
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [vision_share, "config", "whycon_config_sim.yaml"]
        ),
        description="Path to the WhyCode detector configuration",
    )

    image_view_arg = DeclareLaunchArgument(
        "image_view",
        default_value="true",
        description="Show the annotated detector output (true/false)",
    )

    use_composition_arg = DeclareLaunchArgument(
        "use_composition",
        default_value="false",
        description="Run the detector as a composable node",
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

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sim_share, "/launch/sim.launch.py"]),
        launch_arguments={
            "scene_config": LaunchConfiguration("scene_config"),
            "follow": LaunchConfiguration("follow"),
            "record": LaunchConfiguration("record"),
        }.items(),
    )

    detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vision_share, "/launch/whycon.launch.py"]),
        launch_arguments={
            "config_file": LaunchConfiguration("config_file"),
            "image_view": LaunchConfiguration("image_view"),
            "use_composition": LaunchConfiguration("use_composition"),
        }.items(),
    )

    return LaunchDescription(
        [
            scene_config_arg,
            config_file_arg,
            image_view_arg,
            use_composition_arg,
            follow_arg,
            record_arg,
            sim_launch,
            detector_launch,
        ]
    )
