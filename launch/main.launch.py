from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("whycode_sim")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="docking.world",
        description="World file from the package worlds/ directory",
    )

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Enable Gazebo GUI (true/false).",
    )

    world_path = PathJoinSubstitution([pkg_share, "worlds", LaunchConfiguration("world")])
    model_path = PathJoinSubstitution([pkg_share, "models"])
    world_dir = PathJoinSubstitution([pkg_share, "worlds"])
    bridge_config = PathJoinSubstitution([pkg_share, "config", "bridge.yaml"])
    camera_model = PathJoinSubstitution([pkg_share, "models", "camera_platform", "model.sdf"])

    set_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            model_path,
            ":",
            world_dir,
            ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    set_sim_time = SetParameter(name="use_sim_time", value=True)

    gz_args_value = PythonExpression(
        [
            "'-r' if '",
            LaunchConfiguration("gui"),
            "' == 'true' else '-r -s --headless-rendering'",
        ]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": [gz_args_value, " ", world_path]
        }.items(),
    )

    spawn_camera = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "camera", "-file", camera_model],
        output="screen",
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_bridge"), "/launch/ros_gz_bridge.launch.py"]
        ),
        launch_arguments={
            "bridge_name": "ros_gz_bridge",
            "config_file": bridge_config,
        }.items(),
    )

    return LaunchDescription(
        [
            world_arg,
            gui_arg,
            set_resource_path,
            set_sim_time,
            gz_sim,
            spawn_camera,
            bridge,
        ]
    )
