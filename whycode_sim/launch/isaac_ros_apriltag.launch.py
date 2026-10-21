"""NVIDIA's GPU AprilTag detector (Isaac ROS) on the benchmark's camera, for the comparison.

Runs in the bench's isaac-ros container, not in ros-dev, which has no Isaac ROS:

    docker compose --profile isaac-ros run --rm isaac-ros bash -c "source /opt/ros/jazzy/setup.bash
        && ros2 launch /root/workspace/src/whycode_sim/whycode_sim/launch/isaac_ros_apriltag.launch.py"

Each tag's pose goes out on /tf as `tag36h11:<id>` in the image's frame. The node takes one tag
size for every tag; evaluate.py rescales a tag of another size.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    size_arg = DeclareLaunchArgument(
        "size", default_value="0.2", description="Tag edge, the black square (m)"
    )
    backend_arg = DeclareLaunchArgument(
        "backends", default_value="CUDA", description="CUDA, CPU or PVA"
    )

    apriltag = ComposableNode(
        package="isaac_ros_apriltag",
        plugin="nvidia::isaac_ros::apriltag::AprilTagNode",
        name="isaac_apriltag",
        parameters=[
            {
                "size": ParameterValue(LaunchConfiguration("size"), value_type=float),
                "max_tags": 64,
                "tag_family": "tag36h11",
                "backends": LaunchConfiguration("backends"),
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("image", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
        ],
    )
    container = ComposableNodeContainer(
        name="isaac_apriltag_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[apriltag],
        output="screen",
    )
    return LaunchDescription([size_arg, backend_arg, container])
