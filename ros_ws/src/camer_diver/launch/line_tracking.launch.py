import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("camer_diver")
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="camera"),
        DeclareLaunchArgument("video_device", default_value="/dev/video0"),
        DeclareLaunchArgument("camera_frame", default_value="camera_optical_frame"),
        DeclareLaunchArgument("camera_params", default_value=os.path.join(share, "config", "camera.yaml")),
        DeclareLaunchArgument("tracker_params", default_value=os.path.join(share, "config", "line_tracker.yaml")),
        DeclareLaunchArgument("show_window", default_value="true", description="Local desktop OpenCV window"),
        DeclareLaunchArgument("demo", default_value="false", description="Synthetic images, not a real camera"),
        Node(
            package="camer_diver", executable="camera_node", name="camera_node",
            namespace=namespace, condition=UnlessCondition(LaunchConfiguration("demo")),
            parameters=[LaunchConfiguration("camera_params"), {"publish_raw": True, "video_device": LaunchConfiguration("video_device"),
                         "frame_id": LaunchConfiguration("camera_frame")}], output="screen",
        ),
        Node(
            package="camer_diver", executable="line_demo.py", name="line_demo",
            namespace=namespace, condition=IfCondition(LaunchConfiguration("demo")), output="screen",
        ),
        Node(
            package="camer_diver", executable="line_tracker_node", name="line_tracker",
            namespace=namespace,
            parameters=[LaunchConfiguration("tracker_params"), {
                "show_window": ParameterValue(LaunchConfiguration("show_window"), value_type=bool),
            }], output="screen",
        ),
    ])
