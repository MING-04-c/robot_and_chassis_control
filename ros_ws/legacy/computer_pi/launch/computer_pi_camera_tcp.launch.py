from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("host", default_value="169.254.199.100"),
        DeclareLaunchArgument("port", default_value="9002"),
        DeclareLaunchArgument("local_ip", default_value="169.254.199.250"),
        DeclareLaunchArgument(
            "compressed_topic", default_value="/computer_pi/camera/image/compressed"
        ),
        DeclareLaunchArgument(
            "image_topic", default_value="/computer_pi/camera/image"
        ),
        DeclareLaunchArgument(
            "capture_service", default_value="/computer_pi/camera/capture"
        ),
        DeclareLaunchArgument(
            "snapshot_directory",
            default_value="/home/robot/Pictures/computer_pi_camera",
        ),
        DeclareLaunchArgument(
            "latest_file", default_value="/tmp/computer_pi_camera/latest.jpg"
        ),
        DeclareLaunchArgument("reconnect_period", default_value="2.0"),
        DeclareLaunchArgument("jpeg_quality", default_value="85"),
        DeclareLaunchArgument("show_image", default_value="true"),
    ]
    receiver = Node(
        package="eli_cs_robot_serial",
        executable="camera_tcp_node.py",
        name="computer_pi_camera_tcp_node",
        output="screen",
        parameters=[{
            "host": LaunchConfiguration("host"),
            "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
            "local_ip": ParameterValue(LaunchConfiguration("local_ip"), value_type=str),
            "compressed_topic": LaunchConfiguration("compressed_topic"),
            "latest_file": ParameterValue(LaunchConfiguration("latest_file"), value_type=str),
            "reconnect_period": ParameterValue(
                LaunchConfiguration("reconnect_period"), value_type=float
            ),
            "jpeg_quality": ParameterValue(
                LaunchConfiguration("jpeg_quality"), value_type=int
            ),
        }],
    )
    image_bridge = Node(
        package="eli_cs_robot_serial",
        executable="camera_viewer.py",
        name="computer_pi_camera_viewer",
        output="screen",
        parameters=[{
            "compressed_topic": LaunchConfiguration("compressed_topic"),
            "image_topic": LaunchConfiguration("image_topic"),
            "capture_service": LaunchConfiguration("capture_service"),
            "snapshot_directory": ParameterValue(
                LaunchConfiguration("snapshot_directory"), value_type=str
            ),
            "show_image": ParameterValue(
                LaunchConfiguration("show_image"), value_type=bool
            ),
        }],
    )
    return LaunchDescription(arguments + [receiver, image_bridge])
