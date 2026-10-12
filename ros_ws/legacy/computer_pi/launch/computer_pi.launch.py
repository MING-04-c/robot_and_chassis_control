import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("eli_cs_robot_serial")
    default_config = os.path.join(
        package_share, "device", "computer_pi", "config", "computer_pi_tcp.yaml"
    )

    arguments = [
        DeclareLaunchArgument("host", default_value="169.254.199.100"),
        DeclareLaunchArgument("local_ip", default_value="169.254.199.250"),
        DeclareLaunchArgument("status_port", default_value="9001"),
        DeclareLaunchArgument("image_port", default_value="9002"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("enable_status", default_value="true"),
        DeclareLaunchArgument("enable_camera", default_value="true"),
        DeclareLaunchArgument("show_image", default_value="true"),
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
        DeclareLaunchArgument("status_reconnect_period", default_value="1.0"),
        DeclareLaunchArgument("image_reconnect_period", default_value="2.0"),
        DeclareLaunchArgument("jpeg_quality", default_value="85"),
        DeclareLaunchArgument("offline_timeout", default_value="3.0"),
        DeclareLaunchArgument("heartbeat_period", default_value="1.0"),
    ]

    status_transport = Node(
        package="eli_cs_robot_serial",
        executable="tcp_node",
        name="computer_pi_tcp_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_status")),
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "mode": "client",
                "host": LaunchConfiguration("host"),
                "port": ParameterValue(
                    LaunchConfiguration("status_port"), value_type=int
                ),
                "reconnect_period": ParameterValue(
                    LaunchConfiguration("status_reconnect_period"),
                    value_type=float,
                ),
            },
        ],
    )
    status_monitor = Node(
        package="eli_cs_robot_serial",
        executable="computer_pi_status_node.py",
        name="computer_pi_status_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_status")),
        parameters=[{
            "offline_timeout": ParameterValue(
                LaunchConfiguration("offline_timeout"), value_type=float
            ),
            "heartbeat_period": ParameterValue(
                LaunchConfiguration("heartbeat_period"), value_type=float
            ),
        }],
    )
    camera_receiver = Node(
        package="eli_cs_robot_serial",
        executable="camera_tcp_node.py",
        name="computer_pi_camera_tcp_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_camera")),
        parameters=[{
            "host": LaunchConfiguration("host"),
            "port": ParameterValue(
                LaunchConfiguration("image_port"), value_type=int
            ),
            "local_ip": ParameterValue(
                LaunchConfiguration("local_ip"), value_type=str
            ),
            "compressed_topic": LaunchConfiguration("compressed_topic"),
            "latest_file": ParameterValue(
                LaunchConfiguration("latest_file"), value_type=str
            ),
            "reconnect_period": ParameterValue(
                LaunchConfiguration("image_reconnect_period"), value_type=float
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
        condition=IfCondition(LaunchConfiguration("enable_camera")),
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

    return LaunchDescription(
        arguments
        + [status_transport, status_monitor, camera_receiver, image_bridge]
    )
