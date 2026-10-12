import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("eli_cs_robot_serial"),
        "device",
        "computer_pi",
        "config",
        "computer_pi_tcp.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("host", default_value="169.254.199.100"),
        DeclareLaunchArgument("port", default_value="9001"),
        Node(
            package="eli_cs_robot_serial",
            executable="tcp_node",
            name="computer_pi_tcp_node",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "mode": "client",
                    "host": LaunchConfiguration("host"),
                    "port": ParameterValue(
                        LaunchConfiguration("port"), value_type=int
                    ),
                },
            ],
        ),
    ])
