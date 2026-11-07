import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("eli_cs_robot_serial"),
        "device", "detactor", "config", "detactor.yaml",
    )
    default_data_directory = "~/.ros/eli_cs_robot_serial/detactor"
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("transport", default_value="serial", choices=["tcp", "serial"]),
        DeclareLaunchArgument("device", default_value="/dev/ttyCH341USB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("task_once", default_value="false"),
        DeclareLaunchArgument("host", default_value="169.254.199.90"),
        DeclareLaunchArgument("port", default_value="9001"),
        DeclareLaunchArgument(
            "auto_request", default_value="true",
            description="Override automatic periodic acquisition: true or false.",
        ),
        DeclareLaunchArgument(
            "request_period", default_value="1.0",
            description="Override acquisition period in seconds.",
        ),
        DeclareLaunchArgument(
            "data_directory", default_value=default_data_directory,
            description="Override the detector data output directory.",
        ),
        Node(
            package="eli_cs_robot_serial",
            executable="detactor_node",
            name="detactor_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "auto_request": ParameterValue(LaunchConfiguration("auto_request"), value_type=bool),
                    "task_once": ParameterValue(LaunchConfiguration("task_once"), value_type=bool),
                    "request_period": ParameterValue(LaunchConfiguration("request_period"), value_type=float),
                    "require_transport_connection": ParameterValue(
                        True, value_type=bool),
                    "data_directory": LaunchConfiguration("data_directory"),
                },
            ],
        ),
        Node(
            package="eli_cs_robot_serial",
            executable="serial_node",
            name="serial_node",
            output="screen",
            parameters=[config_file, {
                "device": LaunchConfiguration("device"),
                "baudrate": ParameterValue(LaunchConfiguration("baudrate"), value_type=int),
            }],
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration("transport"), "serial")),
        ),
        Node(
            package="eli_cs_robot_serial", executable="tcp_node", name="tcp_node",
            output="screen",
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration("transport"), "tcp")),
            parameters=[config_file, {
                "mode": "client", "host": LaunchConfiguration("host"),
                "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
            }],
        ),
    ])
