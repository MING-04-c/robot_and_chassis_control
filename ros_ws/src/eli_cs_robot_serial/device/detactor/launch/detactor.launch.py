import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("eli_cs_robot_serial"),
        "device", "detactor", "config", "detactor.yaml",
    )
    # --symlink-install 时 realpath 指向源码中的 device/detactor/data；
    # 普通安装时则指向安装空间中的同名目录。
    launch_directory = os.path.dirname(os.path.realpath(__file__))
    source_data_directory = os.path.join(launch_directory, "..", "data")
    installed_data_directory = os.path.join(
        launch_directory, "..", "device", "detactor", "data"
    )
    default_data_directory = (
        source_data_directory
        if os.path.isdir(source_data_directory)
        else installed_data_directory
    )
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
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
                    "auto_request": LaunchConfiguration("auto_request"),
                    "request_period": LaunchConfiguration("request_period"),
                    "data_directory": LaunchConfiguration("data_directory"),
                },
            ],
        ),
        Node(
            package="eli_cs_robot_serial",
            executable="serial_node",
            name="serial_node",
            output="screen",
            parameters=[config_file],
        ),
    ])
