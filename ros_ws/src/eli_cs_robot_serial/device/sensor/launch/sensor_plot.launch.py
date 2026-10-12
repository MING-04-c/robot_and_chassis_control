import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_data_directory = os.path.join(
        get_package_share_directory("eli_cs_robot_serial"), "device", "sensor", "data")
    return LaunchDescription([
        DeclareLaunchArgument("data_directory", default_value=default_data_directory),
        Node(
            package="eli_cs_robot_serial",
            executable="sensor_plot.py",
            name="sensor_plot",
            output="screen",
            parameters=[{"data_directory": LaunchConfiguration("data_directory")}],
        ),
    ])
