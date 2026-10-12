import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    default_config = os.path.join(get_package_share_directory("eli_cs_robot_serial"), "device", "sensor", "config", "sensor_usb.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("frame_id", default_value="force_sensor"),
        Node(package="eli_cs_robot_serial", executable="sensor_node", name="sensor_node", output="screen",
             parameters=[
                 LaunchConfiguration("config_file"),
                 {"frame_id": LaunchConfiguration("frame_id")},
             ]),
    ])
