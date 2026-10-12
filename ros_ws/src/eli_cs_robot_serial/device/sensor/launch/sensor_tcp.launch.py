import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    default_config = os.path.join(get_package_share_directory("eli_cs_robot_serial"), "device", "sensor", "config", "sensor_tcp.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("host", default_value="169.254.199.80"),
        DeclareLaunchArgument("port", default_value="9001"),
        DeclareLaunchArgument("frame_id", default_value="force_sensor"),
        Node(package="eli_cs_robot_serial", executable="sensor_node", name="sensor_node", output="screen",
             parameters=[
                 LaunchConfiguration("config_file"),
                 {
                     "transport": "tcp",
                     "host": LaunchConfiguration("host"),
                     "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                     "frame_id": LaunchConfiguration("frame_id"),
                 },
             ]),
    ])
