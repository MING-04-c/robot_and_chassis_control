"""Start Gemini Pro using the configuration verified on ROS 2 Jazzy."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('orbbec_camera'),
                'config',
                'gemini_pro_jazzy.yaml',
            ]),
            description='Gemini Pro parameter YAML file',
        ),
        Node(
            package='orbbec_camera',
            executable='orbbec_camera_node',
            name='orbbec_camera_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
