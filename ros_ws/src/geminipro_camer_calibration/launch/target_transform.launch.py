"""坐标转换单独启动；不启动相机或机器人。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("extrinsic_file", default_value=""),
            DeclareLaunchArgument("observation_frame", default_value="terminal_link_1"),
            Node(
                package="geminipro_camer_calibration",
                executable="target_transform_node.py",
                parameters=[
                    {
                        "extrinsic_file": ParameterValue(
                            LaunchConfiguration("extrinsic_file"), value_type=str
                        ),
                        "observation_frame": ParameterValue(
                            LaunchConfiguration("observation_frame"), value_type=str
                        ),
                    }
                ],
                output="screen",
            ),
        ]
    )
