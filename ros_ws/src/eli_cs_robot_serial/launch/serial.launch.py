import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Only the YAML filename is needed, for example ttyCH341USB0.yaml.
        # Absolute paths remain supported for compatibility.
        DeclareLaunchArgument("config_file", default_value="serial.yaml"),
        DeclareLaunchArgument(
            "publish_wrench",
            default_value="false",
            description="Parse serial lines as Fx Fy Fz Tx Ty Tz and publish WrenchStamped.",
        ),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    if not os.path.isabs(config_file):
        config_file = os.path.join(
            FindPackageShare("eli_cs_robot_serial").perform(context),
            "config", config_file,
        )
    return [Node(
        package="eli_cs_robot_serial",
        executable="serial_node",
        name="serial_node",
        output="screen",
        parameters=[
            config_file,
            {"publish_wrench": LaunchConfiguration("publish_wrench")},
        ],
    )]
