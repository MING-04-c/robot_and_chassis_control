"""启动检测任务接口；复用已运行的真机驱动、MoveIt、导纳和力传感器。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    overrides = {}
    targets_file = LaunchConfiguration("targets_file").perform(context)
    if targets_file:
        overrides["targets_file"] = targets_file
    return [
        Node(
            package="detect_control",
            executable="detect_controller.py",
            name="detect_controller",
            parameters=[LaunchConfiguration("config"), overrides],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("detect_control"), "config", "detect.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "targets_file",
                default_value="",
                description="Override YAML target storage path; empty uses config.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
