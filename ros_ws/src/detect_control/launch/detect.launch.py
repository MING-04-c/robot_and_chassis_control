"""启动检测任务接口；复用已运行的真机驱动、MoveIt、导纳和力传感器。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    overrides = {}
    targets_file = LaunchConfiguration("targets_file").perform(context)
    if targets_file:
        overrides["targets_file"] = targets_file
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("eli_cs_robot_serial"), "launch", "detactor.launch.py"
            ])),
            launch_arguments={"config_file": LaunchConfiguration("detactor_config_file"),
                              "auto_request": "false",
                              "data_directory": "~/.ros/eli_cs_robot_serial/detactor"}.items(),
            condition=IfCondition(LaunchConfiguration("start_thickness_detector")),
        ),
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
            DeclareLaunchArgument("start_thickness_detector", default_value="true"),
            DeclareLaunchArgument("detactor_config_file", default_value=PathJoinSubstitution([
                FindPackageShare("eli_cs_robot_serial"), "device", "detactor", "config", "detactor.yaml"
            ])),
            OpaqueFunction(function=launch_setup),
        ]
    )
