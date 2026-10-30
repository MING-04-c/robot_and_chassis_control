"""整体联动：初始点、循迹、机械臂点位、采图发送；可选 TCP 远程开始。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, NotSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("robot_task_control")
    return LaunchDescription([
        DeclareLaunchArgument("remote_control", default_value="false", choices=["true", "false"],
                             description="true: 加载TCP并等待主机开始；false: 就绪后自动执行"),
        DeclareLaunchArgument("auto_start", default_value=NotSubstitution(
            LaunchConfiguration("remote_control"))),
        DeclareLaunchArgument("key_file", default_value=""),
        DeclareLaunchArgument("workflow_file", default_value=PathJoinSubstitution(
            [share, "config", "line_follow_arm_test.yaml"])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("eli_cs_robot_serial"), "launch", "industrial_pc.launch.py",
            ])),
            condition=IfCondition(LaunchConfiguration("remote_control")),
            launch_arguments={"key_file": LaunchConfiguration("key_file"),
                              "start_hk_transfer": "false"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [share, "launch", "internal", "robot_system.launch.py"])),
            launch_arguments={
                "workflow_file": LaunchConfiguration("workflow_file"),
                "auto_start": LaunchConfiguration("auto_start"),
                "require_arm_ready": "true", "check_devices_on_startup": "true",
                "chassis_pid_enabled": "false", "use_velocity_mux": "true",
                "launch_arm_executor": "true", "arm_use_cartesian_path": "false",
                "start_hk_camera": "false", "start_image_transfer": "false",
                "start_line_camera": "true",
            }.items(),
        ),
    ])
