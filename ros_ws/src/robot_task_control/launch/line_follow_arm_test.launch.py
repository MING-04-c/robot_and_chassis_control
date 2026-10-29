"""先到机械臂初始点，再循迹、确认停稳，按 YAML 执行后续点位。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("robot_task_control")
    return LaunchDescription([
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("workflow_file", default_value=PathJoinSubstitution(
            [share, "config", "line_follow_arm_test.yaml"])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [share, "launch", "robot_system.launch.py"])),
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
