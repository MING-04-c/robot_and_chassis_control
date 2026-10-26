"""Launch the selected control profile without unrelated devices."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("robot_task_control")
    return LaunchDescription(
        [
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument("start_arm_driver", default_value="true"),
            DeclareLaunchArgument("start_moveit", default_value="true"),
            DeclareLaunchArgument(
                "workflow_file",
                default_value=PathJoinSubstitution(
                    [share, "config", "arm_admittance_line_follow.yaml"]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            share,
                            "launch",
                            "robot_system.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "workflow_file": LaunchConfiguration("workflow_file"),
                    "auto_start": LaunchConfiguration("auto_start"),
                    "require_arm_ready": "true",
                    "check_devices_on_startup": "true",
                    "start_chassis": "true",
                    "chassis_pid_enabled": "false",
                    "use_velocity_mux": "true",
                    "start_arm_driver": LaunchConfiguration("start_arm_driver"),
                    "start_moveit": LaunchConfiguration("start_moveit"),
                    "launch_force_sensor": "true",
                    "launch_arm_executor": "false",
                    "start_line_camera": "true",
                    "start_hk_camera": "false",
                    "start_image_transfer": "false",
                }.items(),
            ),
        ]
    )
