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
            DeclareLaunchArgument(
                "workflow_file",
                default_value=PathJoinSubstitution(
                    [share, "config", "line_follow_only.yaml"]
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
                    "start_chassis": "true",
                    "chassis_pid_enabled": "false",
                    "use_velocity_mux": "true",
                    "start_arm_driver": "false",
                    "start_moveit": "false",
                    "launch_force_sensor": "false",
                    "launch_arm_executor": "false",
                    "start_line_camera": "true",
                    "start_hk_camera": "false",
                    "start_image_transfer": "false",
                }.items(),
            ),
        ]
    )
