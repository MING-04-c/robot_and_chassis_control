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
            DeclareLaunchArgument(
                "line_stop_marker_target_ratio",
                default_value="0.46",
                description="停车横线纵坐标/图高；前方朝down时调小可多前进一点，up时相反",
            ),
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
                            "internal",
                            "robot_system.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "auto_start": LaunchConfiguration("auto_start"),
                    "workflow_file": LaunchConfiguration("workflow_file"),
                    "line_stop_marker_target_ratio": LaunchConfiguration("line_stop_marker_target_ratio"),
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
