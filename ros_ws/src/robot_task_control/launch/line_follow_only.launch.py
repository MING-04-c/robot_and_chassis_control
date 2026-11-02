"""Launch the selected control profile without unrelated devices."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("robot_task_control")
    return LaunchDescription(
        [
            DeclareLaunchArgument('round_trip', default_value='false',
                                 description='前进途中站等待，最终横线停稳封存，执行中间步骤后倒车回出发点'),
            DeclareLaunchArgument("drive_direction", default_value="forward", choices=["forward", "reverse"],
                                 description="同一前置相机，forward前进或reverse低速后退"),
            DeclareLaunchArgument("line_follow_speed", default_value="0.075"),
            DeclareLaunchArgument("route_visual_finish", default_value="true"),
            DeclareLaunchArgument("route_camera_stop", default_value="true"),
            DeclareLaunchArgument("route_file", default_value="~/.ros/line_follow/last_route.json"),
            DeclareLaunchArgument("reverse_mode", default_value="recorded_route", choices=["recorded_route", "camera"]),
            DeclareLaunchArgument("reverse_steering_sign", default_value="-1.0",
                                 description="仅倒车转向修正；-1翻转，+1标准映射"),
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument(
                "line_stop_marker_target_ratio",
                default_value="0.46",
                description="停车横线纵坐标/图高；前方朝down时调小可多前进一点，up时相反",
            ),
            DeclareLaunchArgument(
                "workflow_file",
                default_value=PathJoinSubstitution(
                    [share, "config", IfElseSubstitution(LaunchConfiguration('round_trip'),
                        'line_follow_round_trip.yaml', IfElseSubstitution(
                        EqualsSubstitution(LaunchConfiguration("drive_direction"), "reverse"),
                        "line_follow_return.yaml", "line_follow_only.yaml"))]
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
                    "line_drive_direction": IfElseSubstitution(LaunchConfiguration('round_trip'),
                                                               'forward', LaunchConfiguration("drive_direction")),
                    "line_route_file": LaunchConfiguration("route_file"),
                    "line_route_visual_finish": LaunchConfiguration("route_visual_finish"),
                    "line_route_camera_stop": LaunchConfiguration("route_camera_stop"),
                    "line_reverse_mode": LaunchConfiguration("reverse_mode"),
                    "line_reverse_steering_sign": LaunchConfiguration("reverse_steering_sign"),
                    "line_follow_speed": LaunchConfiguration("line_follow_speed"),
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
