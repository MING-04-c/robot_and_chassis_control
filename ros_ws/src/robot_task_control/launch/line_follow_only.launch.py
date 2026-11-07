"""Launch the selected control profile without unrelated devices."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, IfElseSubstitution, LaunchConfiguration, NotSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("robot_task_control")
    return LaunchDescription(
        [
            DeclareLaunchArgument('navigation_mode', default_value='vision', choices=['vision', 'teach', 'replay'],
                                 description='vision原有循迹；teach低速记录标准；replay轨迹主导、视觉辅助'),
            DeclareLaunchArgument('reverse_route_lookahead', default_value='0.26',
                description='记录路线倒车前瞻距离m；加大减轻急纠偏，过大会切弯'),
            DeclareLaunchArgument('line_reverse_speed', default_value='0.05',
                description='倒车巡航速度上限m/s；与前进速度独立设置'),
            DeclareLaunchArgument('reverse_route_max_curvature', default_value='5.0',
                description='途中倒车跟踪曲率上限1/m；独立于末段充电限制，范围0.2到5'),
            DeclareLaunchArgument('reverse_finish_heading_gain', default_value='0.60',
                description='倒车末段摆正力度；前进转向不使用此参数'),
            DeclareLaunchArgument('start_alignment', default_value='true', choices=['true', 'false'],
                description='启动循迹或标准重放前，先用相机多帧对齐起步横线'),
            DeclareLaunchArgument('start_alignment_min_lateral_speed', default_value='0.02',
                description='起步横移最低目标速度m/s；独立于巡航速度，需实车校准'),
            DeclareLaunchArgument('start_alignment_lateral_speed', default_value='0.03',
                description='起步横移速度上限m/s'),
            DeclareLaunchArgument('route_name', default_value='route_01'),
            DeclareLaunchArgument('standard_route_file', default_value=PathJoinSubstitution(
                ['~/.ros/line_follow/standards', LaunchConfiguration('route_name'), 'standard_route.json'])),
            DeclareLaunchArgument('line_processing_fps', default_value=IfElseSubstitution(
                EqualsSubstitution(LaunchConfiguration('navigation_mode'), 'replay'), '8.0', '0.0')),
            DeclareLaunchArgument('round_trip', default_value=NotSubstitution(
                EqualsSubstitution(LaunchConfiguration('navigation_mode'), 'vision')),
                                 description='自动往返；配合charging_mode=true时首条有效横线停5秒后返程充电'),
            DeclareLaunchArgument("drive_direction", default_value="forward", choices=["forward", "reverse"],
                                 description="同一前置相机，forward前进或reverse低速后退"),
            DeclareLaunchArgument('line_record_speed', default_value='0.03',
                description='teach专门录制速度m/s，独立于正常运行速度'),
            DeclareLaunchArgument('line_follow_speed', default_value='0.1',
                description='vision/replay正常前进巡航速度m/s，不改变teach录制速度'),
            DeclareLaunchArgument('charging_mode', default_value=NotSubstitution(
                EqualsSubstitution(LaunchConfiguration('navigation_mode'), 'vision')), choices=['true', 'false']),
            DeclareLaunchArgument('charging_status_layout', default_value='byte_flags', choices=['bitfield32', 'byte_flags']),
            DeclareLaunchArgument("route_visual_finish", default_value="true"),
            DeclareLaunchArgument("route_camera_stop", default_value="true"),
            DeclareLaunchArgument("route_file", default_value="~/.ros/line_follow/last_route.json"),
            DeclareLaunchArgument("reverse_mode", default_value="recorded_route", choices=["recorded_route", "camera"]),
            DeclareLaunchArgument("reverse_steering_sign", default_value="-1.0",
                                 description="仅倒车转向修正；-1翻转，+1标准映射"),
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument('check_devices_on_startup', default_value=NotSubstitution(
                EqualsSubstitution(LaunchConfiguration('navigation_mode'), 'vision'))),
            DeclareLaunchArgument(
                "line_stop_marker_target_ratio",
                default_value=IfElseSubstitution(EqualsSubstitution(LaunchConfiguration('navigation_mode'), 'vision'), '0.46', '0.5'),
                description="停车横线纵坐标/图高；前方朝down时调小可多前进一点，up时相反",
            ),
            DeclareLaunchArgument(
                "workflow_file",
                default_value=PathJoinSubstitution(
                    [share, "config", IfElseSubstitution(LaunchConfiguration('round_trip'),
                        IfElseSubstitution(LaunchConfiguration('charging_mode'),
                            'line_follow_charge_test.yaml', 'line_follow_round_trip.yaml'), IfElseSubstitution(
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
                    "check_devices_on_startup": LaunchConfiguration("check_devices_on_startup"),
                    "line_navigation_mode": IfElseSubstitution(
                        EqualsSubstitution(LaunchConfiguration('navigation_mode'), 'replay'), 'replay', 'vision'),
                    "standard_route_file": LaunchConfiguration("standard_route_file"),
                    "standard_route_output": IfElseSubstitution(
                        EqualsSubstitution(LaunchConfiguration('navigation_mode'), 'teach'),
                        LaunchConfiguration('standard_route_file'), ''),
                    "line_route_sample_distance": IfElseSubstitution(
                        EqualsSubstitution(LaunchConfiguration('navigation_mode'), 'teach'), '0.01', '0.02'),
                    "line_processing_fps": LaunchConfiguration('line_processing_fps'),
                    "line_drive_direction": IfElseSubstitution(LaunchConfiguration('round_trip'),
                                                               'forward', LaunchConfiguration("drive_direction")),
                    'line_charging_mode': LaunchConfiguration('charging_mode'),
                    'line_charging_status_layout': LaunchConfiguration('charging_status_layout'),
                    "line_route_file": LaunchConfiguration("route_file"),
                    "line_route_visual_finish": LaunchConfiguration("route_visual_finish"),
                    "line_route_camera_stop": LaunchConfiguration("route_camera_stop"),
                    "line_reverse_mode": LaunchConfiguration("reverse_mode"),
                    "line_reverse_steering_sign": LaunchConfiguration("reverse_steering_sign"),
                    "line_follow_speed": IfElseSubstitution(
                        EqualsSubstitution(LaunchConfiguration('navigation_mode'), 'teach'),
                        LaunchConfiguration('line_record_speed'), LaunchConfiguration('line_follow_speed')),
                    "line_reverse_speed": LaunchConfiguration("line_reverse_speed"),
                    "line_reverse_route_lookahead": LaunchConfiguration("reverse_route_lookahead"),
                    "line_reverse_route_max_curvature": LaunchConfiguration("reverse_route_max_curvature"),
                    "line_reverse_finish_heading_gain": LaunchConfiguration("reverse_finish_heading_gain"),
                    "line_start_alignment_enabled": LaunchConfiguration("start_alignment"),
                    "line_start_alignment_min_lateral_speed": LaunchConfiguration("start_alignment_min_lateral_speed"),
                    "line_start_alignment_lateral_speed": LaunchConfiguration("start_alignment_lateral_speed"),
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
