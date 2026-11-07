"""同一探针测试：默认接收主机检测点，可选本地 YAML 点位。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    share = FindPackageShare('robot_task_control')
    profile = LaunchConfiguration('task_config')
    driver_arguments = ('robot_ip', 'local_ip', 'cs_type', 'headless_mode',
                        'start_arm_driver', 'start_arm_control', 'launch_force_sensor',
                        'start_thickness_detector', 'launch_rviz', 'arm_control_start_delay')
    defaults = ('169.254.199.150', '0.0.0.0', 'ls65', 'true', 'true', 'true',
                'true', 'true', 'true', '5.0')
    return LaunchDescription([
        SetLaunchConfiguration('sigterm_timeout', '3.0'),
        SetLaunchConfiguration('sigkill_timeout', '2.0'),
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in zip(driver_arguments, defaults)],
        DeclareLaunchArgument('task_profile', default_value='camera_detection_real',
                             choices=['camera_detection_real', 'camera_probe_touch'],
                             description='Select the detection task YAML; task_config can override its path'),
        DeclareLaunchArgument('touch_control_mode', default_value='moveit_force_guard',
            choices=['admittance', 'moveit_force_guard']),
        DeclareLaunchArgument('target_source', default_value=IfElseSubstitution(
            EqualsSubstitution(LaunchConfiguration('task_profile'), 'camera_probe_touch'), 'local', 'tcp'),
            choices=['tcp', 'local']),
        DeclareLaunchArgument('targets_file', default_value=IfElseSubstitution(
            EqualsSubstitution(LaunchConfiguration('task_profile'), 'camera_probe_touch'),
            '~/.ros/detect_control/camera_probe_touch_targets.yaml',
            '~/.ros/detect_control/camera_detection_targets.yaml')),
        DeclareLaunchArgument('capture_config_file', default_value=PathJoinSubstitution([
            share, 'config', 'capture', 'final_capture.yaml'])),
        DeclareLaunchArgument('task_config', default_value=PathJoinSubstitution([
            share, 'config', [LaunchConfiguration('task_profile'), '.yaml']])),
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, 'launch', 'internal', 'detection_system.launch.py'])),
            launch_arguments={
                **{name: LaunchConfiguration(name) for name in driver_arguments},
                'touch_control_mode': LaunchConfiguration('touch_control_mode'),
                'simulation': 'false', 'start_test_task': 'false', 'test_config': profile,
                'targets_file': LaunchConfiguration('targets_file'),
            }.items())], scoped=True),
        Node(package='eli_cs_robot_serial', executable='industrial_pc_node.py', name='industrial_pc',
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration('target_source'), 'tcp')),
             output='screen', parameters=[
                 PathJoinSubstitution([FindPackageShare('eli_cs_robot_serial'), 'device',
                                       'industrial_pc', 'config', 'industrial_pc.yaml']),
                 {'camera_optical_frame': 'terminal_link_1'},
             ], sigterm_timeout='3.0', sigkill_timeout='2.0'),
        Node(package='robot_task_control', executable='camera_detection_task',
             name='camera_detection_task', output='screen', parameters=[profile, {
                 'target_source': LaunchConfiguration('target_source'),
                 'capture_config_file': LaunchConfiguration('capture_config_file'),
             }],
             sigterm_timeout='3.0', sigkill_timeout='2.0'),
    ])
