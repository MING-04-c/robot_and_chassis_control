"""USB 蓝线循迹：相机 + 识别 + 底盘 + 循迹控制，不启动距离任务 PID。

启动循迹并显示实时画面：
    ros2 launch masc_chassis_user_control line_follow.launch.py show_window:=true forward_speed:=0.05
画面开关：show_window:=true 显示；show_window:=false 关闭（默认）。
该开关只控制可视化窗口；相机识别和底盘循迹继续运行。
暂停启动：enabled:=false；只观测输出：dry_run:=true（不启动底盘）。
不要同时运行 masc_tcp.launch.py 或其他发布 /cmd_vel 的控制器。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    control_share = get_package_share_directory('masc_chassis_user_control')
    camera_share = get_package_share_directory('camer_diver')
    driver_share = get_package_share_directory('masc_chassis_ros_driver')
    return LaunchDescription([
        DeclareLaunchArgument('forward_speed', default_value='0.05', description='前进速度，m/s'),
        DeclareLaunchArgument('enabled', default_value='true', description='识别到有效蓝线后循迹'),
        DeclareLaunchArgument('dry_run', default_value='false', description='仅输出到 /line_follow/cmd_vel'),
        DeclareLaunchArgument('start_camera', default_value='true', description='启动相机和识别节点'),
        # 与底盘循迹一起运行的画面开关：true 显示，false 关闭。
        DeclareLaunchArgument(
            'show_window', default_value='false',
            description='循迹时显示相机四格实时画面；true 开启，false 关闭（默认）',
        ),
        DeclareLaunchArgument('control_params', default_value=os.path.join(
            control_share, 'config', 'line_follow.yaml')),
        DeclareLaunchArgument('camera_params', default_value=os.path.join(
            camera_share, 'config', 'camera.yaml')),
        DeclareLaunchArgument('tracker_params', default_value=os.path.join(
            camera_share, 'config', 'line_tracker.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                driver_share, 'launch', 'masc_chassis_ros_driver.launch.py')),
            condition=UnlessCondition(LaunchConfiguration('dry_run')),
            launch_arguments={'enable_pid': 'false'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                camera_share, 'launch', 'line_tracking.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_camera')),
            launch_arguments={
                'namespace': 'camera', 'demo': 'false',
                'show_window': LaunchConfiguration('show_window'),
                'camera_params': LaunchConfiguration('camera_params'),
                'tracker_params': LaunchConfiguration('tracker_params'),
            }.items(),
        ),
        Node(
            package='masc_chassis_user_control', executable='line_follow_node.py',
            name='line_follow_controller', output='screen',
            condition=UnlessCondition(LaunchConfiguration('dry_run')),
            parameters=[LaunchConfiguration('control_params'), {
                'forward_speed': ParameterValue(LaunchConfiguration('forward_speed'), value_type=float),
                'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
            }],
        ),
        Node(
            package='masc_chassis_user_control', executable='line_follow_node.py',
            name='line_follow_controller', output='screen',
            condition=IfCondition(LaunchConfiguration('dry_run')),
            parameters=[LaunchConfiguration('control_params'), {
                'forward_speed': ParameterValue(LaunchConfiguration('forward_speed'), value_type=float),
                'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
                'dry_run': True,
                'cmd_vel_topic': '/line_follow/cmd_vel',
            }],
        ),
    ])
