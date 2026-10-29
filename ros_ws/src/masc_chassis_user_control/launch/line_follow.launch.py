"""USB 蓝线循迹：相机 + 识别 + 底盘 + 循迹控制，不启动距离任务 PID。

启动循迹并显示实时画面：
    ros2 launch masc_chassis_user_control line_follow.launch.py show_window:=true forward_speed:=0.075
画面开关：show_window:=true 显示；show_window:=false 关闭（默认）。
该开关只控制可视化窗口；相机识别和底盘循迹继续运行。
行进方向：image_forward_direction:=down（轮子/车身在画面上方，前方朝下）。
识别反馈曝光：adaptive_exposure:=true（默认）；false 关闭曝光调节。
底部连接要求：require_bottom_connected:=true 强制贴到底边，默认 false。
暂停启动：enabled:=false；只观测输出：dry_run:=true（不启动底盘）。
不要同时运行 masc_tcp.launch.py 或其他发布 /cmd_vel 的控制器。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    control_share = get_package_share_directory('masc_chassis_user_control')
    camera_share = get_package_share_directory('ruier_camer_diver')
    driver_share = get_package_share_directory('masc_chassis_ros_driver')
    return LaunchDescription([
        DeclareLaunchArgument('forward_speed', default_value='0.075', description='前进速度，m/s'),
        DeclareLaunchArgument(
            'image_forward_direction', default_value='down',
            description='前方方向：down 向下（当前车身在上方），up 向上',
        ),
        DeclareLaunchArgument(
            'require_bottom_connected', default_value='false',
            description='是否强制胶带连接画面底边；默认允许有效可见中线',
        ),
        DeclareLaunchArgument('vehicle_reference_x_ratio', default_value='0.5',
                              description='虚拟车身中心横坐标 / 图宽'),
        DeclareLaunchArgument('vehicle_reference_y_ratio', default_value='-0.05',
                              description='虚拟车身纵坐标 / 图高，-0.05 位于画面上方'),
        DeclareLaunchArgument('steering_sign', default_value='-1.0',
                              description='左右标定符号；与前瞻方向共同决定实际转向，当前 down 配 -1.0'),
        DeclareLaunchArgument('adaptive_exposure', default_value='true',
                              description='结合识别质量和图像亮度自适应调节曝光'),
        DeclareLaunchArgument('exposure_device', default_value='',
                              description='曝光设备覆盖，空值沿用 camera_params 中的 video_device'),
        DeclareLaunchArgument('exposure_params', default_value=os.path.join(
            control_share, 'config', 'adaptive_exposure.yaml')),
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
        Node(
            package='masc_chassis_user_control', executable='line_exposure_node.py',
            name='line_exposure_controller', output='screen',
            condition=IfCondition(LaunchConfiguration('adaptive_exposure')),
            parameters=[LaunchConfiguration('camera_params'), LaunchConfiguration('exposure_params'), {
                'device_override': LaunchConfiguration('exposure_device'),
            }],
        ),
        # Keep the camera include from overriding the controller show_window.
        GroupAction(actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    camera_share, 'launch', 'line_tracking.launch.py')),
                condition=IfCondition(LaunchConfiguration('start_camera')),
                launch_arguments={
                    'namespace': 'camera', 'demo': 'false',
                    'show_window': 'false',
                    'camera_params': LaunchConfiguration('camera_params'),
                    'tracker_params': LaunchConfiguration('tracker_params'),
                }.items(),
            ),
        ], scoped=True),
        Node(
            package='masc_chassis_user_control', executable='line_follow_node.py',
            name='line_follow_controller', output='screen',
            condition=UnlessCondition(LaunchConfiguration('dry_run')),
            parameters=[LaunchConfiguration('control_params'), {
                'forward_speed': ParameterValue(LaunchConfiguration('forward_speed'), value_type=float),
                'image_forward_direction': LaunchConfiguration('image_forward_direction'),
                'steering_sign': ParameterValue(LaunchConfiguration('steering_sign'), value_type=float),
                'vehicle_reference_x_ratio': ParameterValue(
                    LaunchConfiguration('vehicle_reference_x_ratio'), value_type=float),
                'vehicle_reference_y_ratio': ParameterValue(
                    LaunchConfiguration('vehicle_reference_y_ratio'), value_type=float),
                'show_window': ParameterValue(LaunchConfiguration('show_window'), value_type=bool),
                'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
                'require_bottom_connected': ParameterValue(
                    LaunchConfiguration('require_bottom_connected'), value_type=bool),
            }],
        ),
        Node(
            package='masc_chassis_user_control', executable='line_follow_node.py',
            name='line_follow_controller', output='screen',
            condition=IfCondition(LaunchConfiguration('dry_run')),
            parameters=[LaunchConfiguration('control_params'), {
                'forward_speed': ParameterValue(LaunchConfiguration('forward_speed'), value_type=float),
                'image_forward_direction': LaunchConfiguration('image_forward_direction'),
                'steering_sign': ParameterValue(LaunchConfiguration('steering_sign'), value_type=float),
                'vehicle_reference_x_ratio': ParameterValue(
                    LaunchConfiguration('vehicle_reference_x_ratio'), value_type=float),
                'vehicle_reference_y_ratio': ParameterValue(
                    LaunchConfiguration('vehicle_reference_y_ratio'), value_type=float),
                'show_window': ParameterValue(LaunchConfiguration('show_window'), value_type=bool),
                'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
                'require_bottom_connected': ParameterValue(
                    LaunchConfiguration('require_bottom_connected'), value_type=bool),
                'dry_run': True,
                'cmd_vel_topic': '/line_follow/cmd_vel',
            }],
        ),
    ])
