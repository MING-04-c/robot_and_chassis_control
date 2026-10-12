"""MASC 总启动：默认只启动底盘驱动、PID 和 9001 状态/控制 TCP。

不启动摄像头（默认）：
    ros2 launch masc_user_serial masc_tcp.launch.py

需要摄像头和 9002 图像 TCP 时显式开启：
    ros2 launch masc_user_serial masc_tcp.launch.py \
        start_hik_camera:=true image_stream:=true

启动前请先 source 工作空间的 install/setup.bash。can0 由
masc-chassis-can.service 在开机时配置。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_chassis_driver', default_value='true',
                              description='同时启动 MASC 底盘 ROS 驱动'),
        DeclareLaunchArgument('chassis_enable_pid', default_value='true',
                              description='启动底盘内置PID和方案1的10厘米控制'),
        DeclareLaunchArgument('start_hik_camera', default_value='false',
                              description='启动海康相机驱动；默认不启动摄像头'),
        DeclareLaunchArgument('status_stream', default_value='true',
                              description='启动 9001 状态 TCP 服务'),
        DeclareLaunchArgument('status_listen_ip', default_value='169.254.199.100',
                              description='状态 TCP 本机网线 IPv4 地址'),
        DeclareLaunchArgument('status_allowed_client_ip', default_value='169.254.199.250',
                              description='允许连接状态端口的上位机 IPv4 地址'),
        DeclareLaunchArgument('status_port', default_value='9001'),
        DeclareLaunchArgument('image_stream', default_value='false',
                              description='启动 9002 图像 TCP 服务；默认不启动'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('chassis_status_topic',
                              default_value='/masc_chassis/chassis_status'),
        DeclareLaunchArgument('chassis_control_topic',
                              default_value='/chassis/control_command'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('test_start_topic',
                              default_value='/chassis/task_start'),
        DeclareLaunchArgument('target_reached_topic',
                              default_value='/target_reached'),
        DeclareLaunchArgument('peer_timeout_sec', default_value='3.0'),
        DeclareLaunchArgument('image_listen_ip', default_value='169.254.199.100',
                              description='本机网线 IPv4 地址'),
        DeclareLaunchArgument('image_allowed_client_ip', default_value='169.254.199.250',
                              description='允许接收图像的对端 IPv4 地址'),
        DeclareLaunchArgument('image_port', default_value='9002'),
        DeclareLaunchArgument('image_format', default_value='jpeg',
                              description='图像传输模式: jpeg/raw'),
        DeclareLaunchArgument('image_jpeg_quality', default_value='80'),
        DeclareLaunchArgument('image_max_fps', default_value='60.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('masc_chassis_ros_driver'),
                    'launch',
                    'masc_chassis_ros_driver.launch.py',
                ])
            ),
            condition=IfCondition(LaunchConfiguration('start_chassis_driver')),
            launch_arguments={
                'enable_pid': LaunchConfiguration('chassis_enable_pid'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'test_start_topic': LaunchConfiguration('test_start_topic'),
                'control_topic': LaunchConfiguration('chassis_control_topic'),
                'task_status_topic': '/chassis_arrival_status',
                'target_reached_topic': LaunchConfiguration('target_reached_topic'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('hik_camera_ros2_driver'),
                    'launch',
                    'hik_camera_launch.py',
                ])
            ),
            condition=IfCondition(LaunchConfiguration('start_hik_camera')),
            launch_arguments={
                'show_image': 'false',
                'image_topic': LaunchConfiguration('image_topic'),
            }.items(),
        ),
        Node(package='masc_user_serial',
             executable='status_tcp_node.py',
             name='status_tcp_node',
             output='screen',
             condition=IfCondition(LaunchConfiguration('status_stream')),
             parameters=[
                 PathJoinSubstitution([
                     FindPackageShare('masc_chassis_ros_driver'),
                     'config', 'ros2', 'task_plans.yaml',
                 ]),
                 {
                 'listen_ip': LaunchConfiguration('status_listen_ip'),
                 'allowed_client_ip': LaunchConfiguration('status_allowed_client_ip'),
                 'listen_port': ParameterValue(LaunchConfiguration('status_port'), value_type=int),
                 'image_topic': LaunchConfiguration('image_topic'),
                 'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                 'image_port': ParameterValue(LaunchConfiguration('image_port'), value_type=int),
                 'image_format': LaunchConfiguration('image_format'),
                 'jpeg_quality': ParameterValue(LaunchConfiguration('image_jpeg_quality'), value_type=int),
                 'image_max_fps': ParameterValue(LaunchConfiguration('image_max_fps'), value_type=float),
                 'chassis_topic': LaunchConfiguration('chassis_status_topic'),
                 'control_topic': LaunchConfiguration('chassis_control_topic'),
                 'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                 'test_start_topic': LaunchConfiguration('test_start_topic'),
                 'target_reached_topic': LaunchConfiguration('target_reached_topic'),
                 'peer_timeout_sec': ParameterValue(
                     LaunchConfiguration('peer_timeout_sec'), value_type=float),
                 },
             ]),
        Node(package='masc_user_serial',
             executable='camera_image_tcp_node.py',
             name='camera_image_tcp_node',
             output='screen',
             condition=IfCondition(LaunchConfiguration('image_stream')),
             parameters=[{
                 'image_topic': LaunchConfiguration('image_topic'),
                 'listen_ip': LaunchConfiguration('image_listen_ip'),
                 'allowed_client_ip': LaunchConfiguration('image_allowed_client_ip'),
                 'listen_port': ParameterValue(LaunchConfiguration('image_port'), value_type=int),
                 'max_fps': ParameterValue(LaunchConfiguration('image_max_fps'), value_type=float),
                 'image_format': LaunchConfiguration('image_format'),
                 'jpeg_quality': ParameterValue(LaunchConfiguration('image_jpeg_quality'), value_type=int),
             }]),
    ])
