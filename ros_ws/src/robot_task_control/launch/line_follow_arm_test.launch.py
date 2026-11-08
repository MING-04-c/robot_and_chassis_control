"""循迹各站点联动：行驶关节姿态、中转、机械臂点位、等待下一次开始。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, IfElseSubstitution, LaunchConfiguration, NotSubstitution, OrSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = FindPackageShare("robot_task_control")
    return LaunchDescription([
        DeclareLaunchArgument('ros_discovery_range', default_value='LOCALHOST',
                             choices=['LOCALHOST', 'SUBNET', 'SYSTEM_DEFAULT', 'OFF'],
                             description='ROS发现范围；默认仅本机，TCP工控机与机械臂网口连接保留'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', LaunchConfiguration('ros_discovery_range')),
        DeclareLaunchArgument('test_profile', default_value='stations', choices=['stations', 'return_charge'],
                             description='stations逐站四点；return_charge首条有效横线到站后中转等待5秒并返程充电'),
        DeclareLaunchArgument('charging_status_layout', default_value='byte_flags', choices=['bitfield32', 'byte_flags']),
        DeclareLaunchArgument('enable_status_monitor', default_value='true', choices=['true', 'false'],
                             description='启用额外只读状态汇总；与TCP通信、自动开始独立'),
        DeclareLaunchArgument("remote_control", default_value="false", choices=["true", "false"],
                             description="true: 加载TCP并等待主机开始；false: 就绪后自动执行"),
        DeclareLaunchArgument('start_tcp', default_value=OrSubstitution(
            LaunchConfiguration('remote_control'), LaunchConfiguration('enable_status_monitor')),
            choices=['true', 'false'], description='启动9010 TCP桥；关闭状态汇总后仍可单独开启'),
        DeclareLaunchArgument('line_follow_speed', default_value=IfElseSubstitution(
            EqualsSubstitution(LaunchConfiguration('test_profile'), 'return_charge'), '0.03', '0.075')),
        DeclareLaunchArgument('route_file', default_value='~/.ros/line_follow/last_route.json'),
        DeclareLaunchArgument('reverse_route_max_curvature', default_value='5.0'),
        DeclareLaunchArgument("auto_start", default_value=NotSubstitution(
            LaunchConfiguration("remote_control"))),
        DeclareLaunchArgument("key_file", default_value=""),
        DeclareLaunchArgument("launch_admittance", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="false",
                             description="树莓派默认关闭RViz；循迹画面由line_show_window单独控制"),
        DeclareLaunchArgument("prepare_line_camera_early", default_value="false",
                             description="默认先等待机械臂控制器就绪，再启动循迹图像处理，降低启动争抢"),
        DeclareLaunchArgument('line_processing_fps', default_value='8.0',
                             description='联测默认限制视觉处理频率；相机采集仍为15帧'),
        DeclareLaunchArgument('line_opencv_threads', default_value='2',
                             description='主线图像处理的OpenCV线程数，不改变控制执行器线程数'),
        DeclareLaunchArgument('line_marker_opencv_threads', default_value='1',
                             description='横线检测的OpenCV线程数，与主线图像处理独立'),
        DeclareLaunchArgument('launch_force_sensor', default_value=IfElseSubstitution(
            EqualsSubstitution(LaunchConfiguration('test_profile'), 'return_charge'), 'false', 'true')),
        DeclareLaunchArgument("workflow_file", default_value=PathJoinSubstitution(
            [share, "config", IfElseSubstitution(
                EqualsSubstitution(LaunchConfiguration('test_profile'), 'return_charge'),
                'line_follow_arm_charge_test.yaml', 'line_follow_arm_test.yaml')])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("eli_cs_robot_serial"), "launch", "industrial_pc.launch.py",
            ])),
            condition=IfCondition(OrSubstitution(LaunchConfiguration('remote_control'),
                                                LaunchConfiguration('start_tcp'))),
            launch_arguments={"key_file": LaunchConfiguration("key_file"),
                              "enable_status_monitor": LaunchConfiguration('enable_status_monitor'),
                              "start_hk_transfer": "false",
                              "charging_status_layout": LaunchConfiguration('charging_status_layout')}.items(),
        ),
        Node(package='robot_task_control', executable='arm_joint_postures',
             name='arm_joint_postures', output='screen', parameters=[{
                 'simulation': False, 'manage_admittance': ParameterValue(LaunchConfiguration('launch_admittance'), value_type=bool),
                 'velocity_scaling': .1, 'startup_timeout': 480.,
                 'report_file': '~/.ros/robot_control/joint_postures.json',
             }]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [share, "launch", "internal", "robot_system.launch.py"])),
            launch_arguments={
                "ros_discovery_range": LaunchConfiguration('ros_discovery_range'),
                "workflow_file": LaunchConfiguration("workflow_file"),
                "launch_admittance": LaunchConfiguration("launch_admittance"),
                "auto_start": LaunchConfiguration("auto_start"),
                "require_arm_ready": "true", "check_devices_on_startup": "true",
                "joint_posture_only": EqualsSubstitution(LaunchConfiguration("test_profile"), "return_charge"),
                "manage_line_stream": EqualsSubstitution(LaunchConfiguration("test_profile"), "return_charge"),
                "chassis_pid_enabled": "false", "use_velocity_mux": "true",
                "line_follow_speed": LaunchConfiguration('line_follow_speed'),
                "line_route_file": LaunchConfiguration('route_file'),
                "line_reverse_route_max_curvature": LaunchConfiguration('reverse_route_max_curvature'),
                "launch_arm_executor": IfElseSubstitution(
                    EqualsSubstitution(LaunchConfiguration('test_profile'), 'return_charge'), 'false', 'true'),
                "arm_use_cartesian_path": "false",
                'line_charging_mode': EqualsSubstitution(LaunchConfiguration('test_profile'), 'return_charge'),
                'line_charging_status_layout': LaunchConfiguration('charging_status_layout'),
                "start_hk_camera": "false", "start_image_transfer": "false",
                "start_line_camera": "true",
                "prepare_line_camera_early": LaunchConfiguration("prepare_line_camera_early"),
                "line_processing_fps": LaunchConfiguration('line_processing_fps'),
                "line_opencv_threads": LaunchConfiguration('line_opencv_threads'),
                "line_marker_opencv_threads": LaunchConfiguration('line_marker_opencv_threads'),
                "launch_rviz": LaunchConfiguration("launch_rviz"),
                'launch_force_sensor': LaunchConfiguration('launch_force_sensor'),
            }.items(),
        ),
    ])
