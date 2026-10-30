"""固定假硬件和独立ROS域，自动回放像素检测点；没有真机入口。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('robot_task_control')
    def include(package, filename, args):
        return GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', filename])), launch_arguments=args.items())], scoped=True)
    return LaunchDescription([
        DeclareLaunchArgument('simulation_domain', default_value='191'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('repeat', default_value='false'),
        DeclareLaunchArgument('points_file', default_value=PathJoinSubstitution([
            share, 'config', 'tests', 'pixel_target_sample.json'])),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('simulation_domain')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        include('eli_cs_robot_driver', 'elite_control.launch.py', {
            'cs_type': 'ls65', 'robot_ip': '127.0.0.1', 'use_fake_hardware': 'true',
            'launch_rviz': 'false', 'initial_joint_controller': 'joint_trajectory_controller',
            'activate_joint_controller': 'true'}),
        include('eli_cs_robot_moveit_config', 'cs_moveit.launch.py', {
            'cs_type': 'ls65', 'use_fake_hardware': 'true', 'launch_rviz': 'false',
            'launch_servo': 'false', 'publish_robot_description': 'false'}),
        Node(package='eli_cs_robot_usr', executable='admittance_pose_controller',
             name='admittance_controller', output='screen', parameters=[
                 PathJoinSubstitution([FindPackageShare('eli_cs_robot_usr'), 'config', 'compliant_real.yaml']),
                 PathJoinSubstitution([FindPackageShare('eli_cs_robot_moveit_config'), 'config', 'kinematics.yaml']),
                 {'enabled_on_startup': False, 'force_source': 'topic', 'require_speed_scaling': False,
                  'joint_command_topic': '/joint_trajectory_controller/joint_trajectory'},
             ]),
        Node(package='detect_control', executable='detect_controller.py', name='detect_controller',
             output='screen', parameters=[
                 PathJoinSubstitution([FindPackageShare('detect_control'), 'config', 'detect.yaml']),
                 PathJoinSubstitution([share, 'config', 'tests', 'pixel_point_simulation.yaml']),
             ]),
        Node(package='rviz2', executable='rviz2', name='pixel_point_rviz',
             condition=IfCondition(LaunchConfiguration('launch_rviz')),
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'pixel_point_simulation.rviz'])], output='log'),
        Node(package='robot_task_control', executable='pixel_point_simulation', output='screen', parameters=[
            PathJoinSubstitution([share, 'config', 'tests', 'pixel_point_simulation.yaml']), {
            'points_file': LaunchConfiguration('points_file'),
            'simulation_domain': ParameterValue(LaunchConfiguration('simulation_domain'), value_type=int),
            'repeat': ParameterValue(LaunchConfiguration('repeat'), value_type=bool),
        }]),
    ])
