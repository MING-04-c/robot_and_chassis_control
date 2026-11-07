"""双关节姿态往返：默认假硬件；simulation:=false显式启用真机。"""
import os
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from eli_cs_robot_moveit_config.joint_postures import load_postures, JOINTS
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, IfElseSubstitution, EqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    postures_file = LaunchConfiguration('postures_file').perform(context)
    simulation = LaunchConfiguration('simulation').perform(context).lower() == 'true'
    actions = []
    driver = {'cs_type': 'ls65', 'launch_rviz': 'false', 'activate_joint_controller': 'true',
              'use_fake_hardware': str(simulation).lower(),
              'initial_joint_controller': 'joint_trajectory_controller' if simulation else 'scaled_joint_trajectory_controller',
              'robot_ip': '127.0.0.1' if simulation else LaunchConfiguration('robot_ip').perform(context),
              'local_ip': LaunchConfiguration('local_ip').perform(context), 'headless_mode': 'true'}
    if simulation:
        travel = load_postures(postures_file)['travel']
        with tempfile.NamedTemporaryFile('w', suffix='.yaml', prefix='joint-posture-initial-', delete=False) as file:
            yaml.safe_dump(dict(zip(JOINTS, travel)), file)
            initial = file.name
        driver['initial_positions_file'] = initial
        actions.extend([
            SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('simulation_domain')),
            SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
                function=lambda _: Path(initial).unlink(missing_ok=True) or [])])),
        ])
    def include(package, filename, arguments):
        return GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', filename])), launch_arguments=arguments.items())], scoped=True)
    return actions + [
        GroupAction([include('eli_cs_robot_driver', 'elite_control.launch.py', driver)],
                    condition=IfCondition(LaunchConfiguration('start_arm_driver'))),
        GroupAction([include('eli_cs_robot_moveit_config', 'cs_moveit.launch.py', {
            'cs_type': 'ls65', 'use_fake_hardware': str(simulation).lower(), 'launch_rviz': 'false',
            'publish_robot_description': 'false', 'launch_servo': 'false'})],
                    condition=IfCondition(LaunchConfiguration('start_moveit'))),
        Node(package='rviz2', executable='rviz2', output='log',
             condition=IfCondition(LaunchConfiguration('launch_rviz')),
             arguments=['-d', PathJoinSubstitution([FindPackageShare('eli_cs_robot_moveit_config'), 'rviz', 'view_robot.rviz'])]),
        Node(package='robot_task_control', executable='joint_posture_test', output='screen', parameters=[{
            'simulation': simulation,
            'velocity_scaling': ParameterValue(LaunchConfiguration('velocity_scaling'), value_type=float),
            'startup_timeout': 90.0 if simulation else 480.0,
            'simulation_domain': ParameterValue(LaunchConfiguration('simulation_domain'), value_type=int),
            'postures_file': postures_file, 'report_file': LaunchConfiguration('report_file')}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('robot_ip', default_value='169.254.199.150'),
        DeclareLaunchArgument('local_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('start_arm_driver', default_value='true'),
        DeclareLaunchArgument('start_moveit', default_value='true'),
        DeclareLaunchArgument('velocity_scaling', default_value='0.10'),
        DeclareLaunchArgument('simulation_domain', default_value='190'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('report_file', default_value=IfElseSubstitution(EqualsSubstitution(LaunchConfiguration('simulation'), 'true'),
            '/tmp/joint_posture_test_report.json', '/tmp/joint_posture_real_report.json')),
        DeclareLaunchArgument('postures_file', default_value=str(Path(get_package_share_directory(
            'eli_cs_robot_moveit_config'))/'config/arm_joint_postures.yaml')),
        OpaqueFunction(function=setup),
    ])
