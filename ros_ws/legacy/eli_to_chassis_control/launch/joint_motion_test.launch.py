import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    communication = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("eli_cs_robot_serial"),
                "launch",
                "computer_pi.launch.py",
            ])
        ),
        condition=IfCondition(LaunchConfiguration("start_communication")),
        launch_arguments={
            "enable_status": "true",
            "enable_camera": LaunchConfiguration("start_camera"),
            "show_image": LaunchConfiguration("show_image"),
        }.items(),
    )

    robot_motion_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("eli_cs_robot_usr"),
                "launch",
                "compliant_control_real.launch.py",
            ])
        ),
        condition=IfCondition(LaunchConfiguration("start_moveit")),
        launch_arguments={
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "interactive_input": "false",
            "launch_target_publisher": "false",
            # 工作流直接使用导纳控制器内的 MoveIt IK，不启动独立轨迹执行器。
            "launch_executor": "false",
            "admittance_enabled_on_startup": "false",
            "launch_coordinator": "false",
            "launch_sensor": LaunchConfiguration("launch_force_sensor"),
            "sensor_host": LaunchConfiguration("sensor_host"),
            "sensor_port": LaunchConfiguration("sensor_port"),
            "sensor_roll": LaunchConfiguration("sensor_roll"),
            "sensor_pitch": LaunchConfiguration("sensor_pitch"),
            "sensor_yaw": LaunchConfiguration("sensor_yaw"),
        }.items(),
    )

    workflow_controller = Node(
        package="eli_to_chassis_control",
        # 直接执行安装的脚本，避免依赖本包的 PYTHONPATH 和入口元数据。
        executable=sys.executable,
        arguments=[PathJoinSubstitution([
            FindPackageShare("eli_to_chassis_control"),
            "scripts", "workflow_controller.py",
        ])],
        name="workflow_controller",
        output="screen",
        parameters=[{
            "workflow_file": LaunchConfiguration("workflow_file"),
            "auto_start": ParameterValue(
                LaunchConfiguration("auto_start"), value_type=bool
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("start_communication", default_value="true"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("show_image", default_value="true"),
        DeclareLaunchArgument("start_moveit", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("launch_force_sensor", default_value="true"),
        DeclareLaunchArgument("sensor_host", default_value="169.254.199.80"),
        DeclareLaunchArgument("sensor_port", default_value="9001"),
        # force_sensor 在 tool0 中的安装角度，单位为弧度。
        DeclareLaunchArgument("sensor_roll", default_value="0.0"),
        DeclareLaunchArgument("sensor_pitch", default_value="0.0"),
        DeclareLaunchArgument("sensor_yaw", default_value="1.5707963267948966"),
        DeclareLaunchArgument(
            "workflow_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("eli_to_chassis_control"),
                "config",
                "workflow.yaml",
            ]),
        ),
        communication,
        robot_motion_system,
        workflow_controller,
    ])
