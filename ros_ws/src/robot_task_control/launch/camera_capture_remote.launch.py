"""独立相机远程采集：只加载等待采集的控制器及TCP通讯，不启动运动驱动。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("robot_task_control")
    return LaunchDescription([
        DeclareLaunchArgument("key_file", default_value=""),
        DeclareLaunchArgument("capture_config_file", default_value=PathJoinSubstitution([
            share, "config", "capture", "final_capture.yaml",
        ])),
        Node(
            package="robot_task_control", executable="task_controller", name="task_controller",
            parameters=[{
                "workflow_file": PathJoinSubstitution([share, "config", "capture", "camera_capture_only.yaml"]),
                "capture_config_file": LaunchConfiguration("capture_config_file"),
                "auto_start": False, "use_velocity_mux": False,
                "check_devices_on_startup": False, "require_arm_ready": False,
            }], output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("eli_cs_robot_serial"), "launch", "industrial_pc.launch.py",
            ])),
            launch_arguments={"key_file": LaunchConfiguration("key_file"),
                              "start_hk_transfer": "false"}.items(),
        ),
    ])
