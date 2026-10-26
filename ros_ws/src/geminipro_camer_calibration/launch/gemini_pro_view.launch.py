"""一键打开 Gemini Pro RGB-D 驱动与可视化，不启动机器人。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [
            FindPackageShare("geminipro_camer_calibration"),
            "config",
            "gemini_pro_capture.yaml",
        ]
    )
    defaults = {
        "start_camera": "true",
        "show_window": "true",
        "camera_params": config,
        "output_dir": "~/geminipro_captures",
        "camera_namespace": "camera",
        "serial_number": "",
    }
    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]
    viewer = Node(
        package="geminipro_camer_calibration",
        executable="rgbd_view_node.py",
        name="geminipro_calibration",
        output="screen",
        parameters=[
            {
                "show_window": ParameterValue(
                    LaunchConfiguration("show_window"), value_type=bool
                ),
                "output_dir": ParameterValue(
                    LaunchConfiguration("output_dir"), value_type=str
                ),
                "camera_namespace": ParameterValue(
                    LaunchConfiguration("camera_namespace"), value_type=str
                ),
            }
        ],
    )
    return LaunchDescription(
        [
            *declarations,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=viewer,
                    on_exit=[EmitEvent(event=Shutdown(reason="RGB-D viewer closed"))],
                )
            ),
            Node(
                package="orbbec_camera",
                executable="orbbec_camera_node",
                name="orbbec_camera_node",
                namespace=LaunchConfiguration("camera_namespace"),
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_camera")),
                parameters=[
                    LaunchConfiguration("camera_params"),
                    {
                        "serial_number": ParameterValue(
                            LaunchConfiguration("serial_number"), value_type=str
                        )
                    },
                ],
            ),
            viewer,
        ]
    )
