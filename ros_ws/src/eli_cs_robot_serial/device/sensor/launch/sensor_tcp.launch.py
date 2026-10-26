import os
import json
from pathlib import Path
from tempfile import NamedTemporaryFile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_node(context):
    overrides = {
        "transport": "tcp",
        "host": LaunchConfiguration("host"),
        "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
        "frame_id": LaunchConfiguration("frame_id"),
    }
    parameters = [LaunchConfiguration("config_file"), overrides]
    cleanup = []
    target = LaunchConfiguration("target_unit").perform(context).strip()
    if target:
        if target not in ("N", "kg", "kgf", "keep"):
            raise ValueError("target_unit 必须为 N、kgf 或 keep")
        # ROS 的 YAML 解析器可能把未加引号的 N 当成布尔值，显式写带引号字符串。
        with NamedTemporaryFile(mode="w", prefix="sensor_unit_", suffix=".yaml", delete=False) as output:
            json.dump({"sensor_node": {"ros__parameters": {"target_unit": target}}}, output)
            path = output.name
        parameters.append(path)

        def remove_file(_context):
            Path(path).unlink(missing_ok=True)
            return []

        cleanup.append(RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(function=remove_file)]
        )))
    return [*cleanup, Node(
        package="eli_cs_robot_serial", executable="sensor_node", name="sensor_node",
        output="screen", parameters=parameters,
    )]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("eli_cs_robot_serial"),
        "device", "sensor", "config", "sensor_tcp.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("host", default_value="169.254.199.80"),
        DeclareLaunchArgument("port", default_value="9001"),
        DeclareLaunchArgument("frame_id", default_value="force_sensor"),
        DeclareLaunchArgument(
            "target_unit", default_value="",
            description="Hardware unit after verification: N, kgf, keep; empty uses YAML.",
        ),
        OpaqueFunction(function=launch_node),
    ])
