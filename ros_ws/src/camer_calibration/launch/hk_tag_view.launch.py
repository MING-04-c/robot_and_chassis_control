"""启动海康相机、AprilTag 测量面板及可选 RViz，不启动机械臂。"""

from pathlib import Path
import tempfile

import numpy as np
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def camera_info_url(filename, temporary_directories):
    """ROS 格式直接读取；离线标定格式转换为驱动可读取的临时 CameraInfo YAML。"""
    path = Path(filename).expanduser().resolve()
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if isinstance(data["camera_matrix"], dict):
        return path.as_uri()
    matrix = np.asarray(data["camera_matrix"], dtype=float).reshape(3, 3)
    distortion = np.asarray(data["distortion_coefficients"], dtype=float).ravel()
    projection = np.column_stack((matrix, np.zeros(3)))
    converted = {
        "image_width": data["image_width"],
        "image_height": data["image_height"],
        "camera_name": "camera",
        "distortion_model": "plumb_bob",
        "camera_matrix": {"rows": 3, "cols": 3, "data": matrix.ravel().tolist()},
        "distortion_coefficients": {
            "rows": 1,
            "cols": len(distortion),
            "data": distortion.tolist(),
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": np.eye(3).ravel().tolist(),
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": projection.ravel().tolist(),
        },
    }
    directory = tempfile.TemporaryDirectory(prefix="hk-camera-info-")
    temporary_directories.append(directory)
    output = Path(directory.name) / "camera_info.yaml"
    output.write_text(yaml.safe_dump(converted), encoding="utf-8")
    return output.as_uri()


def generate_launch_description():
    temporary_directories = []

    def cleanup(context):
        for directory in temporary_directories:
            directory.cleanup()

    share = FindPackageShare("camer_calibration")
    defaults = {
        "start_camera": "true",
        "show_window": "true",
        "use_rviz": "false",
        "image_topic": "/camera/image",
        "camera_frame": "camera_optical_frame",
        "calibration_file": PathJoinSubstitution([share, "config", "camera_info.yaml"]),
        "tag_family": "auto",
        "tag_id": "-1",
        "tag_size_mm": "0.0",
        "board_cols": "0",
        "board_rows": "0",
        "square_size_mm": "0.0",
        "output_dir": "~/camera_calibration",
        "process_rate_hz": "10.0",
        "image_timeout_sec": "0.5",
        "max_reprojection_error_px": "3.0",
        "camera_params": PathJoinSubstitution([share, "config", "camera_params.yaml"]),
    }
    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]
    parameters = {}
    for name in (
        "image_topic",
        "camera_frame",
        "calibration_file",
        "tag_family",
        "output_dir",
    ):
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=str)
    for name in (
        "tag_size_mm",
        "square_size_mm",
        "process_rate_hz",
        "image_timeout_sec",
        "max_reprojection_error_px",
    ):
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=float)
    for name in ("tag_id", "board_cols", "board_rows"):
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=int)
    parameters["show_window"] = ParameterValue(
        LaunchConfiguration("show_window"), value_type=bool
    )
    viewer = Node(
        package="camer_calibration",
        executable="tag_pose_node.py",
        name="camera_calibration",
        output="screen",
        parameters=[parameters],
    )

    def start_camera(context):
        # 复用原海康驱动，采集参数来自复制的 YAML；内参覆盖时同步 camera_info。
        overrides = {
            "camera_topic": ParameterValue(
                LaunchConfiguration("image_topic"), value_type=str
            ),
            "frame_id": ParameterValue(
                LaunchConfiguration("camera_frame"), value_type=str
            ),
        }
        calibration = LaunchConfiguration("calibration_file").perform(context).strip()
        if calibration:
            overrides["camera_info_url"] = camera_info_url(
                calibration, temporary_directories
            )
        return [
            Node(
                package="hik_camera_ros2_driver",
                executable="hik_camera_ros2_driver_node",
                name="hik_camera_ros2_driver",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_camera")),
                parameters=[LaunchConfiguration("camera_params"), overrides],
            )
        ]

    return LaunchDescription(
        [
            *declarations,
            RegisterEventHandler(
                OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])
            ),
            OpaqueFunction(function=start_camera),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=viewer,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="AprilTag viewer closed"))
                    ],
                )
            ),
            viewer,
            Node(
                package="rviz2",
                executable="rviz2",
                name="camera_calibration_rviz",
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                arguments=[
                    "-d",
                    PathJoinSubstitution([share, "config", "tag_view.rviz"]),
                    "-f",
                    LaunchConfiguration("camera_frame"),
                ],
                output="screen",
            ),
        ]
    )
