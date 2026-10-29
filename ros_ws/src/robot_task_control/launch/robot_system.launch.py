"""Start device drivers, wait for the arm connection, then load task control."""

import math

from launch import LaunchDescription
from launch.logging import get_logger

from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    chassis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("masc_chassis_ros_driver"),
                    "launch",
                    "masc_chassis_ros_driver.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_chassis")),
        launch_arguments={
            "driver_params": LaunchConfiguration("chassis_params"),
            "enable_pid": LaunchConfiguration("chassis_pid_enabled"),
            "pid_cmd_vel_topic": "/robot_control/pid/cmd_vel",
            "task_plans": LaunchConfiguration("task_plans"),
        }.items(),
    )
    arm_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("eli_cs_robot_driver"),
                    "launch",
                    "elite_control.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_arm_driver")),
        launch_arguments={
            "cs_type": LaunchConfiguration("cs_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "local_ip": LaunchConfiguration("local_ip"),
            "launch_rviz": "false",
        }.items(),
    )

    robot_motion_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("eli_cs_robot_usr"),
                    "launch",
                    "compliant_control_real.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_moveit")),
        launch_arguments={
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "interactive_input": "false",
            "launch_target_publisher": "false",
            # 导纳步骤使用 IK；MoveIt 步骤由独立执行器处理。
            "launch_executor": LaunchConfiguration("launch_arm_executor"),
            "use_cartesian_path": LaunchConfiguration("arm_use_cartesian_path"),
            "cs_type": LaunchConfiguration("cs_type"),
            "admittance_enabled_on_startup": "false",
            "auto_start_admittance": "false",
            "launch_coordinator": "false",
            "launch_sensor": LaunchConfiguration("launch_force_sensor"),
            "sensor_host": LaunchConfiguration("sensor_host"),
            "sensor_port": LaunchConfiguration("sensor_port"),
            "sensor_roll": LaunchConfiguration("sensor_roll"),
            "sensor_pitch": LaunchConfiguration("sensor_pitch"),
            "sensor_yaw": LaunchConfiguration("sensor_yaw"),
        }.items(),
    )

    hk_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("hik_camera_ros2_driver"),
                    "launch",
                    "hik_camera_launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_hk_camera")),
        launch_arguments={"show_image": "false"}.items(),
    )
    hk_capture = Node(
        package="hik_camera_ros2_driver",
        executable="dataset_capture_node.py",
        name="dataset_capture",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_hk_camera")),
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_task_control"),
                    "config",
                    "hk_capture.yaml",
                ]
            ),
            {
                "interactive_input": False,
                "output_dir": LaunchConfiguration("hk_output_dir"),
            },
        ],
    )
    hk_transfer = Node(
        package="robot_task_control",
        executable="hk_image_transfer",
        name="hk_image_transfer",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_image_transfer")),
        parameters=[
            {
                "delivery_mode": LaunchConfiguration("image_delivery_mode"),
                "image_topic": LaunchConfiguration("transfer_image_topic"),
                "listen_ip": LaunchConfiguration("image_listen_ip"),
                "allowed_client_ip": LaunchConfiguration("image_receiver_ip"),
                "listen_port": ParameterValue(
                    LaunchConfiguration("image_port"), value_type=int
                ),
                "max_fps": ParameterValue(
                    LaunchConfiguration("image_max_fps"), value_type=float
                ),
                "image_format": LaunchConfiguration("image_format"),
                "jpeg_quality": ParameterValue(
                    LaunchConfiguration("image_jpeg_quality"), value_type=int
                ),
            }
        ],
    )
    line_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ruier_camer_diver"),
                    "launch",
                    "line_tracking.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_line_camera")),
        launch_arguments={
            "namespace": "line_camera",
            "show_window": "false",
            "demo": "false",
            "video_device": LaunchConfiguration("line_camera_device"),
            "camera_frame": "line_camera_optical_frame",
        }.items(),
    )
    line_follower = Node(
        package="masc_chassis_user_control",
        executable="line_follow_node.py",
        name="line_follow_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_line_camera")),
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("masc_chassis_user_control"),
                    "config",
                    "line_follow.yaml",
                ]
            ),
            {
                "enabled": False,
                "show_window": ParameterValue(
                    LaunchConfiguration("line_show_window"), value_type=bool
                ),
                "image_forward_direction": LaunchConfiguration(
                    "line_image_forward_direction"
                ),
                "vehicle_reference_y_ratio": ParameterValue(
                    LaunchConfiguration("line_vehicle_reference_y_ratio"),
                    value_type=float,
                ),
                "debug_image_topic": "/line_camera/line/debug_image",
                "line_status_topic": "/line_camera/line/status",
                "centerline_topic": "/line_camera/line/centerline",
                "stop_marker_mask_topic": "/line_camera/line/mask",
                "cmd_vel_topic": "/robot_control/line_follow/cmd_vel",
                "forward_speed": ParameterValue(
                    LaunchConfiguration("line_follow_speed"), value_type=float
                ),
            },
        ],
    )
    line_exposure = Node(
        package="masc_chassis_user_control",
        executable="line_exposure_node.py",
        name="line_exposure_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("line_adaptive_exposure")),
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("masc_chassis_user_control"),
                    "config",
                    "adaptive_exposure.yaml",
                ]
            ),
            {
                "device_override": LaunchConfiguration("line_camera_device"),
                "image_topic": "/line_camera/image_raw",
                "line_status_topic": "/line_camera/line/status",
            },
        ],
    )
    line_system = GroupAction(
        condition=IfCondition(LaunchConfiguration("start_line_camera")),
        actions=[line_camera, line_follower, line_exposure],
        scoped=True,
    )
    velocity_mux = Node(
        package="robot_task_control",
        executable="velocity_mux",
        name="chassis_velocity_mux",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_velocity_mux")),
        parameters=[{"enabled_on_startup": False}],
    )

    workflow_controller = Node(
        package="robot_task_control",
        executable="task_controller",
        name="task_controller",
        output="screen",
        parameters=[
            {
                "use_velocity_mux": ParameterValue(
                    LaunchConfiguration("use_velocity_mux"), value_type=bool
                ),
                "stop_pid_before_follow": ParameterValue(
                    LaunchConfiguration("chassis_pid_enabled"), value_type=bool
                ),
                "require_image_receiver": ParameterValue(
                    LaunchConfiguration("start_image_transfer"), value_type=bool
                ),
                "workflow_file": LaunchConfiguration("workflow_file"),
                "require_arm_ready": ParameterValue(
                    LaunchConfiguration("require_arm_ready"), value_type=bool
                ),
                "check_devices_on_startup": ParameterValue(
                    LaunchConfiguration("check_devices_on_startup"), value_type=bool
                ),
                "auto_start": ParameterValue(
                    LaunchConfiguration("auto_start"), value_type=bool
                ),
            }
        ],
    )

    def start_control_stack(context):
        delay = float(LaunchConfiguration("arm_control_start_delay").perform(context))
        if not math.isfinite(delay) or delay < 0:
            raise ValueError("arm_control_start_delay must be finite and nonnegative")
        driver_enabled = LaunchConfiguration("start_arm_driver").perform(
            context
        ).lower() in ("true", "1")
        motion_enabled = LaunchConfiguration("start_moveit").perform(
            context
        ).lower() in ("true", "1")
        stack = [robot_motion_system, line_system, workflow_controller]
        if not driver_enabled or not motion_enabled or delay == 0:
            return stack
        gate = Node(
            package="eli_cs_robot_usr",
            executable="auto_admittance_start.py",
            name="arm_connection_wait",
            output="screen",
            parameters=[{"wait_only": True, "settle_seconds": delay}],
        )

        def connected(event, launch_context):
            if launch_context.is_shutdown:
                return []
            if event.returncode == 0:
                return stack
            get_logger("arm_startup").error(
                "Arm connection did not stabilize; control startup cancelled"
            )
            return [EmitEvent(event=Shutdown(reason="Arm connection startup failed"))]

        return [
            RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=connected)),
            gate,
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_hk_camera", default_value="true"),
            DeclareLaunchArgument("start_line_camera", default_value="true"),
            DeclareLaunchArgument("start_image_transfer", default_value="true"),
            DeclareLaunchArgument("image_listen_ip", default_value="0.0.0.0"),
            DeclareLaunchArgument("image_receiver_ip", default_value="169.254.199.120"),
            DeclareLaunchArgument("image_port", default_value="9002"),
            DeclareLaunchArgument("image_delivery_mode", default_value="photos"),
            DeclareLaunchArgument("image_format", default_value="jpeg"),
            DeclareLaunchArgument("image_jpeg_quality", default_value="95"),
            DeclareLaunchArgument(
                "transfer_image_topic", default_value="/dataset_capture/captured_image"
            ),
            DeclareLaunchArgument("image_max_fps", default_value="10.0"),
            DeclareLaunchArgument("hk_output_dir", default_value="~/robot_data/hk"),
            DeclareLaunchArgument("line_camera_device", default_value="/dev/video0"),
            DeclareLaunchArgument("line_follow_speed", default_value="0.075"),
            DeclareLaunchArgument("line_show_window", default_value="false"),
            DeclareLaunchArgument("line_adaptive_exposure", default_value="true"),
            DeclareLaunchArgument("line_image_forward_direction", default_value="down"),
            DeclareLaunchArgument(
                "line_vehicle_reference_y_ratio", default_value="-0.05"
            ),
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument("require_arm_ready", default_value="false"),
            DeclareLaunchArgument("check_devices_on_startup", default_value="false"),
            DeclareLaunchArgument("arm_control_start_delay", default_value="5.0"),
            DeclareLaunchArgument("start_chassis", default_value="true"),
            DeclareLaunchArgument("use_velocity_mux", default_value="true"),
            DeclareLaunchArgument("chassis_pid_enabled", default_value="true"),
            DeclareLaunchArgument("launch_arm_executor", default_value="true"),
            DeclareLaunchArgument("arm_use_cartesian_path", default_value="true"),
            DeclareLaunchArgument("start_arm_driver", default_value="true"),
            DeclareLaunchArgument("cs_type", default_value="ls65"),
            DeclareLaunchArgument("robot_ip", default_value="169.254.199.150"),
            DeclareLaunchArgument("local_ip", default_value="0.0.0.0"),
            DeclareLaunchArgument(
                "chassis_params",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("masc_chassis_ros_driver"),
                        "config",
                        "ros2",
                        "driver_params.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "task_plans",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("masc_chassis_ros_driver"),
                        "config",
                        "ros2",
                        "task_plans.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("start_moveit", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("launch_force_sensor", default_value="true"),
            DeclareLaunchArgument("sensor_host", default_value="169.254.199.80"),
            DeclareLaunchArgument("sensor_port", default_value="9001"),
            # force_sensor 在 tool0 中的安装角度，单位为弧度。
            DeclareLaunchArgument("sensor_roll", default_value="0.0"),
            DeclareLaunchArgument("sensor_pitch", default_value="0.0"),
            DeclareLaunchArgument("sensor_yaw", default_value="1.5707963267948966"),
            DeclareLaunchArgument(
                "workflow_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("robot_task_control"),
                        "config",
                        "workflow.yaml",
                    ]
                ),
            ),
            velocity_mux,
            chassis,
            hk_camera,
            hk_capture,
            hk_transfer,
            arm_driver,
            OpaqueFunction(function=start_control_stack),
        ]
    )
