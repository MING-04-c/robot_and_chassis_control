"""共用检测驱动栈：供真机检测入口复用，保留假硬件调试参数。"""

import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    RegisterEventHandler,
    EmitEvent,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.logging import get_logger
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    # 延迟事件会在 include 的局部参数作用域结束后执行；此处先冻结参数值。
    def config(name):
        return LaunchConfiguration(name).perform(context)

    simulation_value = config("simulation").lower()
    if simulation_value not in ("true", "false"):
        raise ValueError("simulation 必须为 true 或 false")
    simulation = simulation_value == "true"
    profile = config("test_config")
    actions = []

    def include(package, filename, arguments, **kwargs):
        return GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [FindPackageShare(package), "launch", filename]
                        )
                    ),
                    launch_arguments=arguments.items(),
                )
            ],
            scoped=True,
            **kwargs,
        )

    if simulation:
        domain = int(config("simulation_domain"))
        if not 0 <= domain <= 232:
            raise ValueError("simulation_domain 必须在 0～232 之间")
        actions.append(SetEnvironmentVariable("ROS_DOMAIN_ID", str(domain)))

        actions.extend(
            [
                include(
                    "eli_cs_robot_driver",
                    "elite_control.launch.py",
                    {
                        "cs_type": config("cs_type"),
                        "robot_ip": "127.0.0.1",
                        "use_fake_hardware": "true",
                        "launch_rviz": "false",
                        "initial_joint_controller": "joint_trajectory_controller",
                        "activate_joint_controller": "true",
                    },
                ),
                include(
                    "eli_cs_robot_moveit_config",
                    "cs_moveit.launch.py",
                    {
                        "cs_type": config("cs_type"),
                        "use_fake_hardware": "true",
                        "launch_rviz": "false",
                        "launch_servo": "false",
                        "publish_robot_description": "false",
                    },
                ),
                Node(
                    package="eli_cs_robot_usr",
                    executable="admittance_pose_controller",
                    name="admittance_controller",
                    output="screen",
                    parameters=[
                        PathJoinSubstitution(
                            [
                                FindPackageShare("eli_cs_robot_usr"),
                                "config",
                                "compliant_real.yaml",
                            ]
                        ),
                        PathJoinSubstitution(
                            [
                                FindPackageShare("eli_cs_robot_moveit_config"),
                                "config",
                                "kinematics.yaml",
                            ]
                        ),
                        {
                            "enabled_on_startup": False,
                            "force_source": "topic",
                            "require_speed_scaling": False,
                            "joint_command_topic": "/joint_trajectory_controller/joint_trajectory",
                        },
                    ],
                ),
            ]
        )
    else:
        actions.append(include(
            "eli_cs_robot_serial", "detactor.launch.py",
            {"config_file": config("detactor_config_file"),
             "auto_request": "false", "data_directory": "~/.ros/eli_cs_robot_serial/detactor"},
            condition=IfCondition(config("start_thickness_detector")),
        ))
        actions.append(
            include(
                "eli_cs_robot_driver",
                "elite_control.launch.py",
                {
                    "cs_type": config("cs_type"),
                    "robot_ip": config("robot_ip"),
                    "local_ip": config("local_ip"),
                    "headless_mode": config("headless_mode"),
                    "use_fake_hardware": "false",
                    "launch_rviz": "false",
                    "initial_joint_controller": "scaled_joint_trajectory_controller",
                    "activate_joint_controller": "true",
                },
                condition=IfCondition(config("start_arm_driver")),
            )
        )
        # 后续组件先收集起来，待驱动控制器与外控连接稳定后统一加载。
        control_stack = include(
            "eli_cs_robot_usr",
            "compliant_control_real.launch.py",
            {
                "cs_type": config("cs_type"),
                "launch_rviz": "false",
                "auto_start_admittance": "false",
                "admittance_enabled_on_startup": "false",
                "launch_executor": "false",
                "launch_coordinator": "false",
                "launch_target_publisher": "false",
                "launch_sensor": config("launch_force_sensor"),
                **{
                    key: config(key)
                    for key in (
                        "sensor_host",
                        "sensor_port",
                        "sensor_config_file",
                        "sensor_x",
                        "sensor_y",
                        "sensor_z",
                        "sensor_roll",
                        "sensor_pitch",
                        "sensor_yaw",
                    )
                },
            },
        )
    startup_action_count = len(actions)
    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="detection_test_rviz",
            condition=IfCondition(config("launch_rviz")),
            output="log",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [
                        FindPackageShare("robot_task_control"),
                        "rviz",
                        "detection_test.rviz",
                    ]
                ),
                "-t",
                (
                    f"DETECTION TEST - SIM - Domain {domain}"
                    if simulation
                    else "DETECTION TEST - REAL"
                ),
            ],
        )
    )

    actions.extend(
        [
            Node(
                package="detect_control",
                executable="detect_controller.py",
                name="detect_controller",
                output="screen",
                sigterm_timeout="3.0",
                sigkill_timeout="2.0",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("detect_control"), "config", "detect.yaml"]
                    ),
                    profile,
                    {
                        "targets_file": ParameterValue(
                            config("targets_file"), value_type=str
                        )
                    },
                ],
            ),
            Node(
                package="robot_task_control",
                executable="virtual_detection_task",
                name="virtual_detection_task",
                condition=IfCondition(config("start_test_task")),
                output="screen",
                parameters=[profile, {"simulation": simulation}],
            ),
        ]
    )
    if not simulation:
        enabled = config("start_arm_control").lower()
        if enabled not in ("true", "false"):
            raise ValueError("start_arm_control 必须为 true 或 false")
        if enabled == "true":
            delay = float(
                config("arm_control_start_delay")
            )
            if not math.isfinite(delay) or delay < 0:
                raise ValueError("arm_control_start_delay 必须为有限非负秒数")
            # RViz、检测节点与 MoveIt 一并延后，避免挤占控制器加载阶段的 CPU。
            stack = [control_stack, *actions[startup_action_count:]]
            actions = actions[:startup_action_count]
            gate = Node(
                package="eli_cs_robot_usr",
                executable="auto_admittance_start.py",
                name="detection_connection_wait",
                output="screen",
                parameters=[
                    {
                        "wait_only": True,
                        "wait_for_all_controllers": True,
                        "settle_seconds": delay,
                    }
                ],
            )

            def connected(event, launch_context):
                if launch_context.is_shutdown:
                    return []
                if event.returncode == 0:
                    return stack
                get_logger("detection_startup").error(
                    "机械臂外控或控制器未稳定，取消检测控制系统启动"
                )
                return [
                    EmitEvent(event=Shutdown(reason="Robot connection startup failed"))
                ]

            actions.extend(
                [
                    RegisterEventHandler(
                        OnProcessExit(target_action=gate, on_exit=connected)
                    ),
                    gate,
                ]
            )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "simulation",
                default_value="true",
                description="true: start mock arm; false: start real arm and detection task.",
            ),
            DeclareLaunchArgument("simulation_domain", default_value="186"),
            DeclareLaunchArgument("start_test_task", default_value="true",
                                  description="false: 只启动检测驱动和控制系统，不自动发送旧测试点"),
            DeclareLaunchArgument("cs_type", default_value="ls65"),
            DeclareLaunchArgument("robot_ip", default_value="169.254.199.150"),
            DeclareLaunchArgument("local_ip", default_value="0.0.0.0"),
            DeclareLaunchArgument("headless_mode", default_value="true"),
            DeclareLaunchArgument(
                "start_arm_driver",
                default_value="true",
                description="Real mode: start the robot driver; false reuses an existing driver.",
            ),
            DeclareLaunchArgument(
                "start_arm_control",
                default_value="true",
                description="Real mode: start MoveIt, admittance and optional force sensor.",
            ),
            DeclareLaunchArgument("arm_control_start_delay", default_value="5.0"),
            DeclareLaunchArgument("start_thickness_detector", default_value="true",
                                  description="真机时启用测厚仪；已有测厚节点则设 false，仿真不启用"),
            DeclareLaunchArgument("detactor_config_file", default_value=PathJoinSubstitution([
                FindPackageShare("eli_cs_robot_serial"), "device", "detactor", "config", "detactor.yaml"
            ])),
            DeclareLaunchArgument(
                "launch_force_sensor",
                default_value="true",
                description="Start the force sensor with start_arm_control; false reuses its driver.",
            ),
            DeclareLaunchArgument("sensor_host", default_value="169.254.199.80"),
            DeclareLaunchArgument("sensor_port", default_value="9001"),
            DeclareLaunchArgument(
                "sensor_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("eli_cs_robot_serial"),
                        "device",
                        "sensor",
                        "config",
                        "sensor_tcp.yaml",
                    ]
                ),
            ),
            *[
                DeclareLaunchArgument(key, default_value=value)
                for key, value in (
                    ("sensor_x", "0.0"),
                    ("sensor_y", "0.0"),
                    ("sensor_z", "0.0"),
                    ("sensor_roll", "0.0"),
                    ("sensor_pitch", "0.0"),
                    ("sensor_yaw", "1.5707963267948966"),
                )
            ],
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "test_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("robot_task_control"),
                        "config",
                        "tests",
                        "detection_test.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "targets_file",
                default_value="~/.ros/detect_control/virtual_test_targets.yaml",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
