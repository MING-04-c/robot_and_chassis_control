"""Real robot: use the existing driver and scaled trajectory controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    usr = FindPackageShare("eli_cs_robot_usr")
    moveit_cfg = FindPackageShare("eli_cs_robot_moveit_config")
    auto_start = Node(
        package="eli_cs_robot_usr", executable="auto_admittance_start.py",
        name="auto_admittance_start", output="screen",
        condition=IfCondition(LaunchConfiguration("auto_start_admittance")),
        parameters=[{"settle_seconds": ParameterValue(
            LaunchConfiguration("arm_control_start_delay"), value_type=float)}],
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([moveit_cfg, "launch", "cs_moveit.launch.py"])),
        launch_arguments={
            "cs_type": LaunchConfiguration("cs_type"), "use_fake_hardware": "false",
            "use_sim_time": "false", "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_servo": "false",
            # The driver's robot_state_publisher owns /robot_description.
            # MoveIt's planning URDF omits the real hardware connection arguments;
            # publishing it races the driver at controller-manager startup.
            "publish_robot_description": "false",
        }.items(),
    )
    admittance = Node(
        package="eli_cs_robot_usr", executable="admittance_pose_controller",
        name="admittance_controller", output="screen",
        sigterm_timeout="3.0", sigkill_timeout="2.0",
        parameters=[PathJoinSubstitution([usr, "config", "compliant_real.yaml"]),
                    PathJoinSubstitution([moveit_cfg, "config", "kinematics.yaml"]),
                    {"use_sim_time": False,
                     "enabled_on_startup": ParameterValue(
                         LaunchConfiguration("admittance_enabled_on_startup"), value_type=bool
                     )}],
    )
    executor = Node(
        package="eli_cs_robot_usr", executable="moveit_pose_executor",
        name="moveit_pose_executor", output="screen",
        condition=IfCondition(LaunchConfiguration("launch_executor")),
        parameters=[
            PathJoinSubstitution([moveit_cfg, "config", "kinematics.yaml"]),
            {
                "planning_group": "cs_manipulator",
                "end_effector_link": "tool0",
                "reference_frame": "base_link",
                "velocity_scaling": 0.10,
                "acceleration_scaling": 0.10,
                "planning_time": 5.0,
                # 笛卡尔插值用于直线运动；单点长距离接近可选择普通规划。
                "cartesian_eef_step": 0.01,
                "cartesian_jump_threshold": 0.0,
                "cartesian_min_fraction": 0.95,
                "use_sim_time": False,
                "initialize_on_startup": False,
                "initialize_before_execution": False,
                "use_cartesian_path": ParameterValue(
                    LaunchConfiguration("use_cartesian_path"), value_type=bool
                ),
            },
        ],
    )
    coordinator = Node(
        package="eli_cs_robot_usr", executable="staged_pose_coordinator",
        name="staged_pose_coordinator", output="screen",
        condition=IfCondition(LaunchConfiguration("launch_coordinator")),
        parameters=[PathJoinSubstitution([usr, "config", "compliant_real.yaml"]),
                    {"use_sim_time": False}],
    )
    publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([usr, "launch", "pose_target_publisher.launch.py"])),
        condition=IfCondition(LaunchConfiguration("launch_target_publisher")),
        launch_arguments={
            "target_topic": "/staged_pose_coordinator/target_poses",
            "use_sim_time": "false", "interactive_input": LaunchConfiguration("interactive_input"),
            "execute_after_input": "false", "execute_after_yaml": "false",
            "angles_in_degrees": LaunchConfiguration("angles_in_degrees"),
        }.items(),
    )
    sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("eli_cs_robot_serial"), "launch", "sensor_tcp.launch.py"
        ])),
        launch_arguments={
            "config_file": LaunchConfiguration("sensor_config_file"),
            "host": LaunchConfiguration("sensor_host"),
            "port": LaunchConfiguration("sensor_port"),
            "frame_id": "force_sensor",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_sensor")),
    )
    sensor_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="force_sensor_mount_tf",
        output="screen",
        arguments=[
            "--x", LaunchConfiguration("sensor_x"),
            "--y", LaunchConfiguration("sensor_y"),
            "--z", LaunchConfiguration("sensor_z"),
            "--roll", LaunchConfiguration("sensor_roll"),
            "--pitch", LaunchConfiguration("sensor_pitch"),
            "--yaw", LaunchConfiguration("sensor_yaw"),
            "--frame-id", "tool0",
            "--child-frame-id", "force_sensor",
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("cs_type", default_value="ls65"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("auto_start_admittance", default_value="true"),
        DeclareLaunchArgument("arm_control_start_delay", default_value="5.0"),
        DeclareLaunchArgument("admittance_enabled_on_startup", default_value="false"),
        DeclareLaunchArgument("launch_coordinator", default_value="false"),
        DeclareLaunchArgument("launch_executor", default_value="false"),
        DeclareLaunchArgument(
            "use_cartesian_path", default_value="true",
            description="Use Cartesian interpolation; false uses pose planning for a single target.",
        ),
        DeclareLaunchArgument("interactive_input", default_value="false"),
        DeclareLaunchArgument("launch_target_publisher", default_value="false"),
        DeclareLaunchArgument("angles_in_degrees", default_value="true"),
        DeclareLaunchArgument("launch_sensor", default_value="true"),
        DeclareLaunchArgument("sensor_host", default_value="169.254.199.80"),
        DeclareLaunchArgument("sensor_port", default_value="9001"),
        # force_sensor 原点/轴系在 tool0 中的固定安装位姿；角度单位为弧度。
        DeclareLaunchArgument("sensor_x", default_value="0.0"),
        DeclareLaunchArgument("sensor_y", default_value="0.0"),
        DeclareLaunchArgument("sensor_z", default_value="0.0"),
        DeclareLaunchArgument("sensor_roll", default_value="0.0"),
        DeclareLaunchArgument("sensor_pitch", default_value="0.0"),
        # Sensor +X points along tool0 +Y, and sensor -Y points along tool0 +X.
        DeclareLaunchArgument("sensor_yaw", default_value="1.5707963267948966"),
        DeclareLaunchArgument(
            "sensor_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("eli_cs_robot_serial"),
                "device", "sensor", "config", "sensor_tcp.yaml",
            ]),
        ),
        moveit, sensor_mount_tf, sensor, executor, admittance, auto_start, coordinator, publisher,
    ])
