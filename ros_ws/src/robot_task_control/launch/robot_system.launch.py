from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    chassis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("masc_chassis_ros_driver"), "launch",
            "masc_chassis_ros_driver.launch.py",
        ])),
        condition=IfCondition(LaunchConfiguration("start_chassis")),
        launch_arguments={
            "driver_params": LaunchConfiguration("chassis_params"),
            "pid_cmd_vel_topic": "/robot_control/pid/cmd_vel",
            "task_plans": LaunchConfiguration("task_plans"),
        }.items(),
    )
    arm_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("eli_cs_robot_driver"), "launch", "elite_control.launch.py",
        ])),
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
            # 导纳步骤使用 IK；MoveIt 步骤由独立执行器处理。
            "launch_executor": "true",
            "cs_type": LaunchConfiguration("cs_type"),
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

    hk_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("hik_camera_ros2_driver"), "launch", "hik_camera_launch.py",
        ])),
        condition=IfCondition(LaunchConfiguration("start_hk_camera")),
        launch_arguments={"show_image": "false"}.items(),
    )
    hk_capture = Node(
        package="hik_camera_ros2_driver", executable="dataset_capture_node.py",
        name="dataset_capture", output="screen",
        condition=IfCondition(LaunchConfiguration("start_hk_camera")),
        parameters=[PathJoinSubstitution([
            FindPackageShare("robot_task_control"), "config", "hk_capture.yaml",
        ]), {"interactive_input": False, "output_dir": LaunchConfiguration("hk_output_dir")}],
    )
    hk_transfer = Node(
        package="robot_task_control", executable="hk_image_transfer",
        name="hk_image_transfer", output="screen",
        condition=IfCondition(LaunchConfiguration("start_image_transfer")),
        parameters=[{
            "delivery_mode": LaunchConfiguration("image_delivery_mode"),
            "image_topic": LaunchConfiguration("transfer_image_topic"),
            "listen_ip": LaunchConfiguration("image_listen_ip"),
            "allowed_client_ip": LaunchConfiguration("image_receiver_ip"),
            "listen_port": ParameterValue(LaunchConfiguration("image_port"), value_type=int),
            "max_fps": ParameterValue(LaunchConfiguration("image_max_fps"), value_type=float),
            "image_format": LaunchConfiguration("image_format"),
            "jpeg_quality": ParameterValue(LaunchConfiguration("image_jpeg_quality"), value_type=int),
        }],
    )
    line_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("camer_diver"), "launch", "line_tracking.launch.py",
        ])),
        condition=IfCondition(LaunchConfiguration("start_line_camera")),
        launch_arguments={
            "namespace": "line_camera", "show_window": "false", "demo": "false",
            "video_device": LaunchConfiguration("line_camera_device"),
            "camera_frame": "line_camera_optical_frame",
        }.items(),
    )
    line_follower = Node(
        package="masc_chassis_user_control", executable="line_follow_node.py",
        name="line_follow_controller", output="screen",
        condition=IfCondition(LaunchConfiguration("start_line_camera")),
        parameters=[PathJoinSubstitution([
            FindPackageShare("masc_chassis_user_control"), "config", "line_follow.yaml",
        ]), {
            "enabled": False,
            "line_status_topic": "/line_camera/line/status",
            "centerline_topic": "/line_camera/line/centerline",
            "cmd_vel_topic": "/robot_control/line_follow/cmd_vel",
            "forward_speed": ParameterValue(LaunchConfiguration("line_follow_speed"), value_type=float),
        }],
    )
    velocity_mux = Node(
        package="robot_task_control", executable="velocity_mux",
        name="chassis_velocity_mux", output="screen",
    )

    workflow_controller = Node(
        package="robot_task_control",
        executable="task_controller",
        name="task_controller",
        output="screen",
        parameters=[{
            "use_velocity_mux": True,
            "require_image_receiver": ParameterValue(
                LaunchConfiguration("start_image_transfer"), value_type=bool),
            "workflow_file": LaunchConfiguration("workflow_file"),
            "auto_start": ParameterValue(
                LaunchConfiguration("auto_start"), value_type=bool
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_hk_camera", default_value="true"),
        DeclareLaunchArgument("start_line_camera", default_value="true"),
        DeclareLaunchArgument("start_image_transfer", default_value="true"),
        DeclareLaunchArgument("image_listen_ip", default_value="169.254.199.100"),
        DeclareLaunchArgument("image_receiver_ip", default_value="169.254.199.250"),
        DeclareLaunchArgument("image_port", default_value="9002"),
        DeclareLaunchArgument("image_delivery_mode", default_value="photos"),
        DeclareLaunchArgument("image_format", default_value="jpeg"),
        DeclareLaunchArgument("image_jpeg_quality", default_value="95"),
        DeclareLaunchArgument("transfer_image_topic", default_value="/dataset_capture/captured_image"),
        DeclareLaunchArgument("image_max_fps", default_value="10.0"),
        DeclareLaunchArgument("hk_output_dir", default_value="~/robot_data/hk"),
        DeclareLaunchArgument("line_camera_device", default_value="/dev/video0"),
        DeclareLaunchArgument("line_follow_speed", default_value="0.05"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("start_chassis", default_value="true"),
        DeclareLaunchArgument("start_arm_driver", default_value="true"),
        DeclareLaunchArgument("cs_type", default_value="ls65"),
        DeclareLaunchArgument("robot_ip", default_value="169.254.199.150"),
        DeclareLaunchArgument("local_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("chassis_params", default_value=PathJoinSubstitution([
            FindPackageShare("masc_chassis_ros_driver"), "config", "ros2", "driver_params.yaml",
        ])),
        DeclareLaunchArgument("task_plans", default_value=PathJoinSubstitution([
            FindPackageShare("masc_chassis_ros_driver"), "config", "ros2", "task_plans.yaml",
        ])),
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
            default_value=PathJoinSubstitution([
                FindPackageShare("robot_task_control"),
                "config",
                "workflow.yaml",
            ]),
        ),
        velocity_mux,
        chassis,
        hk_camera, hk_capture, hk_transfer,
        line_camera, line_follower,
        arm_driver,
        robot_motion_system,
        workflow_controller,
    ])
