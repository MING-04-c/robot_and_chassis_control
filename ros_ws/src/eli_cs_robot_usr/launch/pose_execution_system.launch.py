"""Launch the pose execution pipeline for either Gazebo or a real robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_simulation = LaunchConfiguration("use_simulation")
    cs_type = LaunchConfiguration("cs_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    publisher_mode = LaunchConfiguration("publisher_mode")
    execute_after_input = LaunchConfiguration("execute_after_input")
    angles_in_degrees = LaunchConfiguration("angles_in_degrees")
    config_package = LaunchConfiguration("config_package")
    config_file = LaunchConfiguration("config_file")
    initial_config_package = LaunchConfiguration("initial_config_package")
    initial_config_file = LaunchConfiguration("initial_config_file")
    velocity_scaling = LaunchConfiguration("velocity_scaling")
    acceleration_scaling = LaunchConfiguration("acceleration_scaling")
    initialize_on_startup = LaunchConfiguration("initialize_on_startup")
    initialize_before_execution = LaunchConfiguration("initialize_before_execution")

    publisher_enabled = PythonExpression(["'", publisher_mode, "' != 'off'"])
    interactive_publisher = PythonExpression(["'", publisher_mode, "' == 'interactive'"])
    yaml_publisher = PythonExpression(["'", publisher_mode, "' == 'yaml'"])

    # config_file 使用 ROS 包 share/config 目录下的文件名。例如：
    #   config_package:=eli_cs_robot_usr
    #   config_file:=cartesian_path.yaml
    # 如果 config_file 以 / 开头，PathJoinSubstitution 会保留该绝对路径，
    # 因此之前使用绝对路径的启动命令仍然兼容。
    resolved_config_file = PathJoinSubstitution([
        FindPackageShare(config_package),
        "config",
        config_file,
    ])
    resolved_initial_config_file = PathJoinSubstitution([
        FindPackageShare(initial_config_package),
        "config",
        initial_config_file,
    ])

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("eli_cs_robot_simulation_gz"),
                "launch",
                "cs_sim_moveit.launch.py",
            ])
        ),
        condition=IfCondition(use_simulation),
        launch_arguments={
            "cs_type": cs_type,
            "launch_rviz": launch_rviz,
            "launch_servo": "false",
        }.items(),
    )

    # The real robot driver must already be running. This include starts MoveIt
    # with the real scaled_joint_trajectory_controller selected as its default.
    real_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("eli_cs_robot_moveit_config"),
                "launch",
                "cs_moveit.launch.py",
            ])
        ),
        condition=UnlessCondition(use_simulation),
        launch_arguments={
            "cs_type": cs_type,
            "use_fake_hardware": "false",
            "use_sim_time": "false",
            "launch_rviz": launch_rviz,
            "launch_servo": "false",
        }.items(),
    )

    # MoveIt executor is defined directly here. There is no second launch file
    # to maintain just for starting this one node.
    executor_node = Node(
        package="eli_cs_robot_usr",
        executable="moveit_pose_executor",
        name="moveit_pose_executor",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("eli_cs_robot_moveit_config"),
                "config",
                "kinematics.yaml",
            ]),
            # Initial joint configuration is kept separate from Cartesian targets.
            resolved_initial_config_file,
            {
                "planning_group": "cs_manipulator",
                "end_effector_link": "tool0",
                "reference_frame": "base_link",
                # 这是 MoveIt 生成轨迹时使用的名义限制。真机控制器还会在
                # 执行过程中继续乘以示教器的实时速度缩放值。
                "velocity_scaling": velocity_scaling,
                "acceleration_scaling": acceleration_scaling,
                "planning_time": 5.0,
                "cartesian_eef_step": 0.01,
                "cartesian_jump_threshold": 0.0,
                "cartesian_min_fraction": 0.95,
                "use_sim_time": use_simulation,
                "initialize_on_startup": ParameterValue(
                    initialize_on_startup, value_type=bool),
                "initialize_before_execution": ParameterValue(
                    initialize_before_execution, value_type=bool),
            },
        ],
    )

    publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("eli_cs_robot_usr"),
                "launch",
                "pose_target_publisher.launch.py",
            ])
        ),
        condition=IfCondition(publisher_enabled),
        launch_arguments={
            "use_sim_time": use_simulation,
            "config_file": resolved_config_file,
            "interactive_input": interactive_publisher,
            "execute_after_input": execute_after_input,
            "execute_after_yaml": yaml_publisher,
            "angles_in_degrees": angles_in_degrees,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_simulation",
            default_value="true",
            description=(
                "true: Gazebo + joint_trajectory_controller; "
                "false: real robot + scaled_joint_trajectory_controller"
            ),
        ),
        DeclareLaunchArgument(
            "cs_type",
            default_value="ls65",
            description="Elite robot model",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz",
        ),
        DeclareLaunchArgument(
            "publisher_mode",
            default_value="off",
            choices=["off", "interactive", "yaml"],
            description=(
                "off: no pose publisher; interactive: terminal input; "
                "yaml: publish and execute config_file"
            ),
        ),
        DeclareLaunchArgument(
            "execute_after_input",
            default_value="true",
            description="Automatically execute each interactive pose",
        ),
        DeclareLaunchArgument(
            "angles_in_degrees",
            default_value="false",
            description="true is interpreted as degrees, false as radians",
        ),
        DeclareLaunchArgument(
            "config_package",
            default_value="eli_cs_robot_usr",
            description="ROS package containing the target pose YAML",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="cartesian_path.yaml",
            description=(
                "File under config_package/config, or an absolute path"
            ),
        ),
        DeclareLaunchArgument(
            "initial_config_package",
            default_value="eli_cs_robot_usr",
            description="ROS package containing the initial joint pose YAML",
        ),
        DeclareLaunchArgument(
            "initial_config_file",
            default_value="initial_joint_pose.yaml",
            description=(
                "File under initial_config_package/config, or an absolute path"
            ),
        ),
        DeclareLaunchArgument(
            "velocity_scaling",
            default_value="0.5",
            description="MoveIt nominal velocity scaling in the range (0, 1]",
        ),
        DeclareLaunchArgument(
            "acceleration_scaling",
            default_value="0.5",
            description="MoveIt nominal acceleration scaling in the range (0, 1]",
        ),
        DeclareLaunchArgument(
            "initialize_on_startup",
            default_value="true",
            description="Move to initial_joint_pose when the executor starts",
        ),
        DeclareLaunchArgument(
            "initialize_before_execution",
            default_value="true",
            description="Move to initial_joint_pose before each target",
        ),
        simulation_launch,
        real_moveit_launch,
        executor_node,
        TimerAction(period=5.0, actions=[publisher_launch]),
    ])
