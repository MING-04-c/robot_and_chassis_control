"""Start the MoveIt Servo keyboard window for simulation or a real robot."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    environment = LaunchConfiguration("environment").perform(context)
    if environment not in ("simulation", "real"):
        raise RuntimeError("environment must be 'simulation' or 'real'")

    is_simulation = environment == "simulation"
    cs_type = LaunchConfiguration("cs_type")
    requested_linear_speed = LaunchConfiguration("linear_speed").perform(context)
    requested_angular_speed = LaunchConfiguration("angular_speed").perform(context)

    # Empty values select conservative real-robot defaults and more visible
    # simulation defaults. Users may override either value on the command line.
    linear_speed = float(
        requested_linear_speed or ("0.05" if is_simulation else "0.01")
    )
    angular_speed = float(
        requested_angular_speed or ("0.20" if is_simulation else "0.05")
    )
    publish_rate = float(LaunchConfiguration("publish_rate").perform(context))

    if is_simulation:
        backend = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("eli_cs_robot_simulation_gz"),
                    "launch",
                    "cs_sim_moveit.launch.py",
                ])
            ),
            launch_arguments={
                "cs_type": cs_type,
                "launch_rviz": LaunchConfiguration("launch_rviz"),
                "launch_servo": "true",
            }.items(),
        )
        use_sim_time = True
        start_delay = 6.0
    else:
        # The real robot driver and scaled_joint_trajectory_controller must be
        # running before this launch is started.
        backend = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("eli_cs_robot_moveit_config"),
                    "launch",
                    "cs_moveit.launch.py",
                ])
            ),
            launch_arguments={
                "cs_type": cs_type,
                "use_fake_hardware": "false",
                "use_sim_time": "false",
                "launch_rviz": LaunchConfiguration("launch_rviz"),
                "launch_servo": "true",
            }.items(),
        )
        use_sim_time = False
        start_delay = 5.0

    keyboard_node = Node(
        package="eli_cs_robot_usr",
        executable="pose_keyboard_teleop",
        name="pose_keyboard_teleop",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "command_frame": "base_link",
            "linear_speed": linear_speed,
            "angular_speed": angular_speed,
            "publish_rate": publish_rate,
        }],
    )

    return [backend, TimerAction(period=start_delay, actions=[keyboard_node])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "environment",
            default_value="simulation",
            choices=["simulation", "real"],
            description="simulation: Gazebo; real: a separately started real robot driver",
        ),
        DeclareLaunchArgument("cs_type", default_value="ls65"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument(
            "linear_speed",
            default_value="",
            description="m/s; empty chooses the environment default",
        ),
        DeclareLaunchArgument(
            "angular_speed",
            default_value="",
            description="rad/s; empty chooses the environment default",
        ),
        DeclareLaunchArgument("publish_rate", default_value="50.0"),
        OpaqueFunction(function=launch_setup),
    ])
