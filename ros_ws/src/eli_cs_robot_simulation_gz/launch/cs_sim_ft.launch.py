"""Start the isolated LS65 Gazebo force/torque sensor simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare("eli_cs_robot_simulation_gz"),
        "worlds",
        "ls65_ft_world.sdf",
    ])

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("eli_cs_robot_simulation_gz"),
            "launch",
            "cs_sim_control.launch.py",
        ])),
        launch_arguments={
            "cs_type": LaunchConfiguration("cs_type"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "world": world,
            "enable_ft_sensor": "true",
            "initial_joint_controller": LaunchConfiguration("initial_joint_controller"),
        }.items(),
    )

    wrench_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="tcp_ft_bridge",
        arguments=[
            "/ls65/tcp_ft@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench",
        ],
        remappings=[
            ("/ls65/tcp_ft", "/force_torque_sensor_broadcaster/ft_data"),
        ],
        output="screen",
    )

    # MoveIt Servo and the admittance node use ROS time. Gazebo publishes its
    # simulation clock on the Gazebo transport, so bridge it explicitly or
    # TwistStamped headers remain at sec=0 and Servo rejects the commands.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="simulation_clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("cs_type", default_value="ls65"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Controller to activate for this dedicated F/T simulation.",
        ),
        simulation,
        clock_bridge,
        wrench_bridge,
    ])
