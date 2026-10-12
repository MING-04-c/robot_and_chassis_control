"""Gazebo: MoveIt approaches a target, then admittance controls the last segment."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    usr = FindPackageShare("eli_cs_robot_usr")
    moveit_cfg = FindPackageShare("eli_cs_robot_moveit_config")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("eli_cs_robot_simulation_gz"), "launch", "cs_sim_ft.launch.py"
        ])),
        launch_arguments={
            "cs_type": "ls65",
            "launch_rviz": "false",
            "initial_joint_controller": "joint_trajectory_controller",
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            moveit_cfg, "launch", "cs_moveit.launch.py"
        ])),
        launch_arguments={
            "cs_type": "ls65", "use_fake_hardware": "true",
            "use_sim_time": "true", "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_servo": "false",
        }.items(),
    )
    executor = Node(
        package="eli_cs_robot_usr", executable="moveit_pose_executor",
        name="moveit_pose_executor", output="screen",
        parameters=[
            PathJoinSubstitution([moveit_cfg, "config", "kinematics.yaml"]),
            PathJoinSubstitution([usr, "config", "initial_joint_pose.yaml"]),
            {"planning_group": "cs_manipulator", "end_effector_link": "tool0",
             "reference_frame": "base_link", "velocity_scaling": 0.2,
             "acceleration_scaling": 0.2, "use_sim_time": True},
        ],
    )
    admittance = Node(
        package="eli_cs_robot_usr", executable="admittance_pose_controller",
        name="admittance_controller", output="screen",
        parameters=[
            PathJoinSubstitution([usr, "config", "compliant_sim.yaml"]),
            PathJoinSubstitution([moveit_cfg, "config", "kinematics.yaml"]),
            # In this dedicated test launch, force is a software input. It is
            # not applied to Gazebo rigid-body dynamics, so the response is
            # determined only by the admittance law and the joint controller.
            {"use_sim_time": True, "force_source": "topic"},
        ],
    )
    coordinator = Node(
        package="eli_cs_robot_usr", executable="staged_pose_coordinator",
        name="staged_pose_coordinator", output="screen",
        parameters=[PathJoinSubstitution([usr, "config", "compliant_sim.yaml"]),
                    {"use_sim_time": True}],
    )
    publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([usr, "launch", "pose_target_publisher.launch.py"])),
        launch_arguments={
            "target_topic": "/staged_pose_coordinator/target_poses",
            "use_sim_time": "true", "interactive_input": "true",
            "execute_after_input": "false", "execute_after_yaml": "false",
            "angles_in_degrees": LaunchConfiguration("angles_in_degrees"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("angles_in_degrees", default_value="true"),
        GroupAction(actions=[simulation], scoped=True),
        GroupAction(actions=[moveit], scoped=True),
        executor, admittance, coordinator,
        TimerAction(period=5.0, actions=[publisher]),
    ])
