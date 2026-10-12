import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the package directory
    bringup_dir = get_package_share_directory("hik_camera_ros2_driver")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")
    show_image = LaunchConfiguration("show_image")
    image_topic = LaunchConfiguration("image_topic")
    view_width = LaunchConfiguration("view_width")
    view_height = LaunchConfiguration("view_height")

    # Create the launch configuration variables
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    # Declare the launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "camera_params.yaml"),
        description="The joystick configuration file path",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    declare_show_image_cmd = DeclareLaunchArgument(
        "show_image", default_value="true", description="Open a live camera image window"
    )

    declare_image_topic_cmd = DeclareLaunchArgument(
        "image_topic", default_value="/camera/image",
        description="Image topic to display (match camera_topic in the parameter file)",
    )

    declare_view_width_cmd = DeclareLaunchArgument(
        "view_width", default_value="960", description="Camera window width in pixels"
    )
    declare_view_height_cmd = DeclareLaunchArgument(
        "view_height", default_value="720", description="Camera window height in pixels"
    )

    start_hik_camera_cmd = Node(
        name="hik_camera_ros2_driver",
        package="hik_camera_ros2_driver",
        executable="hik_camera_ros2_driver_node",
        parameters=[params_file],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    start_camera_view_cmd = Node(
        name="hik_camera_view",
        package="hik_camera_ros2_driver",
        executable="camera_view_node.py",
        parameters=[{
            "image_topic": image_topic,
            "window_width": ParameterValue(view_width, value_type=int),
            "window_height": ParameterValue(view_height, value_type=int),
        }],
        condition=IfCondition(show_image),
        output="screen",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch arguments
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_show_image_cmd)
    ld.add_action(declare_image_topic_cmd)
    ld.add_action(declare_view_width_cmd)
    ld.add_action(declare_view_height_cmd)

    # Add the actions to launch the nodes
    ld.add_action(start_hik_camera_cmd)
    ld.add_action(start_camera_view_cmd)

    return ld
