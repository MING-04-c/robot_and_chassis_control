from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def port_node(index):
    device = LaunchConfiguration(f"port{index}_device")
    return Node(
        package="eli_cs_robot_serial", executable="serial_node",
        name=f"serial_port{index}", namespace=f"serial{index}", output="screen",
        condition=IfCondition(PythonExpression(["'", device, "' != ''"])),
        parameters=[{
            "device": device,
            "baudrate": LaunchConfiguration(f"port{index}_baudrate"),
            "line_mode": LaunchConfiguration(f"port{index}_line_mode"),
            "append_newline": LaunchConfiguration(f"port{index}_append_newline"),
            "text_topic": "rx", "bytes_topic": "rx_bytes", "write_service": "write",
            "tx_string_topic": "tx", "tx_float_topic": "tx_float", "tx_int_topic": "tx_int",
        }],
    )


def generate_launch_description():
    arguments = []
    for index, default_device in ((1, "/dev/ttyUSB0"), (2, ""), (3, "")):
        arguments.extend([
            DeclareLaunchArgument(f"port{index}_device", default_value=default_device),
            DeclareLaunchArgument(f"port{index}_baudrate", default_value="115200"),
            DeclareLaunchArgument(f"port{index}_line_mode", default_value="true"),
            DeclareLaunchArgument(f"port{index}_append_newline", default_value="true"),
        ])
    return LaunchDescription(arguments + [port_node(1), port_node(2), port_node(3)])
