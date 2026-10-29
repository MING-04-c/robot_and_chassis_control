"""工控机结果回传及可选的独立海康照片通道，不启动相机或机器人。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def setup(context):
    overrides = {}
    for name in ("bind_host", "port", "key_file", "output_dir"):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = int(value) if name == "port" else value
    hk_overrides = {}
    for argument, parameter in (("hk_bind_host", "listen_ip"),
                                ("industrial_pc_ip", "allowed_client_ip"),
                                ("hk_port", "listen_port")):
        value = LaunchConfiguration(argument).perform(context)
        if value:
            hk_overrides[parameter] = int(value) if argument == "hk_port" else value
    return [
        Node(
            package="eli_cs_robot_serial", executable="industrial_pc_node.py",
            name="industrial_pc", output="screen",
            parameters=[LaunchConfiguration("config_file"), overrides],
        ),
        Node(
            package="eli_cs_robot_serial", executable="industrial_pc_hk_transfer.py",
            name="hk_image_transfer", output="screen",
            condition=IfCondition(LaunchConfiguration("start_hk_transfer")),
            parameters=[LaunchConfiguration("hk_config_file"), hk_overrides],
        ),
    ]


def generate_launch_description():
    config_dir = [FindPackageShare("eli_cs_robot_serial"), "device", "industrial_pc", "config"]
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=PathJoinSubstitution(
            [*config_dir, "industrial_pc.yaml"])),
        *[DeclareLaunchArgument(name, default_value="", description="留空使用 YAML 配置")
          for name in ("bind_host", "port", "key_file", "output_dir")],
        DeclareLaunchArgument("start_hk_transfer", default_value="false",
                              description="启用独立海康拍照发送通道；不启动相机或触发拍照"),
        DeclareLaunchArgument("hk_config_file", default_value=PathJoinSubstitution(
            [*config_dir, "hk_transfer.yaml"])),
        *[DeclareLaunchArgument(name, default_value="", description="留空使用海康 YAML 配置")
          for name in ("hk_bind_host", "industrial_pc_ip", "hk_port")],
        OpaqueFunction(function=setup),
    ])
