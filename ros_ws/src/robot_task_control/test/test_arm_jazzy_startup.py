#!/usr/bin/env python3
"""Start real ROS controllers and MoveIt with mock hardware on an isolated domain.

Run after sourcing ROS and the built workspace. No motion commands are sent.
Optional controllers missing from a minimal ROS installation are excluded.
"""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]

LAUNCH = """
import importlib.util
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions

ROOT = Path({root!r})

def module(package, filename):
    spec = importlib.util.spec_from_file_location(package, ROOT / package / "launch" / filename)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result

driver = module("eli_cs_robot_driver", "elite_control.launch.py")

def mock_driver(context):
    nodes = []
    def resolve(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))
    for node in driver.launch_setup(context):
        package = resolve(node.node_package)
        executable = resolve(node.node_executable)
        if package == "controller_manager" and executable == "ros2_control_node":
            nodes.append(node)
        elif package == "robot_state_publisher":
            nodes.append(node)
        elif executable == "spawner":
            name = resolve(node.cmd[1])
            if name in ("scaled_joint_trajectory_controller", "joint_state_broadcaster",
                        "speed_scaling_state_broadcaster", "tcp_pose_broadcaster"):
                nodes.append(node)
    return nodes

def declarations(mod):
    return [x for x in mod.generate_launch_description().entities if isinstance(x, DeclareLaunchArgument)]

def generate_launch_description():
    return LaunchDescription([
        GroupAction([
            SetLaunchConfiguration("use_fake_hardware", "true"),
            SetLaunchConfiguration("launch_rviz", "false"),
            SetLaunchConfiguration("cs_type", "ls65"),
            *declarations(driver), OpaqueFunction(function=mock_driver),
        ]),
        GroupAction([
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
                ROOT / "eli_cs_robot_usr/launch/compliant_control_real.launch.py")),
                launch_arguments=dict(
                    cs_type="ls65", launch_rviz="false", launch_sensor="false",
                    launch_executor="false", launch_coordinator="false",
                    launch_target_publisher="false", admittance_enabled_on_startup="false",
                    auto_start_admittance="false",
                ).items()),
        ]),
    ])
"""


def main():
    os.environ["ROS_DOMAIN_ID"] = "187"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_LOG_DIR"] = "/tmp/eli-arm-jazzy-startup-logs"
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from moveit_msgs.srv import GetPositionIK
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, DurabilityPolicy

    with tempfile.TemporaryDirectory(prefix="eli-arm-startup-") as directory:
        launch = Path(directory) / "mock.launch.py"
        launch.write_text(LAUNCH.format(root=str(ROOT)))
        log_path = Path("/tmp/eli-arm-jazzy-startup.log")
        with log_path.open("w") as log:
            process = subprocess.Popen(
                ["ros2", "launch", str(launch)],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            rclpy.init()
            node = rclpy.create_node("arm_startup_check")
            controllers = node.create_client(
                ListControllers, "/controller_manager/list_controllers"
            )
            ik = node.create_client(GetPositionIK, "/compute_ik")
            required = {
                "scaled_joint_trajectory_controller",
                "joint_state_broadcaster",
                "tcp_pose_broadcaster",
                "speed_scaling_state_broadcaster",
            }
            states = {}
            descriptions = []
            description_sub = node.create_subscription(
                String,
                "/robot_description",
                lambda msg: descriptions.append(msg.data),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            try:
                deadline = time.monotonic() + 75
                while time.monotonic() < deadline and process.poll() is None:
                    if controllers.wait_for_service(timeout_sec=0.5):
                        future = controllers.call_async(ListControllers.Request())
                        rclpy.spin_until_future_complete(node, future, timeout_sec=2)
                        if future.done() and future.result():
                            states = {
                                c.name: c.state for c in future.result().controller
                            }
                    if (
                        all(states.get(name) == "active" for name in required)
                        and ik.service_is_ready()
                        and "You can start planning now!" in log_path.read_text()
                    ):
                        assert (
                            description_sub.get_publisher_count() == 1
                        ), "Competing robot description publishers"
                        assert descriptions and all(
                            "mock_components/GenericSystem" in msg
                            for msg in descriptions
                        )
                        print(
                            "PASS: four controllers active; MoveIt IK ready; only mock driver publishes robot description"
                        )
                        return
                    time.sleep(0.5)
                raise AssertionError(
                    f"Startup failed: controllers={states}, IK={ik.service_is_ready()}; see {log_path}"
                )
            finally:
                node.destroy_node()
                rclpy.shutdown()
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()


if __name__ == "__main__":
    main()
