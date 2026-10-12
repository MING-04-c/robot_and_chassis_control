#!/usr/bin/env python3
"""ROS integration smoke test. Uses synthetic images and an isolated ROS domain.

Run from the workspace root after building and sourcing the install:
  ROS_DOMAIN_ID=137 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 src/camer_diver/test/test_live_ros.py
"""
import json
from pathlib import Path
import signal
import subprocess
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParametersAtomically
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PolygonStamped


def main():
    package = Path(__file__).resolve().parents[1]
    workspace = package.parents[1]
    binary = workspace / "install/camer_diver/lib/camer_diver/line_tracker_node"
    process = subprocess.Popen([
        str(binary), "--ros-args", "-r", "__ns:=/line_integration_test",
        "-p", "show_window:=false", "-p", "frame_timeout_sec:=0.5",
    ])
    node = None
    rclpy.init()
    try:
        node = Node("line_integration_probe")
        bridge = CvBridge()
        status, debug, path, estimated = {}, [], [], []
        node.create_subscription(String, "/line_integration_test/line/status",
                                 lambda m: status.update(json.loads(m.data)), 10)
        node.create_subscription(Image, "/line_integration_test/line/debug_image",
                                 lambda m: debug.__setitem__(slice(None), [m]), qos_profile_sensor_data)
        node.create_subscription(PolygonStamped, "/line_integration_test/line/centerline",
                                 lambda m: path.__setitem__(slice(None), [m]), qos_profile_sensor_data)
        node.create_subscription(PolygonStamped, "/line_integration_test/line/estimated_centerline",
                                 lambda m: estimated.__setitem__(slice(None), [m]), qos_profile_sensor_data)
        publisher = node.create_publisher(Image, "/line_integration_test/image_raw", qos_profile_sensor_data)
        client = node.create_client(SetParametersAtomically,
                                    "/line_integration_test/line_tracker/set_parameters_atomically")

        def pump(predicate, image=None, seconds=6):
            deadline, next_publish = time.monotonic() + seconds, 0
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise AssertionError(f"Tracker unexpectedly exited: {process.returncode}")
                if image is not None and time.monotonic() >= next_publish:
                    message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.header.frame_id = "integration_synthetic_test"
                    publisher.publish(message)
                    next_publish = time.monotonic() + 0.05
                rclpy.spin_once(node, timeout_sec=0.02)
                if predicate():
                    return
            raise AssertionError(f"Timeout; latest state: {status}")

        pump(lambda: status.get("state") == "NO_FRAME")
        image = np.full((480, 640, 3), 230, np.uint8)
        cv2.rectangle(image, (345, 0), (375, 479), (220, 80, 20), -1)
        pump(lambda: status.get("state") == "TRACKING" and
             status.get("error_normalized", 0) > 0.1, image)
        pump(lambda: bool(debug) and debug[0].width == 1280 and
             debug[0].height == 1024 and bool(path) and len(path[0].polygon.points) > 0, image)
        assert client.wait_for_service(timeout_sec=2)

        def set_hue(low, high):
            request = SetParametersAtomically.Request()
            request.parameters = [Parameter("hue_low", value=low).to_parameter_msg(),
                                  Parameter("hue_high", value=high).to_parameter_msg()]
            future = client.call_async(request)
            pump(future.done, image)
            return future.result().result

        assert not set_hue(150, 100).successful
        assert set_hue(0, 10).successful
        pump(lambda: status.get("lost") is True, image)
        assert set_hue(95, 135).successful
        pump(lambda: status.get("state") == "TRACKING", image)
        pale = image.copy()
        cv2.rectangle(pale, (345, 240), (375, 249), (250, 240, 232), -1)
        pump(lambda: status.get("state") == "TRACKING" and status.get("lost") is False, pale)
        occluded = image.copy()
        cv2.rectangle(occluded, (335, 180), (385, 205), (15, 15, 15), -1)
        pump(lambda: status.get("state") == "OCCLUDED" and
             status.get("occlusion_suspected") is True and
             status.get("target_estimated") is True and status.get("inferred_rows", 0) > 0,
             occluded)
        assert status.get("joins") and any(not j["rejection"] for j in status["joins"])
        assert sum(c["retained"] for c in status["candidates"]) >= 2
        pump(lambda: bool(estimated) and len(estimated[0].polygon.points) > 0 and
             all(180 <= p.y <= 205 for p in estimated[0].polygon.points) and
             bool(path) and len(path[0].polygon.points) > 0 and
             all(not (180 <= p.y <= 205) for p in path[0].polygon.points), occluded)
        horizontal = np.full((480, 640, 3), 230, np.uint8)
        cv2.rectangle(horizontal, (100, 290), (439, 314), (220, 80, 20), -1)
        pump(lambda: status.get("state") == "HORIZONTAL" and
             status.get("orientation") == "horizontal" and
             abs(status.get("target_x", 0) - 270) <= 6 and
             len(status.get("candidates", [])) > 0, horizontal)
        assert status["selected_candidate"] > 0
        partial = np.full((480, 640, 3), 230, np.uint8)
        cv2.rectangle(partial, (95, 40), (125, 150), (220, 80, 20), -1)
        pump(lambda: status.get("state") == "PARTIAL" and
             status.get("bottom_connected") is False and
             status.get("target_fallback") is True and
             status.get("error_pixels") is not None and status["error_pixels"] < -200, partial)
        pump(lambda: bool(path) and len(path[0].polygon.points) >= 4 and
             all(95 <= p.x <= 125 and 40 <= p.y <= 150 for p in path[0].polygon.points), partial)
        empty = np.full((480, 640, 3), 230, np.uint8)
        pump(lambda: status.get("state") == "LOST" and status.get("target_x") is None, empty)
        assert status["lost_reason"] == "no_blue_pixels"
        pump(lambda: bool(path) and not path[0].polygon.points and
             bool(estimated) and not estimated[0].polygon.points, empty)
        pump(lambda: status.get("state") == "TRACKING" and
             status.get("lost") is False, image)
        patch = empty.copy()
        cv2.rectangle(patch, (295, 270), (344, 324), (60, 20, 5), -1)
        pump(lambda: status.get("lost") is True and
             status.get("lost_reason") == "shape_rejected" and
             status.get("error_pixels") is None, patch)
        assert any(c["rejection"] == "aspect" and c["aspect"] > 0
                   for c in status["candidates"])
        pump(lambda: bool(path) and not path[0].polygon.points, patch)
        pump(lambda: status.get("state") == "TRACKING", image)
        grey = empty.copy()
        cv2.rectangle(grey, (100, 70), (250, 300), (130, 126, 120), -1)
        pump(lambda: status.get("state") == "LOST" and
             status.get("lost_reason") == "no_blue_pixels" and
             status.get("target_x") is None, grey)
        pump(lambda: status.get("state") == "STALE_FRAME", seconds=3)
        assert status["lost"] and status["error_pixels"] is None
        pump(lambda: bool(estimated) and not estimated[0].polygon.points, seconds=2)
        print("PASS: NO_FRAME -> TRACKING -> OCCLUDED -> PARTIAL -> noise rejected -> STALE_FRAME")
        print("PASS: real ROS Image dashboard, pixel path, null error when lost, empty path on loss")
    finally:
        process.send_signal(signal.SIGINT) if process.poll() is None else None
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
