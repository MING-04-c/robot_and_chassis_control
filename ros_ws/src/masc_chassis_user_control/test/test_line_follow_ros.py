#!/usr/bin/env python3
"""ROS integration test with fake camera/chassis and an isolated command topic."""

import json
import cv2
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, Twist
from masc_chassis_msgs.msg import ChassisStatus, CommStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from line_follow_node import LineFollowNode  # noqa: E402
from test_line_follow_control import observation  # noqa: E402


def main():
    rclpy.init(args=['--ros-args',
                    '-p', 'cmd_vel_topic:=/line_follow_test/cmd_vel',
                    '-p', 'enabled:=false',
                    '-p', 'stop_marker_mask_topic:=/line_follow_test/mask',
                    '-p', 'forward_speed:=0.05',
                    '-p', 'image_forward_direction:=up',
                    '-p', 'vehicle_reference_y_ratio:=1.05',
                    '-p', 'steering_sign:=-1.0',
                    '-p', 'line_status_topic:=/line_follow_test/line',
                    '-p', 'centerline_topic:=/line_follow_test/path',
                    '-p', 'comm_status_topic:=/line_follow_test/comm',
                    '-p', 'chassis_status_topic:=/line_follow_test/chassis'])
    controller = LineFollowNode()
    harness = Node('line_follow_test_harness')
    from std_msgs.msg import String
    line = harness.create_publisher(String, '/line_follow_test/line', 10)
    path = harness.create_publisher(PolygonStamped, '/line_follow_test/path', qos_profile_sensor_data)
    comm = harness.create_publisher(CommStatus, '/line_follow_test/comm', 10)
    chassis = harness.create_publisher(ChassisStatus, '/line_follow_test/chassis', 10)
    ready_client = harness.create_client(Trigger, '/line_follow_controller/ready')
    commands = []
    debug_images = []
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import numpy as np
    debug = harness.create_publisher(Image, "/camera/line/debug_image", qos_profile_sensor_data)
    mask_pub = harness.create_publisher(Image, '/line_follow_test/mask', qos_profile_sensor_data)
    from std_msgs.msg import Empty
    resume_pub = harness.create_publisher(Empty, '/line_follow/resume', 10)
    harness.create_subscription(Image, "/line_follow/debug_image", debug_images.append,
                                 qos_profile_sensor_data)
    harness.create_subscription(Twist, '/line_follow_test/cmd_vel', commands.append, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(harness)
    state = observation(error_normalized=0.4)
    comm_value = CommStatus(comm_state=CommStatus.COMM_STATE_CONNECTED)
    chassis_value = ChassisStatus(chassis_state=ChassisStatus.CHASSIS_STATE_COMM_CONTROL)
    frame_id = 'camera_optical_frame'
    old_stamp = False
    path_ranges = [(100, 421)]
    custom_points = None
    marker_y = None

    def pump(seconds, visual=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            comm.publish(comm_value)
            chassis.publish(chassis_value)
            if visual:
                message = PolygonStamped()
                message.header.stamp = harness.get_clock().now().to_msg()
                if old_stamp:
                    message.header.stamp.sec -= 2
                message.header.frame_id = frame_id
                x = 319.5 + state['error_normalized'] * 319.5
                message.polygon.points = (
                    [Point32(x=float(px), y=float(py)) for px, py in custom_points]
                    if custom_points is not None else
                    [Point32(x=x, y=float(y)) for start, stop in path_ranges for y in range(start, stop, 6)])
                path.publish(message)
                mask = np.zeros((480, 640), dtype=np.uint8)
                polygon = np.array([(p.x, p.y) for p in message.polygon.points], dtype=np.int32)
                # 每段单独画，保留用于验证续接的真实缺口。
                if custom_points is not None:
                    cv2.polylines(mask, [polygon], False, 255, 16)
                else:
                    for start, stop in path_ranges:
                        cv2.line(mask, (int(x), start), (int(x), stop-1), 255, 16)
                if marker_y is not None:
                    cv2.line(mask, (250, int(marker_y)), (390, int(marker_y)), 255, 16)
                mask_message = CvBridge().cv2_to_imgmsg(mask, encoding='mono8')
                mask_message.header = message.header
                mask_pub.publish(mask_message)
                line.publish(String(data=json.dumps(state)))
            for _ in range(7):
                executor.spin_once(timeout_sec=0.01)

    def stopped():
        assert commands, 'no command received'
        assert commands[-1].linear.x == 0.0 and commands[-1].angular.z == 0.0

    try:
        # Wait for endpoint discovery, then allow valid observations to converge.
        pump(2.0)
        stopped()
        assert ready_client.service_is_ready()
        future = ready_client.call_async(Trigger.Request())
        pump(0.2)
        assert future.done() and future.result().success, 'disabled follower failed readiness check'
        assert not controller.enabled
        stopped()  # Readiness must never enable movement.
        comm_value.comm_state = CommStatus.COMM_STATE_DISCONNECTED
        pump(0.2)
        assert not controller.on_ready(Trigger.Request(), Trigger.Response()).success
        comm_value.comm_state = CommStatus.COMM_STATE_CONNECTED
        pump(0.3)
        old_stamp = True
        pump(0.2)
        assert not controller.on_ready(Trigger.Request(), Trigger.Response()).success
        old_stamp = False
        pump(0.3)
        controller.set_parameters([Parameter('enabled', value=True)])
        pump(0.5)
        assert commands[-1].linear.x > 0.0, 'fresh valid line did not start motion'
        assert commands[-1].angular.z < 0.0, 'right line must turn clockwise'
        state['orientation'] = 'horizontal'
        pump(0.5)
        assert commands[-1].linear.x > 0.0, 'valid curved candidate was incorrectly stopped'
        state['orientation'] = 'vertical'
        state['lost'] = True
        pump(0.2)
        stopped()
        state['lost'] = False
        pump(0.5)
        assert commands[-1].linear.x > 0.0
        pump(0.8, visual=False)
        stopped()
        pump(0.5)
        assert commands[-1].linear.x > 0.0
        chassis_value.chassis_state = ChassisStatus.CHASSIS_STATE_ESTOP
        pump(0.2)
        stopped()
        chassis_value.chassis_state = ChassisStatus.CHASSIS_STATE_COMM_CONTROL
        comm_value.comm_state = CommStatus.COMM_STATE_DISCONNECTED
        pump(0.2)
        stopped()
        comm_value.comm_state = CommStatus.COMM_STATE_CONNECTED
        old_stamp = True
        pump(0.2)
        stopped()
        old_stamp = False
        frame_id = 'synthetic_demo'
        pump(0.2)
        stopped()
        frame_id = 'camera_optical_frame'
        pump(0.5)
        assert commands[-1].linear.x > 0.0
        result = controller.set_parameters([Parameter('enabled', value=False)])[0]
        assert result.successful
        pump(0.2)
        stopped()
        assert controller.geometry is not None, 'paused preview must retain current geometry'
        image = CvBridge().cv2_to_imgmsg(np.zeros((1024, 1280, 3), dtype=np.uint8), encoding='bgr8')
        image.header.stamp = harness.get_clock().now().to_msg()
        debug.publish(image)
        pump(0.2)
        assert debug_images and debug_images[-1].width == 1280
        overlay = CvBridge().imgmsg_to_cv2(debug_images[-1], desired_encoding='bgr8')
        assert np.count_nonzero(overlay) > 0, 'control reference overlay was not drawn'
        stopped()
        result = controller.set_parameters([Parameter('forward_speed', value=-0.1)])[0]
        assert not result.successful
        assert controller.config.forward_speed == 0.05
        controller.set_parameters([Parameter('forward_speed', value=0.08),
                                   Parameter('enabled', value=True)])
        state['error_normalized'] = 0.0
        # 按配置的加速度等待爬升完成，另留重新确认有效图像的时间。
        pump(0.08 / controller.config.linear_acceleration + 0.3)
        assert abs(commands[-1].linear.x - 0.08) < 1e-6
        controller.set_parameters([Parameter('image_forward_direction', value='down'),
                                   Parameter('vehicle_reference_y_ratio', value=-0.05)])
        state['error_normalized'] = 0.4
        state['bottom_connected'] = False
        pump(0.5)
        assert commands[-1].linear.x > 0.0
        assert commands[-1].angular.z > 0.0, 'image down must reverse the image/body turn mapping'
        controller.set_parameters([Parameter('require_bottom_connected', value=True)])
        pump(0.2)
        stopped()
        controller.set_parameters([Parameter('require_bottom_connected', value=False)])
        pump(0.3)
        assert commands[-1].linear.x > 0.0
        # 有远端缺口时只走近段；近段不足停车，新帧变清楚后经连续有效帧恢复。
        path_ranges = [(120, 270), (430, 470)]
        pump(0.5)
        assert commands[-1].linear.x > 0.0
        assert controller.geometry['trusted_end_y'] < 270
        assert controller.geometry['target_y'] < 270
        path_ranges = [(120, 144), (430, 470)]
        pump(0.3)
        stopped()
        path_ranges = [(120, 470)]
        state['error_normalized'] = 0.0
        controller.set_parameters([Parameter('lookahead_distance_ratio', value=0.6)])
        pump(controller.config.forward_speed / controller.config.linear_acceleration + 0.3)
        continuous_speed = commands[-1].linear.x
        assert abs(continuous_speed - controller.config.forward_speed) < 1e-6
        assert controller.geometry['trusted_end_y'] > 450
        # 上游疑似遮挡/估计/回退都不能对已验证的同一续接缺口重复减速。
        state.update(occlusion_suspected=True, target_estimated=True, target_fallback=True)
        path_ranges = [(120, 220), (260, 470)]
        pump(0.4)
        assert len(controller.geometry['bridged_gaps']) == 1
        assert abs(commands[-1].linear.x - continuous_speed) < 1e-6
        path_ranges = [(120, 220), (320, 470)]
        pump(0.4)
        assert len(controller.geometry['bridged_gaps']) == 1
        assert controller.geometry['target_gap_interpolated']
        assert abs(commands[-1].linear.x - continuous_speed) < 1e-6
        # 几何不支持的缺口仍保留上游不确定性限速。
        path_ranges = [(120, 270), (430, 470)]
        pump(0.7)
        assert not controller.geometry['bridged_gaps']
        assert commands[-1].linear.x < continuous_speed
        state.update(occlusion_suspected=False, target_estimated=False, target_fallback=False)
        path_ranges = [(120, 180)]
        pump(0.5)
        assert commands[-1].linear.x > 0.0, '短近端实测段仍应允许低速推进'
        path_ranges = [(120, 156)]
        pump(0.5)
        assert commands[-1].linear.x > 0.0, '短于原门槛的可见近段也应低速推进'
        path_ranges = [(120, 144)]
        pump(0.3)
        stopped()
        assert controller.last_blocked_reason == 'insufficient_forward_span'
        custom_points = [(319.5 + 0.012 * (y - 120) ** 2, y) for y in range(120, 270, 6)]
        pump(1.2)
        assert controller.geometry['curve_strength'] > 0.9
        assert controller.config.max_angular_speed < commands[-1].angular.z <= controller.config.curve_max_angular_speed
        custom_points = [(319.5 - 0.012 * (y - 120) ** 2, y) for y in range(120, 270, 6)]
        pump(2*controller.config.curve_max_angular_speed/controller.config.curve_angular_acceleration + .7)
        assert controller.geometry['curve_strength'] > 0.9
        assert -controller.config.curve_max_angular_speed <= commands[-1].angular.z < -controller.config.max_angular_speed
        custom_points = None
        path_ranges = [(120, 470)]
        resume_pub.publish(Empty())  # 提前的消息不授权未来站点自动通过。
        for marker_y in (400, 370, 340, 310, 280, 255, 242):
            pump(.2)
        assert controller.marker_gate.state(controller.config) == 'stopped_intermediate'
        pump(.3)
        stopped()
        resume_pub.publish(Empty())
        pump(.3)
        assert commands[-1].linear.x > 0
        assert not controller.marker_gate.stopped
        for marker_y in (220, 200, 180):
            pump(.15)
        for marker_y in (400, 370, 340, 310, 280, 255, 240):
            path_ranges = [(120, int(marker_y))]
            pump(.2)
        assert controller.marker_gate.state(controller.config) == 'stopped_final'
        stopped()
        resume_pub.publish(Empty())
        pump(.2)
        stopped()
        controller.set_parameters([Parameter('enabled', value=False)])
        marker_y = None
        custom_points = None
        path_ranges = [(120, 470)]
        pump(0.3)
        stopped()
        assert controller.reason == 'disabled' and controller.input_reason == 'tracking'
        assert controller.last_blocked_reason == 'insufficient_forward_span'
        controller.set_parameters([Parameter('forward_speed', value=0.0)])
        pump(0.2)
        stopped()
        print('PASS: ROS direction, lost line, timeout, ESTOP, disconnect, stale stamp, '
              'synthetic rejection, disable, live speed adjustment, trusted prefix, curve priority, intermediate resume and final stop markers')
    finally:
        controller.stop()
        executor.shutdown()
        controller.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
