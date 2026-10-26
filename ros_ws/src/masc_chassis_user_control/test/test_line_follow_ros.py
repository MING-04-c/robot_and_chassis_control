#!/usr/bin/env python3
"""ROS integration test with fake camera/chassis and an isolated command topic."""

import json
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
                message.polygon.points = [Point32(x=x, y=y) for y in (100.0, 200.0, 300.0, 420.0)]
                path.publish(message)
                line.publish(String(data=json.dumps(state)))
            for _ in range(4):
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
        pump(1.0)
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
        controller.set_parameters([Parameter('forward_speed', value=0.0)])
        pump(0.2)
        stopped()
        print('PASS: ROS direction, lost line, timeout, ESTOP, disconnect, stale stamp, '
              'synthetic rejection, disable and live speed adjustment')
    finally:
        controller.stop()
        executor.shutdown()
        controller.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
