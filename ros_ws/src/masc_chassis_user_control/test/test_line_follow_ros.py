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

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from line_follow_node import LineFollowNode  # noqa: E402
from test_line_follow_control import observation  # noqa: E402


def main():
    rclpy.init(args=['--ros-args',
                    '-p', 'cmd_vel_topic:=/line_follow_test/cmd_vel',
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
    commands = []
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
                message.polygon.points = [Point32(x=320.0, y=400.0)]
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
        assert commands[-1].linear.x > 0.0, 'fresh valid line did not start motion'
        assert commands[-1].angular.z < 0.0, 'right line must turn clockwise'
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
        result = controller.set_parameters([Parameter('forward_speed', value=-0.1)])[0]
        assert not result.successful
        assert controller.config.forward_speed == 0.05
        controller.set_parameters([Parameter('forward_speed', value=0.08),
                                   Parameter('enabled', value=True)])
        state['error_normalized'] = 0.0
        pump(1.0)
        assert abs(commands[-1].linear.x - 0.08) < 1e-6
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
