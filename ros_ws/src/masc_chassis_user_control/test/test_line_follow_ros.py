#!/usr/bin/env python3
"""ROS integration test with fake camera/chassis and an isolated command topic."""

import json
import os
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
    rclpy.init(args=['--ros-args', '-p', 'station_log_file:=/tmp/line-test-stations-'+str(os.getpid())+'.json',
                    '-p', 'reverse_mode:=camera',
                    '-p', 'record_route:=false',
                    '-p', 'forward_ignore_start_marker:=false',
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
    # 故意交错两个源图，状态/中线/掩膜只有同一时间戳才能一起提交。
    from types import SimpleNamespace
    committed = []
    probe = SimpleNamespace(frame_sync=False, frame_cache={}, committed_stamp=-1, frame_fault=None,
        input_timeout=.5, config=controller.config, get_clock=harness.get_clock,
        stop=lambda: committed.append('stop'),
        accept_path=lambda message: committed.append('path'),
        accept_marker_mask=lambda message: committed.append('mask'),
        accept_status=lambda message: committed.append('status'))
    stamp = harness.get_clock().now().to_msg()
    first_path, first_mask, other_mask = PolygonStamped(), Image(), Image()
    first_path.header.stamp = first_mask.header.stamp = stamp
    other_mask.header.stamp = harness.get_clock().now().to_msg()
    info = dict(source_stamp_sec=stamp.sec, source_stamp_nanosec=stamp.nanosec)
    LineFollowNode.stage_frame(probe, 'path', first_path)
    LineFollowNode.stage_frame(probe, 'mask', other_mask)
    LineFollowNode.stage_frame(probe, 'status', info)
    assert not committed, 'mixed-frame observation was committed'
    LineFollowNode.stage_frame(probe, 'mask', first_mask)
    assert committed == ['path', 'mask', 'status']
    LineFollowNode.stage_frame(probe, 'status', info)
    assert committed == ['path', 'mask', 'status'], 'same source frame was counted twice'
    older = dict(info, source_stamp_sec=stamp.sec-2)
    LineFollowNode.stage_frame(probe, 'status', older)
    assert probe.frame_fault is None, 'late old frame poisoned a newer committed observation'
    state = observation(error_normalized=0.4)
    comm_value = CommStatus(comm_state=CommStatus.COMM_STATE_CONNECTED)
    chassis_value = ChassisStatus(chassis_state=ChassisStatus.CHASSIS_STATE_COMM_CONTROL)
    frame_id = 'camera_optical_frame'
    old_stamp = False
    path_ranges = [(100, 421)]
    custom_points = None
    marker_y = None
    departure_case = False

    def pump(seconds, visual=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            comm.publish(comm_value)
            chassis_value.header.stamp = harness.get_clock().now().to_msg()
            if departure_case:
                controller.latest_odom = dict(pose=(0., 0., 0.), receipt=time.monotonic(),
                    stamp=harness.get_clock().now().nanoseconds)
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
                current = dict(state, source_stamp_sec=message.header.stamp.sec,
                               source_stamp_nanosec=message.header.stamp.nanosec)
                line.publish(String(data=json.dumps(current)))
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
        departure_case = True
        chassis_value.dock_charging = True
        pump(.4)
        assert controller.on_ready(None, Trigger.Response()).success
        assert controller.departure.state == 'idle'
        controller.set_parameters([Parameter('enabled', value=True)])
        pump(.8)
        assert controller.departure.state == 'active'
        assert 0. < commands[-1].linear.x <= .02, '充电离站应只限速而不阻止前进'
        chassis_value.obstacle_status_flags = 1
        pump(.2)
        stopped()
        chassis_value.obstacle_status_flags = 0
        pump(.4)
        assert 0. < commands[-1].linear.x <= .02
        chassis_value.dock_charging = False
        pump(.7)
        assert controller.departure.state == 'complete' and commands[-1].linear.x > .02
        controller.set_parameters([Parameter('enabled', value=False)])
        departure_case = False
        pump(.1)
        stopped()
        comm_value.comm_state = CommStatus.COMM_STATE_DISCONNECTED
        pump(0.2)
        assert not controller.on_ready(Trigger.Request(), Trigger.Response()).success
        comm_value.comm_state = CommStatus.COMM_STATE_CONNECTED
        pump(0.3)
        old_stamp = True
        pump(0.65)  # 旧乱序帧不覆盖新帧；持续没有新帧则按原0.5秒有效期停车。
        assert not controller.on_ready(Trigger.Request(), Trigger.Response()).success
        old_stamp = False
        pump(0.3)
        # 真正的自动离站：启用时处在横线上，不发送resume；下一根仍停车。
        original_error = state['error_normalized']
        state['error_normalized'], marker_y = 0., 110
        controller.set_parameters([Parameter('forward_ignore_start_marker', value=True)])
        pump(.3)
        controller.set_parameters([Parameter('enabled', value=True)])
        pump(.6)
        assert controller.marker_gate.startup_ignored and controller.previous_v > 0.
        for marker_y in range(130, 341, 20):
            pump(.12)
        marker_y = None
        pump(.2)
        for marker_y in (130, 160, 190, 220, 240):
            pump(.15)
        pump(.6)
        assert controller.marker_gate.arrived and commands[-1].linear.x == 0.
        controller.set_parameters([Parameter('enabled', value=False),
                                   Parameter('forward_ignore_start_marker', value=False)])
        state['error_normalized'], marker_y = original_error, None
        pump(.3)
        controller.set_parameters([Parameter('enabled', value=True)])
        pump(0.5)
        assert commands[-1].linear.x > 0.0, 'fresh valid line did not start motion'
        assert controller.frame_sync
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
        pump(0.65)  # 旧乱序帧不覆盖新帧；持续没有新帧则按原0.5秒有效期停车。
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
        # 等到3帧重新确认及对应速度消息，而不是依赖机器负载下恰好0.3秒完成。
        deadline = time.monotonic()+1.5
        while time.monotonic() < deadline:
            pump(.1)
            if commands[-1].linear.x > 0.0:
                break
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
        pump(1.)
        # 小幅朝向噪声交替变号不应持续让轮子左右换向。
        for frame in range(8):
            slope = .08 if frame % 2 else -.08
            custom_points = [(319.5+slope*(y-120), y) for y in range(120, 470, 6)]
            pump(.08)
            assert abs(commands[-1].angular.z) < 1e-6
        # 小幅但持续的同侧偏移要产生温和纠偏，而不能被防摆动门槛永久吞掉。
        custom_points = None
        for offset in (.065, -.065):
            state['error_normalized'] = offset
            pump(.8)
            assert controller.steering_guard.center_ready
            assert commands[-1].angular.z*offset > 0.
            assert abs(commands[-1].angular.z) < .04
            assert commands[-1].linear.x > 0.
        state['error_normalized'] = 0.
        pump(.8)
        assert not controller.steering_guard.center_ready
        assert abs(commands[-1].angular.z) < 1e-6
        path_ranges = [(120, 144)]
        pump(.08)
        stopped()  # 无效段期间仍立即零速，不靠历史预测继续开。
        path_ranges = [(120, 470)]
        pump(.08)
        assert controller.good_frames == controller.reacquire_frames
        # 恢复只给一张新图：检查本次控制周期已发布的命令，避免订阅队列中
        # 尚未处理的上一条零速消息造成假失败；常规运动/停车仍检查接收端。
        assert controller.previous_v > 0., 'short supported-frame recovery stalled for three extra frames'
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
        # 更远处断线不影响已被实测近段覆盖的前瞻点。
        path_ranges = [(120, 270), (430, 470)]
        pump(0.4)
        assert not controller.geometry['bridged_gaps']
        assert controller.geometry['lookahead_supported']
        assert abs(commands[-1].linear.x - continuous_speed) < 1e-6
        # 前瞻点缺少实测支撑时仍限速。
        path_ranges = [(120, 240), (430, 470)]
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
        ratio = commands[-1].angular.z/commands[-1].linear.x
        assert controller.config.max_angular_speed/controller.config.forward_speed < ratio
        assert ratio <= controller.config.curve_max_angular_speed/controller.config.forward_speed+1e-6
        custom_points = [(319.5 - 0.012 * (y - 120) ** 2, y) for y in range(120, 270, 6)]
        pump(2*controller.config.curve_max_angular_speed/controller.config.curve_angular_acceleration + .7)
        assert controller.geometry['curve_strength'] > 0.9
        ratio = commands[-1].angular.z/commands[-1].linear.x
        assert -controller.config.curve_max_angular_speed/controller.config.forward_speed-1e-6 <= ratio
        assert ratio < -controller.config.max_angular_speed/controller.config.forward_speed
        custom_points = None
        path_ranges = [(120, 470)]
        resume_pub.publish(Empty())  # 提前的消息不授权未来站点自动通过。
        marker_y = 400
        pump(.4)
        marker_y = None
        mark = len(commands)
        pump(.2)  # 新速度下短暂漏检仍可有效爬行，但不能超过5mm总行程。
        assert controller.reason == 'stop_marker_missing_approaching', controller.reason
        assert commands[mark:] and all(0. < cmd.linear.x <= controller.config.stop_marker_creep_speed+1e-6
                                      for cmd in commands[mark+2:]), [round(cmd.linear.x, 5) for cmd in commands[mark:]]
        pump(.3)
        assert commands[-1].linear.x == 0. and controller.reason == 'stop_marker_missing'
        assert controller.marker_gate.dropout_commanded_distance <= .005+1e-9
        marker_y = 400
        pump(.3)
        marker_y = None
        pump(.7)
        assert commands[-1].linear.x == 0. and controller.reason == 'stop_marker_missing'
        marker_y = 400
        pump(.3)
        for marker_y in (400, 370, 340, 310, 280, 255, 242):
            pump(.2)
        pump(.8)
        assert controller.marker_gate.state(controller.config) == 'stopped_intermediate'
        pump(.3)
        stopped()
        marker_y = None
        pump(.3)
        assert controller.reason == 'stop_marker_intermediate' and controller.marker_gate.missing
        resume_pub.publish(Empty())
        pump(.2)
        stopped()
        pump(.7, visual=False)
        assert controller.reason == 'stop_marker_intermediate'
        stopped()
        first_command = len(commands)
        marker_y = 232  # 轻微越线8像素仍到位，不能输出倒车命令。
        pump(.8)
        assert controller.marker_gate.state(controller.config) == 'stopped_intermediate'
        assert all(cmd.linear.x >= 0 for cmd in commands[first_command:])
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
        pump(.8)
        assert controller.marker_gate.state(controller.config) == 'stopped_final'
        stopped()
        resume_pub.publish(Empty())
        pump(.2)
        stopped()
        saved_blocked_reason = controller.last_blocked_reason
        controller.set_parameters([Parameter('enabled', value=False)])
        marker_y = None
        custom_points = None
        path_ranges = [(120, 470)]
        pump(0.3)
        stopped()
        assert controller.reason == 'disabled' and controller.input_reason == 'tracking'
        assert controller.last_blocked_reason == saved_blocked_reason
        results = controller.set_parameters([Parameter('drive_direction', value='reverse'),
                                             Parameter('reverse_steering_sign', value=-1.),
                                             Parameter('forward_speed', value=.025)])
        assert all(result.successful for result in results)
        state['error_normalized'] = 0.
        marker_y = None
        controller.set_parameters([Parameter('enabled', value=True)])
        pump(.25)  # 最初空帧不应在尚未走出离站范围时关闭启动识别。
        marker_y = 287  # 复现起点已在后侧约47px，超出原越线容差。
        pump(.6)
        assert controller.marker_gate.startup_ignored
        assert commands[-1].linear.x < 0.
        for marker_y in (300, 320):
            pump(.2)
        assert controller.marker_gate.startup_ignored  # 仍在画面内，保留出发线身份。
        assert controller.marker_gate.ledger.counts['encountered'] == 1
        assert controller.marker_gate.ledger.counts['passed'] == 1
        assert controller.marker_gate.station_id == 0
        marker_y = None
        state['error_normalized'] = .2
        pump(.8)
        assert commands[-1].linear.x < 0. and commands[-1].angular.z > 0., 'reverse device correction must flip yaw only'
        # 启用时禁止直接换向；后退不是把图像、转向和速度一起反号。
        result = controller.set_parameters([Parameter('drive_direction', value='forward')])[0]
        assert not result.successful and controller.config.drive_direction == 'reverse'
        result = controller.set_parameters([Parameter('reverse_steering_sign', value=1.)])[0]
        assert not result.successful and controller.config.reverse_steering_sign == -1.
        pump(.7, visual=False)
        stopped()
        state['error_normalized'] = 0.
        marker_y = None
        pump(.4)
        assert commands[-1].linear.x < 0.
        for marker_y in (150, 180, 210, 230, 240):
            pump(.2)
        pump(.6)
        assert controller.marker_gate.state(controller.config) == 'stopped_intermediate'
        stopped()
        resume_pub.publish(Empty())
        pump(.3)
        assert commands[-1].linear.x < 0.
        controller.set_parameters([Parameter('enabled', value=False)])
        pump(.2)
        stopped()
        controller.set_parameters([Parameter('forward_speed', value=0.0)])
        pump(0.2)
        stopped()
        print('PASS: ROS direction, lost line, timeout, ESTOP, disconnect, stale stamp, '
              'synthetic rejection, disable, live speed adjustment, trusted prefix, curve priority, '
              'intermediate resume, final stop markers, front-camera reverse and direction-change rejection')
    finally:
        controller.stop()
        executor.shutdown()
        controller.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()


def route_pause_snapshot_checks():
    """不连接硬件，验证停车快照、恢复前失效和ROS关闭后的快照保存。"""
    import tempfile
    from types import SimpleNamespace
    from unittest.mock import patch
    from recorded_route import RouteConfig, RouteRecorder, load_route
    with tempfile.TemporaryDirectory() as directory:
        recorder = RouteRecorder(RouteConfig(), 'odom', 'base_link')
        for index in range(21):
            recorder.add((index*.01, 0., 0.))
        logger = SimpleNamespace(info=lambda message: None, error=lambda message: None)
        node = SimpleNamespace(recorder=recorder, latest_odom={'v': 0., 'vy': 0., 'w': 0.},
                               odom_reason=lambda now: None, odom_fault=None, enabled=True,
                               config=SimpleNamespace(stop_marker_enabled=True),
                               marker_gate=SimpleNamespace(arrived=True),
                               route_file=str(Path(directory)/'route.json'), last_route_save=0.,
                               get_logger=lambda: logger)
        with patch('line_follow_node.rclpy.ok', return_value=True):
            LineFollowNode.flush_route(node)
            assert load_route(node.route_file)['valid'], 'enabled station pause was not saved'
            node.marker_gate.arrived = False
            LineFollowNode.flush_route(node)
            assert not json.loads(Path(node.route_file).read_text())['valid']
            node.marker_gate.arrived = True
            node.latest_odom['v'] = .03
            LineFollowNode.flush_route(node)
            assert not json.loads(Path(node.route_file).read_text())['valid'], 'moving vehicle finalized route'
            node.latest_odom['v'] = 0.
            LineFollowNode.flush_route(node)
            assert load_route(node.route_file)['valid']
        with patch('line_follow_node.rclpy.ok', return_value=False):
            node.odom_reason = lambda now: (_ for _ in ()).throw(AssertionError('ROS clock queried after shutdown'))
            LineFollowNode.flush_route(node, finish=True)
            assert load_route(node.route_file)['valid'], 'shutdown erased a confirmed parked snapshot'
        print('PASS: paused enabled route snapshot, moving/resume invalidation and shutdown preservation')


def background_work_ros():
    """模拟三个慢后台任务，验证控制/里程计不中断，旧检测结果不刷新有效期。"""
    import tempfile
    from threading import Event
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from recorded_route import RouteRecorder, RouteConfig
    from line_follow_workers import LatestWorker
    # 一个任务执行时，积压任务只保留最后一个；保存顺序不能倒退。
    release, started = Event(), Event()
    seen = []
    def operation(value):
        seen.append(value)
        if value == 1:
            started.set()
            release.wait(1.)
        return value
    worker = LatestWorker(operation, 'latest-slot-test')
    try:
        worker.submit(1)
        assert started.wait(1.)
        worker.submit(2)
        ticket = worker.submit(3)
        release.set()
        worker.wait(ticket)
        assert seen == [1, 3], seen
    finally:
        release.set()
        worker.close()

    rclpy.init(args=['--ros-args', '-p', 'station_log_file:=/tmp/line-test-stations-'+str(os.getpid())+'.json', '-p', 'enabled:=false', '-p', 'dry_run:=true',
                    '-p', 'record_route:=false', '-p', 'odom_topic:=/background_test/odom'])
    node = LineFollowNode()
    harness = Node('background_work_test')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(harness)
    odom = harness.create_publisher(Odometry, '/background_test/odom', 10)
    commands = []
    harness.create_subscription(Twist, '/line_follow/cmd_vel',
                                lambda message: commands.append(time.monotonic()), 10)
    def pump(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            message = Odometry()
            message.header.stamp = harness.get_clock().now().to_msg()
            message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
            message.pose.pose.orientation.w = 1.
            odom.publish(message)
            executor.spin_once(timeout_sec=.003)
            time.sleep(.002)
    release = Event()
    try:
        pump(.4)
        entered = [Event() for _ in range(3)]
        originals = [node.marker_worker.function, node.preview_worker.function, node.route_writer.function]
        workers = [node.marker_worker, node.preview_worker, node.route_writer]
        for index, worker in enumerate(workers):
            def slow(value, index=index):
                entered[index].set()
                release.wait(2.)
                return originals[index](value)
            worker.function = slow
        import numpy as np
        from cv_bridge import CvBridge
        mask = CvBridge().cv2_to_imgmsg(np.zeros((480, 640), np.uint8), encoding='mono8')
        mask.header.stamp = harness.get_clock().now().to_msg()
        node.accept_marker_mask(mask)
        node.preview_worker.submit((None, None))  # 绘制任务错误也必须隔离在后台。
        with tempfile.TemporaryDirectory() as directory:
            recorder = RouteRecorder(RouteConfig(), 'odom', 'base_link')
            ticket = node.route_writer.submit((str(Path(directory)/'route.json'), recorder.document()))
            assert all(event.wait(.5) for event in entered)
            commands.clear()
            pump(.65)
            assert len(commands) >= 15, 'slow image/disk work blocked control timer'
            assert max(b-a for a, b in zip(commands, commands[1:])) < .1, 'control heartbeat stalled'
            assert time.monotonic()-node.latest_odom['receipt'] < .1
            assert node.odom_fault is None
            release.set()
            node.route_writer.wait(ticket)
            pump(.15)
            assert node.marker_time is None, 'expired detector result refreshed mask validity'
            assert node.preview_worker.take() is None  # tick已消费绘图失败，不影响控制。
        print('PASS: bounded latest work; slow detector/preview/disk do not block control or odometry; stale result rejected')
    finally:
        release.set()
        executor.shutdown()
        node.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()


def recorded_route_ros():
    """模拟前进记录、驱动归零后倒车、图像丢线、超时和到起点停车。"""
    import math
    import tempfile
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from recorded_route import load_route
    with tempfile.TemporaryDirectory() as directory:
        filename = str(Path(directory)/'last_route.json')
        rclpy.init(args=['--ros-args', '-p', 'station_log_file:=/tmp/line-test-stations-'+str(os.getpid())+'.json', '-p', 'enabled:=false',
                        '-p', 'route_camera_stop:=false',
                        '-p', 'route_file:='+filename,
                        '-p', 'cmd_vel_topic:=/route_test/cmd',
                        '-p', 'odom_topic:=/route_test/odom',
                        '-p', 'comm_status_topic:=/route_test/comm',
                        '-p', 'chassis_status_topic:=/route_test/chassis',
                        '-p', 'line_status_topic:=/route_test/line',
                        '-p', 'centerline_topic:=/route_test/path'])
        executor = SingleThreadedExecutor()
        harness = Node('recorded_route_harness')
        controller = LineFollowNode()
        executor.add_node(harness)
        executor.add_node(controller)
        odometry = harness.create_publisher(Odometry, '/route_test/odom', 10)
        comm = harness.create_publisher(CommStatus, '/route_test/comm', 10)
        chassis = harness.create_publisher(ChassisStatus, '/route_test/chassis', 10)
        lost = harness.create_publisher(String, '/route_test/line', 10)
        empty = harness.create_publisher(PolygonStamped, '/route_test/path', qos_profile_sensor_data)
        commands = []
        harness.create_subscription(Twist, '/route_test/cmd', commands.append, 10)
        def pump(x, seconds, publish_odom=True):
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                comm.publish(CommStatus(comm_state=CommStatus.COMM_STATE_CONNECTED))
                chassis.publish(ChassisStatus(chassis_state=ChassisStatus.CHASSIS_STATE_COMM_CONTROL))
                lost.publish(String(data=json.dumps({'lost': True, 'processing_over_budget': True})))
                empty.publish(PolygonStamped())
                if publish_odom:
                    message = Odometry()
                    message.header.stamp = harness.get_clock().now().to_msg()
                    message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
                    message.pose.pose.position.x = x
                    message.pose.pose.orientation.w = 1.
                    odometry.publish(message)
                for _ in range(8):
                    executor.spin_once(timeout_sec=.005)
        try:
            pump(0., .3)
            controller.set_parameters([Parameter('drive_direction', value='reverse')])
            response = controller.on_ready(Trigger.Request(), Trigger.Response())
            assert not response.success and 'reverse_route_missing' in response.message
            controller.set_parameters([Parameter('enabled', value=True)])
            pump(0., .2)
            assert commands[-1].linear.x == commands[-1].angular.z == 0.
            controller.set_parameters([Parameter('enabled', value=False)])
            controller.set_parameters([Parameter('drive_direction', value='forward')])
            assert controller.set_parameters([Parameter('enabled', value=True)])[0].successful
            for index in range(31):
                pump(index*.01, .04)
            controller.marker_gate.arrived = True  # 已有横线模块确认停车，此处只验证路线自动保存。
            pump(.3, .3)
            recorded = load_route(filename)
            assert controller.enabled and recorded['finished'], 'station pause did not auto-finalize'
            controller.set_parameters([Parameter('enabled', value=False)])
            recorded = load_route(filename)
            assert abs(recorded['points'][-1][0]-.3) < 1e-6
            # 新节点模拟重新启动底盘，里程计从0开始；实际小车仍在记录终点。
            executor.remove_node(controller)
            controller.destroy_node()
            controller = LineFollowNode()
            executor.add_node(controller)
            result = controller.set_parameters([Parameter('drive_direction', value='reverse'),
                                               Parameter('forward_speed', value=.05)])[0]
            assert result.successful
            pump(0., .3)
            response = controller.on_ready(Trigger.Request(), Trigger.Response())
            assert response.success, response.message
            assert controller.return_tracker is None, 'readiness must not anchor or start motion'
            controller.set_parameters([Parameter('enabled', value=True)])
            pump(0., .85)
            assert abs(commands[-1].linear.x+.05) < 1e-6
            assert commands[-1].linear.x < 0. and abs(commands[-1].angular.z) < 1e-6
            assert controller.reason == 'reverse_route_tracking', 'lost front image blocked recorded return'
            assert load_route(filename) == recorded, 'reverse overwrote the forward route'
            pump(0., .65, publish_odom=False)
            assert commands[-1].linear.x == 0. and controller.reason == 'route_odom_timeout'
            pump(0., .2)
            assert controller.reason == 'reverse_route_tracking'
            for x in (-.05, -.10, -.15, -.20, -.25, -.28, -.30):
                pump(x, .12)
                assert abs(commands[-1].angular.z) <= abs(commands[-1].linear.x)*controller.route_config.max_curvature+1e-6
            pump(-.30, .3)
            assert controller.reason == 'reverse_route_complete'
            assert commands[-1].linear.x == commands[-1].angular.z == 0.
            pump(2., .15)
            assert controller.odom_fault == 'route_odom_discontinuity'
            assert 'position_jump' in controller.odom_fault_detail['triggers']
            assert commands[-1].linear.x == 0.
            print('PASS: odometry recording, restart alignment, lost image ignored, odom timeout, goal stop, reset rejection')
        finally:
            controller.stop()
            executor.remove_node(controller)
            executor.remove_node(harness)
            controller.destroy_node()
            harness.destroy_node()
            executor.shutdown()
            rclpy.shutdown()


def recorded_route_camera_ros(visual_finish=False, holonomic=False, wide_entry=False):
    """记录路线倒车叠加同帧视觉横线，验证离站、锁停、继续和失效停车。"""
    import tempfile
    import numpy as np
    from cv_bridge import CvBridge
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from std_msgs.msg import Empty
    from recorded_route import RouteConfig, RouteRecorder
    with tempfile.TemporaryDirectory() as directory:
        filename = str(Path(directory)/'return.json')
        recorder = RouteRecorder(RouteConfig(), 'odom', 'base_link')
        for index in range(101):
            recorder.add((index*.01, 0., 0.))
        recorder.finished = True
        recorder.save(filename)
        rclpy.init(args=['--ros-args', '-p', 'station_log_file:=/tmp/line-test-stations-'+str(os.getpid())+'.json', '-p', 'enabled:=false', '-p', 'drive_direction:=reverse',
                        '-p', 'forward_speed:=0.05', '-p', 'route_camera_stop:=true',
                        '-p', 'reverse_steering_sign:=-1.0',
                        '-p', 'route_visual_finish:='+str(visual_finish).lower(),
                        '-p', 'route_visual_strafe:='+str(holonomic).lower(),
                        '-p', 'line_status_topic:=/return_camera_test/status',
                        '-p', 'route_file:='+filename, '-p', 'cmd_vel_topic:=/return_camera_test/cmd',
                        '-p', 'odom_topic:=/return_camera_test/odom',
                        '-p', 'comm_status_topic:=/return_camera_test/comm',
                        '-p', 'chassis_status_topic:=/return_camera_test/chassis',
                        '-p', 'centerline_topic:=/return_camera_test/path',
                        '-p', 'stop_marker_mask_topic:=/return_camera_test/mask'])
        executor = SingleThreadedExecutor()
        harness, controller = Node('return_camera_harness'), LineFollowNode()
        executor.add_node(harness)
        executor.add_node(controller)
        odom = harness.create_publisher(Odometry, '/return_camera_test/odom', 10)
        comm = harness.create_publisher(CommStatus, '/return_camera_test/comm', 10)
        chassis = harness.create_publisher(ChassisStatus, '/return_camera_test/chassis', 10)
        path_pub = harness.create_publisher(PolygonStamped, '/return_camera_test/path', qos_profile_sensor_data)
        mask_pub = harness.create_publisher(Image, '/return_camera_test/mask', qos_profile_sensor_data)
        resume_pub = harness.create_publisher(Empty, '/line_follow/resume', 10)
        from std_msgs.msg import String
        status_pub = harness.create_publisher(String, '/return_camera_test/status', 10)
        commands = []
        harness.create_subscription(Twist, '/return_camera_test/cmd', commands.append, 10)
        bridge = CvBridge()
        def pump(x, duration, bar=None, vision=True, mismatch=False, stale=False, offset=0., lost=False, short_line=False, slope=0., far_bend=0.):
            end = time.monotonic()+duration
            while time.monotonic() < end:
                comm.publish(CommStatus(comm_state=CommStatus.COMM_STATE_CONNECTED))
                chassis.publish(ChassisStatus(chassis_state=ChassisStatus.CHASSIS_STATE_COMM_CONTROL))
                message = Odometry()
                message.header.stamp = harness.get_clock().now().to_msg()
                message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
                message.pose.pose.position.x, message.pose.pose.orientation.w = x, 1.
                odom.publish(message)
                if vision:
                    path = PolygonStamped()
                    path.header.stamp = harness.get_clock().now().to_msg()
                    path.header.frame_id = 'test_optical_frame'
                    if stale:
                        path.header.stamp.sec -= 2
                    start_y, end_y = (100, 175) if short_line else (20, 461)
                    path.polygon.points = [Point32(x=320.+offset+slope*(y-100)+far_bend*max(0, y-220)**2,
                                                  y=float(y)) for y in range(start_y, end_y, 6)]
                    mask = np.zeros((480, 640), np.uint8)
                    cv2.polylines(mask, [np.array([(point.x, point.y) for point in path.polygon.points], np.int32)], False, 255, 16)
                    if bar is not None:
                        cv2.line(mask, (240, int(bar)), (400, int(bar)), 255, 16)
                    image = bridge.cv2_to_imgmsg(mask, encoding='mono8')
                    image.header.frame_id = path.header.frame_id
                    image.header.stamp.sec = path.header.stamp.sec
                    image.header.stamp.nanosec = path.header.stamp.nanosec
                    if mismatch:
                        image.header.stamp.nanosec = (image.header.stamp.nanosec+1)%1000000000
                    # 刻意让掩膜先到，验证乱序但同源时间戳仍可配对。
                    mask_pub.publish(image)
                    path_pub.publish(path)
                    status_pub.publish(String(data=json.dumps(observation(
                        lost=lost, source_stamp_sec=path.header.stamp.sec,
                        source_stamp_nanosec=path.header.stamp.nanosec))))
                for _ in range(8):
                    executor.spin_once(timeout_sec=.005)
        try:
            pump(0., .65, vision=False)
            ready = controller.on_ready(Trigger.Request(), Trigger.Response())
            assert not ready.success and 'reverse_stop_camera_timeout' in ready.message
            pump(0., .65, bar=240)
            assert controller.on_ready(Trigger.Request(), Trigger.Response()).success
            controller.set_parameters([Parameter('enabled', value=True)])
            pump(0., .65, bar=240)
            assert controller.marker_gate.startup_ignored and controller.previous_v < 0.
            pump(-.01, .5)
            for bar in (100, 130, 160, 190, 220, 240):
                pump(-.02, .35, bar=bar)
            pump(-.02, .65, bar=240)
            assert controller.reason == 'stop_marker_intermediate', controller.reason
            assert controller.marker_gate.arrived and commands[-1].linear.x == 0.
            counts = controller.marker_gate.ledger.counts
            assert counts['encountered'] == 2 and counts['ignored'] == 1 and counts['arrived'] == 1, counts
            assert counts['passed'] == 1, counts
            with open(controller.station_log_file, encoding='utf-8') as stream:
                journal = json.load(stream)
            assert journal['counts']['arrived'] == 1 and len(journal['stations']) == 2
            assert sum(event['action'] == 'ignored_departure' for event in journal['events']) == 1
            pump(-.02, .6, vision=False)
            assert controller.reason == 'stop_marker_intermediate' and commands[-1].linear.x == 0.
            resume_pub.publish(Empty())
            pump(-.02, .15, vision=False)
            assert controller.marker_gate.arrived, 'camera timeout incorrectly authorized resume'
            pump(-.02, .3, bar=240)
            resume_pub.publish(Empty())
            pump(-.02, .35, bar=240)
            assert not controller.marker_gate.arrived and controller.previous_v < 0.
            pump(-.03, .65, mismatch=True)
            assert controller.reason == 'reverse_stop_camera_timeout' and commands[-1].linear.x == 0.
            pump(-.03, .65, stale=True)
            assert commands[-1].linear.x == 0., 'stale frames refreshed camera safety'
            pump(-.03, .4)
            assert controller.previous_v < 0.
            for index in range(1, 11):
                pump(-index*.1, .15, offset=-180. if wide_entry else 0.)
                if visual_finish and index == 5:
                    if wide_entry:
                        pump(-.5, 1.2, offset=-180.)
                        assert controller.return_finish is not None, '偏离约56%半图宽无法进入纠偏'
                        assert controller.reason == 'reverse_visual_finish_aligning', controller.reason
                        assert commands[-1].linear.x == 0. and commands[-1].linear.y < 0.
                        assert abs(commands[-1].angular.z) < .005
                        assert not controller.return_finish.arrived, '放宽接管范围不能算作最终居中'
                        pump(-.5, .8, vision=False)
                        assert commands[-1].linear.x == commands[-1].linear.y == commands[-1].angular.z == 0.
                        print('PASS: logged 56% lateral offset enters bounded alignment, strafe points toward centre, no false arrival, stale camera stops')
                        return
                    pump(-.5, .45, offset=20., short_line=True)
                    assert controller.return_axis is None, 'fixture must fail whole-line fit'
                    assert controller.return_finish is not None, 'confirmed last straight did not hand over before 30cm'
                    assert controller.finish_entry_reason == 'confirmed_visual_path'
                    assert controller.reason == 'reverse_visual_finish_tracking'
                    if holonomic:
                        assert controller.previous_vy == controller.previous_w == 0., 'inside-tolerance noise moved the wheels'
                    else:
                        assert controller.previous_w != 0., '20px offset was ignored by final visual centering'
                    pump(-.5, .7, bar=240)
                    assert controller.reason == 'stop_marker_intermediate', 'early visual handover bypassed an intermediate stop'
                    assert not controller.return_finish.arrived
                    resume_pub.publish(Empty())
                    pump(-.5, .35, bar=240)
                    assert not controller.marker_gate.arrived
            pump(-1., .3)
            if holonomic:
                pump(-1., 1., offset=70.)
                assert controller.reason == 'reverse_visual_finish_aligning', controller.reason
                assert commands[-1].linear.x == 0. and commands[-1].linear.y > 0.
                assert abs(commands[-1].angular.z) < .005, 'lateral offset caused yaw instead of strafe'
                pump(-1., 1., offset=-70., far_bend=.001)
                assert commands[-1].linear.x == 0. and commands[-1].linear.y < 0.
                pump(-1., 1., slope=.15)
                assert commands[-1].linear.x == 0. and commands[-1].angular.z > 0.
                pump(-1., .5, vision=False)
                pump(-1., .2, vision=False)
                assert commands[-1].linear.x == commands[-1].linear.y == commands[-1].angular.z == 0.
                pump(-1., 1.)
                assert controller.reason == 'reverse_visual_finish_tracking', controller.reason
                for bar in (100, 130, 160, 190, 220, 240):
                    pump(-1., .3, bar=bar, offset=40.)
                pump(-1., .4, bar=240, offset=40.)
                # 到横线但没居中时只停纵向，横移仍有效，不会失败或误报完成。
                assert controller.marker_gate.arrived
                assert controller.reason == 'reverse_visual_finish_aligning', controller.reason
                assert commands[-1].linear.x == 0. and commands[-1].linear.y > 0.
                pump(-1., 1., bar=240)
                assert controller.reason == 'reverse_route_complete'
                assert commands[-1].linear.x == commands[-1].linear.y == commands[-1].angular.z == 0.
                print('PASS: holonomic reverse strafe/yaw separation, camera loss zeroes all axes, aligned terminal stop')
                return
            if visual_finish:
                assert controller.return_finish is not None
                assert controller.reason == 'reverse_visual_finish_tracking', controller.reason
                assert -.03 <= controller.previous_v < 0., 'visual speed limit or odom-goal handover failed'
                pump(-1.01, .65, offset=40.)
                first_yaw = controller.previous_w
                assert first_yaw < 0., 'positive camera offset must use standard reverse lateral correction despite legacy sign=-1'
                assert abs(first_yaw) <= abs(controller.previous_v)*controller.route_config.max_curvature+1e-6
                pump(-1.02, .9, offset=-40.)
                assert first_yaw*controller.previous_w < 0., 'opposite offsets did not change steering direction'
                before = controller.return_visual_feedback()
                pump(-1.02, .3, offset=-40., far_bend=.001)
                after = controller.return_visual_feedback()
                assert abs(before['error_normalized']-after['error_normalized']) < 1e-6
                assert abs(before['near_heading_rad']-after['near_heading_rad']) < 1e-6, 'front far bend changed local reverse steering axis'
                pump(-1.02, .75, offset=-65., slope=.1423)
                feedback = controller.return_visual_feedback()
                assert feedback['error_normalized'] < -.15 and feedback['near_heading_rad'] > .1
                assert controller.previous_w > 0., 'logged left-offset/positive-heading case was corrected backwards'
                assert controller.return_finish.diagnostics['lateral_yaw_radps'] > 0.
                assert controller.return_finish.diagnostics['heading_yaw_radps'] > 0.
                assert controller.config.reverse_steering_sign == -1. and controller.route_config.finish_lateral_sign == 1.
                pump(-1.02, .7, offset=-40., short_line=True)
                assert controller.return_axis is None, 'short measured segment unexpectedly passed strict whole-line fit'
                assert controller.reason == 'reverse_visual_finish_tracking', controller.reason
                assert controller.previous_v < 0. and controller.previous_w != 0., 'valid near line did not continue reverse alignment'
                assert controller.return_visual_feedback()['source'] == 'near_measured'
                pump(-1.02, .3, lost=True)
                assert controller.previous_v == controller.previous_w == 0.
                pump(-1.04, .4)
                assert controller.previous_v < 0.
                for bar in (100, 130, 160, 190, 220, 240):
                    pump(-1.04, .4, bar=bar, offset=40.)
                pump(-1.04, .4, bar=240, offset=40.)
                assert controller.marker_gate.arrived
                assert not controller.return_finish.arrived
                assert controller.reason == 'reverse_finish_goal_not_centered', controller.reason
                assert controller.previous_v == controller.previous_w == 0.
                pump(-1.04, .6, bar=240)
                assert controller.return_finish.arrived, controller.reason
            assert controller.reason == 'reverse_route_complete' and commands[-1].linear.x == 0.
            if visual_finish:
                pump(-1.04, .6, vision=False)
                assert controller.reason == 'reverse_route_complete' and commands[-1].linear.x == 0.
            if visual_finish:
                # 新任务没有横线：超过原起点10cm必须故障锁停，回到范围内也不自动开走。
                executor.remove_node(controller)
                controller.destroy_node()
                controller = LineFollowNode()
                executor.add_node(controller)
                pump(0., .4)
                controller.set_parameters([Parameter('enabled', value=True)])
                pump(0., .3)
                for index in range(1, 12):
                    pump(-index*.1, .15)
                pump(-1.11, .3)
                assert controller.reason == 'reverse_finish_search_limit', controller.reason
                assert commands[-1].linear.x == commands[-1].angular.z == 0.
                pump(-1.05, .3)
                assert controller.reason == 'reverse_finish_search_limit' and controller.previous_v == 0.
                executor.remove_node(controller)
                controller.destroy_node()
                controller = LineFollowNode()
                executor.add_node(controller)
                pump(0., .4, slope=.5)
                controller.set_parameters([Parameter('enabled', value=True)])
                pump(0., .3, slope=.5)
                for index in range(1, 9):
                    pump(-index*.1, .15, slope=.5)
                pump(-.8, .3, slope=.5)
                assert controller.return_finish is None, 'distance-only fallback forced a misaligned visual handover'
                assert controller.reason == 'reverse_route_tracking'
                assert abs(controller.previous_v) <= controller.route_config.finish_speed+1e-6
                pump(-.8, .55, slope=.1)
                assert controller.return_finish is not None and controller.reason == 'reverse_visual_finish_tracking'
                pump(-.81, .4, slope=.55)
                assert controller.reason == 'reverse_visual_finish_tracking' and controller.previous_v < 0., 'moderate heading error should slow down and recover'
                assert controller.previous_w*controller.return_visual_feedback()['near_heading_rad'] > 0., 'lateral calibration inverted heading correction'
                last_motion = dict(controller.return_finish.diagnostics['last_tracking_command'])
                assert last_motion['linear_x_mps'] == controller.previous_v
                assert last_motion['angular_z_radps'] == controller.previous_w
                pump(-.82, .3, slope=.8)
                assert controller.reason == 'reverse_finish_alignment_out_of_range' and controller.previous_v == 0.
                diagnostics = controller.return_finish.diagnostics
                assert 'heading' in diagnostics['alignment_exceeded']
                assert not diagnostics['control_terms_valid']
                last_before_stop = dict(diagnostics['last_tracking_command'])
                assert last_before_stop['time_unix'] >= last_motion['time_unix']
                assert last_before_stop['linear_x_mps'] < 0.
                assert abs(last_before_stop['visual_heading_rad']) <= controller.route_config.finish_exit_heading
                pump(-.82, .2, slope=.8)
                assert controller.return_finish.diagnostics['last_tracking_command'] == last_before_stop, 'stopped ticks erased the last moving command'
                # 现场约-0.513的偏差应继续低速纠偏，超过新余量仍应停车。
                pump(-.82, .45, offset=-165.)
                assert controller.reason == 'reverse_visual_finish_tracking' and controller.previous_v < 0.
                assert -.75 < controller.return_visual_feedback()['error_normalized'] < -.5
                pump(-.82, .3, offset=-250.)
                assert controller.reason == 'reverse_finish_alignment_out_of_range' and controller.previous_v == 0.
                assert controller.return_finish.diagnostics['alignment_exceeded'] == ['lateral']
                executor.remove_node(controller)
                controller.destroy_node()
                controller = LineFollowNode()
                executor.add_node(controller)
                controller.set_parameters([Parameter('drive_direction', value='forward')])
                pump(0., .4, bar=240)
                controller.set_parameters([Parameter('enabled', value=True)])
                pump(0., .5, bar=240)
                for x in (.1, .2, .3):
                    pump(x, .2)
                for y in (400, 370, 340, 310, 280, 250, 240):
                    pump(.3, .25, bar=y)
                pump(.3, .65, bar=240)
                assert controller.marker_gate.arrived and controller.marker_gate.ledger.counts['arrived'] == 1
                controller.set_parameters([Parameter('enabled', value=False)])
                recorded = json.loads(Path(filename).read_text())
                assert recorded['valid']
                actions = [event['action'] for event in recorded['station_events']]
                assert 'ignored_departure' in actions and 'arrived' in actions
                checkpoint = controller.on_save_route(Trigger.Request(), Trigger.Response())
                assert checkpoint.success, checkpoint.message
                snapshot = Path(json.loads(checkpoint.message)['snapshot_file'])
                saved_bytes = snapshot.read_bytes()
                pump(.3, 1.2)  # 模拟机械臂/TCP等待，里程计持续更新但不追加封存路线。
                assert snapshot.read_bytes() == saved_bytes and controller.recorder is None
                controller.set_parameters([Parameter('drive_direction', value='reverse')])
                assert [station['id'] for station in controller.recorded_station_order()] == [2, 1]
                assert controller.frozen_route_file == str(snapshot)
                pump(.36, .2)  # 等待期间底盘被移动，不能重新对齐旧路线后盲目倒车。
                result = controller.set_parameters([Parameter('enabled', value=True)])[0]
                assert not result.successful and 'endpoint_moved' in result.reason
                assert snapshot.read_bytes() == saved_bytes
                print('PASS: visual control, station ledger/file, forward route station events, reverse expected order and protections')
            print('PASS: reverse camera readiness, startup marker ignore, paired frames, intermediate latch/resume, camera timeout, stale/mismatched frames and route goal stop')
        finally:
            controller.stop()
            executor.remove_node(controller)
            executor.remove_node(harness)
            controller.destroy_node()
            harness.destroy_node()
            executor.shutdown()
            rclpy.shutdown()


def reverse_tracking_limits_ros():
    """只检查真实节点的输出限幅；隔离速度话题，不连接实车。"""
    import math
    from dataclasses import replace
    rclpy.init(args=['--ros-args', '-p', 'enabled:=true', '-p', 'drive_direction:=reverse',
                    '-p', 'record_route:=false', '-p', 'route_camera_stop:=false',
                    '-p', 'cmd_vel_topic:=/reverse_limits_test/cmd',
                    '-p', 'route_file:=/tmp/nonexistent-reverse-limits-test.json',
                    '-p', 'station_log_file:=/tmp/reverse-limits-test-stations.json'])
    node = LineFollowNode()
    try:
        points = [(.2*math.sin(t), .2*(1-math.cos(t)), t)
                  for t in (i*math.pi/200 for i in range(101))]
        node.route_document = dict(points=points, length_m=math.pi*.1)
        node.latest_odom = dict(pose=points[-1], frame='odom', v=0., vy=0., w=0.)
        # 输入失效/急停门控已有集成用例；此用例隔离检查路径跟踪到发布阶段的限幅。
        node.blocked_reason = lambda *args, **kwargs: None
        for speed in (.03, .08):
            node.config = replace(node.config, forward_speed=speed)
            node.return_tracker = None
            node.previous_v = node.previous_w = 0.
            for _ in range(150):
                node.tick_return_route(time.monotonic(), .02)
                assert node.reason == 'reverse_route_tracking', node.reason
                assert abs(node.previous_w) <= node.config.max_angular_speed+1e-9
                assert abs(node.previous_v) <= min(speed, node.config.reverse_max_speed)+1e-9
                assert abs(node.previous_w) <= abs(node.previous_v)*node.reverse_route_max_curvature+1e-9
            assert abs(node.previous_w/node.previous_v) > 4.5, '发布阶段仍按末段2/m削掉转弯需求'
            if speed == .08:
                assert abs(abs(node.previous_w)-node.config.max_angular_speed) < 1e-6
        assert node.route_config.max_curvature == 2., '末段充电的限制被途中配置覆盖'
        node.latest_odom['pose'] = (*points[-1][:2], points[-1][2]+math.pi)
        node.tick_return_route(time.monotonic(), .02)
        assert node.reason == 'reverse_target_not_behind'
        assert node.previous_v == node.previous_w == 0., '目标方向保护应立即停车'
        # A straight route can reach cruise speed, unlike a curvature-limited
        # bend. Changing the reverse cap must leave the forward setting intact.
        straight = [(i*.01, 0., 0.) for i in range(101)]
        node.return_fault = None  # Independent scenario after the latched-stop assertion above.
        node.route_document = dict(points=straight, length_m=1.)
        node.latest_odom['pose'] = straight[-1]
        for forward, reverse in ((.075, .05), (.075, .025), (.03, .05)):
            node.config = replace(node.config, forward_speed=forward, reverse_max_speed=reverse)
            node.return_tracker = None
            node.previous_v = node.previous_w = 0.
            for _ in range(150):
                node.tick_return_route(time.monotonic(), .02)
                assert node.reason == 'reverse_route_tracking', node.reason
                assert abs(node.previous_v) <= min(forward, reverse)+1e-9
            assert abs(node.previous_v+min(forward, reverse)) < 1e-6
            assert node.config.forward_speed == forward
            assert abs(node.previous_w) < 1e-9
        print('PASS: independent forward 0.075 / reverse 0.05 cruise and lower reverse limits')
        print('PASS: reverse tight bend survives publish-stage cap, yaw speed regulation, finish limit and stop protection')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


def charging_signal_ros():
    """真实节点，本机隔离速度话题；验证充电首帧停车、去重确认和锁停。"""
    from types import SimpleNamespace
    rclpy.init(args=['--ros-args', '-p', 'enabled:=true', '-p', 'drive_direction:=reverse',
                    '-p', 'charging_mode:=true', '-p', 'charging_status_layout:=bitfield32',
                    '-p', 'cmd_vel_topic:=/charging_test/cmd',
                    '-p', 'route_file:=/tmp/nonexistent-charging-test-route.json',
                    '-p', 'station_log_file:=/tmp/charging-test-stations.json'])
    node = LineFollowNode()
    try:
        node.return_tracker = SimpleNamespace(diagnostics={'remaining_m': .1}, points=[(0., 0.), (.1, 0.)])
        node.latest_odom = {'frame': 'odom', 'pose': (.1, 0., 0.),
                            'receipt': time.monotonic(), 'stamp': node.get_clock().now().nanoseconds}
        node.connected, node.comm_time = True, time.monotonic()
        def feed(offset):
            message = ChassisStatus(chassis_state=ChassisStatus.CHASSIS_STATE_COMM_CONTROL, system_status_flags=8)
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.stamp.nanosec += offset
            node.on_chassis(message)
            node.tick_return_route(time.monotonic(), .02)
            assert node.previous_v == node.previous_vy == node.previous_w == 0.
            return message
        first = feed(0)
        assert node.reason == 'charging_dock_confirming'
        node.on_chassis(first)
        assert node.charge_frames == 1, '重复CAN源时间戳不能累计确认'
        feed(1)
        feed(2)
        assert node.reason == 'reverse_route_complete' and node.dock.done
        node.charge_value = False
        node.tick_return_route(time.monotonic(), .02)
        assert node.reason == 'reverse_route_complete', '充电位抖动不能重新发车'
        node.dock.done = False
        node.charge_candidate_at = time.monotonic()
        node.tick_return_route(time.monotonic(), .02)
        assert node.reason == 'charging_dock_confirming' and node.previous_v == 0.
        node.charge_candidate_at -= 3.
        node.tick_return_route(time.monotonic(), .02)
        assert node.reason == 'charging_dock_charge_unstable' and node.previous_v == 0.
        node.charge_at = time.monotonic()-2
        assert node.charging_signal(time.monotonic()) is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def departure_readiness_ros():
    """本机节点验证：只豁免桩充电；安全门、倒车及反馈超时仍有效。"""
    rclpy.init(args=['--ros-args', '-p', 'enabled:=false', '-p', 'record_route:=false',
                    '-p', 'cmd_vel_topic:=/departure_test/cmd',
                    '-p', 'station_log_file:=/tmp/departure-test-stations.json'])
    node = LineFollowNode()
    try:
        now = time.monotonic()
        node.connected, node.comm_time = True, now
        node.latest_odom = dict(pose=(0., 0., 0.), receipt=now,
                               stamp=node.get_clock().now().nanoseconds)
        msg = ChassisStatus(chassis_state=7, dock_charging=True, system_status_flags=53510)
        def feed():
            msg.header.stamp = node.get_clock().now().to_msg()
            node.on_chassis(msg)
        feed()
        assert node.charge_value is True, '实车53510需按SDK dock_charging解析'
        assert node.departure_permitted() and not node.chassis_allowed
        assert node.blocked_reason(now, False) == 'camera_timeout', '只能豁免充电，不豁免相机'
        node.on_ready(None, Trigger.Response())
        assert node.departure.state == 'idle', '就绪检查不能开始离桩计时'
        for field, value in [('manual_charging', True), ('chassis_fault', 1),
                             ('obstacle_status_flags', 1), ('excessive_tilt', True), ('chassis_state', 1)]:
            saved = getattr(msg, field)
            setattr(msg, field, value)
            feed()
            assert not node.departure_permitted(), field
            assert node.blocked_reason(time.monotonic(), False) == 'chassis_not_ready'
            setattr(msg, field, saved)
        feed()
        node.config = __import__('dataclasses').replace(node.config, drive_direction='reverse')
        assert not node.departure_permitted(), '倒车绝不豁免充电停车'
        node.config = __import__('dataclasses').replace(node.config, drive_direction='forward')
        node.chassis_time = time.monotonic()-10
        assert node.blocked_reason(time.monotonic(), False) == 'chassis_not_ready'
        node.chassis_time = time.monotonic()
        node.departure.advance(0., (0., 0.), True)
        node.departure.advance(26., (0., 0.), True)
        assert node.blocked_reason(time.monotonic(), False) == 'dock_departure_timeout'
        print('PASS: dock departure preflight, actual SDK charging bit, safety gates, reverse prohibition, stale feedback and latched timeout')
    finally:
        node.destroy_node()
        rclpy.shutdown()



def standard_route_ros(check_finish=False, check_loss=None):
    """隔离话题：标准路线前进、辅助视觉、过期停车、文件不覆盖及起点拒绝。"""
    import tempfile
    import numpy as np
    from cv_bridge import CvBridge
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    with tempfile.TemporaryDirectory() as directory:
        directory = Path(directory)
        standard = directory/'standard.json'
        standard.write_text(json.dumps(dict(version=1,valid=True,finished=True,
            frame_id='odom',child_frame_id='base_link',points=[[i*.01,0.,0.] for i in range(31 if check_finish else 81)],
            standard_route=dict(schema='robot_standard_route_v1',quality=dict(accepted=True)))))
        original = standard.read_bytes()
        rclpy.init(args=['--ros-args','-p','enabled:=false','-p','navigation_mode:=replay',
            '-p','standard_route_file:='+str(standard),'-p','route_file:='+str(directory/'actual.json'),
            '-p','station_log_file:='+str(directory/'stations.json'),
            '-p','cmd_vel_topic:=/standard_test/cmd','-p','line_status_topic:=/standard_test/line',
            '-p','centerline_topic:=/standard_test/path','-p','stop_marker_mask_topic:=/standard_test/mask',
            '-p','comm_status_topic:=/standard_test/comm','-p','chassis_status_topic:=/standard_test/chassis',
            '-p','odom_topic:=/standard_test/odom','-p','forward_speed:=0.05',
            '-p','image_forward_direction:=up','-p','vehicle_reference_y_ratio:=1.05',
            '-p','steering_sign:=-1.0'])
        node,harness=LineFollowNode(),Node('standard_test_harness')
        executor=SingleThreadedExecutor()
        executor.add_node(node);executor.add_node(harness)
        line=harness.create_publisher(String,'/standard_test/line',10)
        path=harness.create_publisher(PolygonStamped,'/standard_test/path',qos_profile_sensor_data)
        mask=harness.create_publisher(Image,'/standard_test/mask',qos_profile_sensor_data)
        comm=harness.create_publisher(CommStatus,'/standard_test/comm',10)
        chassis=harness.create_publisher(ChassisStatus,'/standard_test/chassis',10)
        odom=harness.create_publisher(Odometry,'/standard_test/odom',10)
        commands=[]
        harness.create_subscription(Twist,'/standard_test/cmd',commands.append,10)
        position=[0.]
        def pump(seconds,visual=True,error=0.,odometry=True,missing=False):
            end=time.monotonic()+seconds
            previous=time.monotonic()
            while time.monotonic()<end:
                now=time.monotonic();dt=now-previous;previous=now
                v=commands[-1].linear.x if commands else 0.
                position[0]+=v*dt
                comm.publish(CommStatus(comm_state=CommStatus.COMM_STATE_CONNECTED))
                state=ChassisStatus(chassis_state=ChassisStatus.CHASSIS_STATE_COMM_CONTROL)
                state.header.stamp=harness.get_clock().now().to_msg();chassis.publish(state)
                if odometry:
                    message=Odometry();message.header.stamp=state.header.stamp
                    message.header.frame_id='odom';message.child_frame_id='base_link'
                    message.pose.pose.orientation.w=1.;message.pose.pose.position.x=position[0]
                    message.twist.twist.linear.x=v;odom.publish(message)
                if visual:
                    polygon=PolygonStamped();polygon.header.stamp=state.header.stamp
                    polygon.header.frame_id='camera_optical_frame'
                    x=319.5+319.5*error
                    polygon.polygon.points=[] if missing else [Point32(x=x,y=float(y)) for y in range(100,422,6)]
                    path.publish(polygon)
                    image=np.zeros((480,640),np.uint8)
                    cv2.line(image,(int(x),100),(int(x),421),255,16)
                    image=CvBridge().cv2_to_imgmsg(image,encoding='mono8');image.header=polygon.header;mask.publish(image)
                    payload=observation(error_normalized=error,image_width=640,image_height=480,
                        source_stamp_sec=state.header.stamp.sec,source_stamp_nanosec=state.header.stamp.nanosec)
                    if missing:
                        payload.update(lost=True, valid_row_fraction=0.)
                    line.publish(String(data=json.dumps(payload)))
                cycle_end=min(end, now+.125)
                while time.monotonic()<cycle_end:
                    executor.spin_once(timeout_sec=.002)
        try:
            pump(1.5)
            readiness=node.on_ready(None,Trigger.Response())
            assert readiness.success, readiness.message
            assert node.set_parameters([Parameter('enabled',value=True)])[0].successful
            pump(1.5)
            assert commands[-1].linear.x>0. and abs(commands[-1].angular.z)<.005
            assert node.standard_follower.diagnostics['progress_m']>0.
            if check_loss:
                if check_loss == 'curve':
                    import math
                    from recorded_route import StandardRouteFollower
                    curved = json.loads(standard.read_text())
                    curved['points'] = [[.25*math.sin(i*.01/.25), .25*(1-math.cos(i*.01/.25)), i*.01/.25]
                                        for i in range(81)]
                    node.standard_follower = StandardRouteFollower(curved, (position[0], 0., 0.), node.forward_route_config)
                    node.standard_follower.command((position[0], 0., 0.), .05)
                before = node.standard_follower.diagnostics['progress_m']
                commands.clear()
                pump(.6, missing=True)
                assert commands and all(0. < c.linear.x <= .05 for c in commands), 'fresh empty centerlines must not repeatedly publish zero'
                assert node.standard_follower.diagnostics['progress_m'] > before
                assert node.standard_visual_loss is not None and node.standard_correction == 0.
                if check_loss == 'curve':
                    assert commands[-1].angular.z > .01, 'recorded curve must keep steering when vision disappears'
                    assert node.steering_guard.state == 'standard_route'
                    curvature = abs(commands[-1].angular.z/commands[-1].linear.x)
                    assert 2. < curvature <= node.forward_route_config.max_curvature+1e-6, curvature
                    assert abs(commands[-1].angular.z) <= node.config.curve_max_angular_speed+1e-6
                elif check_loss in ('timeout', 'distance'):
                    if check_loss == 'timeout':
                        node.standard_visual_loss['started'] -= node.standard_visual_loss_timeout
                    else:
                        node.standard_visual_loss['travel'] = node.standard_visual_loss_distance
                    pump(.25, missing=True)
                    expected = 'standard_visual_loss_timeout' if check_loss == 'timeout' else 'standard_visual_loss_distance_limit'
                    assert node.standard_fault == expected, node.standard_fault
                    assert commands[-1].linear.x == commands[-1].angular.z == 0.
                    pump(.4)
                    assert node.standard_fault == expected and commands[-1].linear.x == 0., 'loss limit must latch'
                elif check_loss == 'finish':
                    position[0] = .66
                    pump(.3, missing=True)
                    assert commands[-1].linear.x == 0., 'final approach cannot continue without vision'
                    assert not node.marker_gate.arrived
                elif check_loss == 'odom':
                    pump(.6, missing=True, odometry=False)
                    assert commands[-1].linear.x == 0., 'odometry loss must still stop'
                elif check_loss == 'deviation':
                    node.latest_odom['pose'] = (position[0], .3, 0.)
                    result = node.standard_forward_command(time.monotonic(), 'insufficient_heading_points')
                    assert result[:2] == (0., 0.) and node.standard_fault == 'standard_route_deviation'
                else:
                    pump(.6)
                    assert node.standard_visual_loss is None and commands[-1].linear.x > 0.
                assert standard.read_bytes() == original
                print('PASS: standard odometry-led fresh line-loss continuation: '+str(check_loss))
                return
            if check_finish:
                deadline=time.monotonic()+28.
                while node.standard_fault is None and time.monotonic()<deadline:
                    pump(.25)
                    if check_finish=='timeout' and node.standard_finish is not None:
                        node.standard_finish['started']-=31.
                expected='standard_finish_timeout' if check_finish=='timeout' else 'standard_finish_search_limit'
                assert node.standard_fault==expected,(node.reason,node.standard_finish)
                assert commands[-1].linear.x==0.
                assert position[0]<=.36,position[0]
                assert standard.read_bytes()==original
                print('PASS: standard final visual handoff, bounded extra search/timeout and immutable route: '+expected)
                return
            pump(.5,error=.15)
            assert 0. < abs(node.standard_correction)<=.06
            assert node.standard_navigator.speed_factor < 1., 'visual deviation must reduce route speed'
            assert standard.read_bytes()==original
            pump(.8,visual=False)
            assert commands[-1].linear.x==0.,'视觉长时间失效仍必须停车'
            node.set_parameters([Parameter('enabled',value=False)])
            pump(.4,error=.4)
            assert not node.set_parameters([Parameter('enabled',value=True)])[0].successful,'起点偏离不能强行重放'
            pump(.4)
            node.standard_file=str(directory/'missing.json')
            from std_srvs.srv import SetBool
            response = node.on_enable(SetBool.Request(data=True), SetBool.Response())
            assert not response.success
            assert 'missing.json' in response.message and response.message != 'disabled', response.message
            assert not node.enabled
            assert standard.read_bytes()==original
            print('PASS: standard path forward, bounded visual assist, stale vision stop, origin alignment, missing route rejection and immutable standard')
        finally:
            executor.shutdown();node.destroy_node();harness.destroy_node();rclpy.shutdown()

def start_alignment_ros(replay=False, docked=False):
    """The recorder and replay anchor must wait for the aligned odometry pose."""
    import tempfile
    import numpy as np
    from cv_bridge import CvBridge
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    with tempfile.TemporaryDirectory() as directory:
        route_file = str(Path(directory)/'route.json')
        standard_file = str(Path(directory)/'standard.json')
        if replay:
            points = [[i*.01, 0., 0.] for i in range(31)]
            Path(standard_file).write_text(json.dumps(dict(version=1, valid=True, finished=True,
                frame_id='odom', child_frame_id='base_link', points=points,
                standard_route=dict(schema='robot_standard_route_v1',
                                    quality=dict(accepted=True)))))
        rclpy.init(args=['--ros-args', '-p', 'enabled:=false', '-p', 'dry_run:=true',
            '-p', 'record_route:='+str(not replay), '-p', 'navigation_mode:='+('replay' if replay else 'vision'),
            '-p', 'standard_route_file:='+standard_file, '-p', 'start_alignment_enabled:=false',
            '-p', 'image_forward_direction:=down', '-p', 'forward_speed:=0.05',
            '-p', 'start_alignment_speed:=0.02', '-p', 'route_file:='+route_file,
            '-p', 'station_log_file:='+str(Path(directory)/'stations.json'),
            '-p', 'cmd_vel_topic:=/start_alignment_test/cmd',
            '-p', 'odom_topic:=/start_alignment_test/odom',
            '-p', 'line_status_topic:=/start_alignment_test/status',
            '-p', 'centerline_topic:=/start_alignment_test/path',
            '-p', 'stop_marker_mask_topic:=/start_alignment_test/mask'])
        executor = SingleThreadedExecutor()
        harness, controller = Node('start_alignment_harness'), LineFollowNode()
        executor.add_node(harness)
        executor.add_node(controller)
        odom_pub = harness.create_publisher(Odometry, '/start_alignment_test/odom', 10)
        status_pub = harness.create_publisher(String, '/start_alignment_test/status', 10)
        path_pub = harness.create_publisher(PolygonStamped, '/start_alignment_test/path', qos_profile_sensor_data)
        mask_pub = harness.create_publisher(Image, '/start_alignment_test/mask', qos_profile_sensor_data)
        commands = []
        harness.create_subscription(Twist, '/line_follow/cmd_vel', commands.append, 10)
        bridge = CvBridge()
        odom_x, odom_y, marker_x, marker_y = [0.], [0.], [350], [290]
        charging = [docked]
        origin_x = .04 if docked else 0.

        def pump(duration):
            end = time.monotonic()+duration
            while time.monotonic() < end:
                stamp = harness.get_clock().now().to_msg()
                odom = Odometry()
                odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_link'
                odom.pose.pose.position.x = odom_x[0]
                odom.pose.pose.position.y = odom_y[0]
                odom.pose.pose.orientation.w = 1.
                odom_pub.publish(odom)
                if docked:
                    controller.on_chassis(ChassisStatus(
                        header=odom.header,
                        chassis_state=ChassisStatus.CHASSIS_STATE_COMM_CONTROL,
                        dock_charging=charging[0]))
                path = PolygonStamped()
                path.header.stamp, path.header.frame_id = stamp, 'start_alignment_camera'
                path.polygon.points = [Point32(x=float(marker_x[0]), y=float(y)) for y in range(100, 421, 6)]
                path_pub.publish(path)
                mask = np.zeros((480, 640), np.uint8)
                cv2.line(mask, (marker_x[0], 100), (marker_x[0], 420), 255, 16)
                cv2.line(mask, (marker_x[0]-80, marker_y[0]), (marker_x[0]+80, marker_y[0]), 255, 16)
                image = bridge.cv2_to_imgmsg(mask, encoding='mono8')
                image.header.stamp, image.header.frame_id = stamp, path.header.frame_id
                mask_pub.publish(image)
                status_pub.publish(String(data=json.dumps(observation(
                    source_stamp_sec=stamp.sec, source_stamp_nanosec=stamp.nanosec,
                    image_width=640, image_height=480))))
                for _ in range(6):
                    executor.spin_once(timeout_sec=.005)

        try:
            pump(.4)
            result = controller.set_parameters([
                Parameter('start_alignment_enabled', value=True),
                Parameter('enabled', value=True)])
            assert result[0].successful, result[0].reason
            if docked:
                commands.clear()
                pump(.6)
                assert controller.departure.state == 'active'
                assert controller.start_alignment_state == 'start_alignment_departing_dock'
                assert commands and 0. < commands[-1].linear.x <= controller.departure_speed
                assert all(c.linear.y == c.angular.z == 0. for c in commands)
                assert controller.start_aligner.step_count == 0
                if replay:
                    assert controller.standard_follower is None
                else:
                    assert not controller.recorder.points
                # A blocked input immediately stops undocking as well.
                assert controller.tick_alignment_departure(
                    time.monotonic(), .02, 'camera_timeout')[:3] == (0., 0., 0.)
                old_start = controller.start_aligner.started_at
                odom_x[0] = origin_x
                charging[0] = False
                pump(.15)
                assert controller.departure.state == 'complete'
                assert controller.start_aligner.started_at > old_start
                assert commands[-1].linear.x == commands[-1].linear.y == commands[-1].angular.z == 0.
            pump(1.3)  # Initial stationary dwell plus fresh stable camera frames.
            assert controller.start_alignment_active
            if replay:
                assert controller.standard_document is not None
                assert controller.standard_follower is None, 'replay anchor must wait for visual alignment'
            else:
                assert controller.recorder is not None and not controller.recorder.points
            assert commands and 0. < commands[-1].linear.y <= controller.config.start_alignment_lateral_speed
            assert commands[-1].linear.x == commands[-1].angular.z == 0.
            assert abs(controller.start_aligner.step_lateral_m-.005) < 1e-9
            assert controller.start_aligner.velocity()[1] >= controller.config.start_alignment_min_lateral_speed
            pump(.35)  # A minimum-speed command is not evidence of actual motion.
            assert controller.start_alignment_active and controller.start_aligner.phase == 'moving'
            assert controller.start_aligner.snapshot()['step_actual_m'] == 0.
            assert commands[-1].linear.y >= controller.config.start_alignment_min_lateral_speed
            odom_y[0], marker_x[0] = .005, 320
            pump(1.5)  # Finish lateral alignment; a separate forward step is still needed.
            assert controller.start_alignment_active
            assert commands[-1].linear.x > 0.
            assert commands[-1].linear.y == commands[-1].angular.z == 0.
            if replay:
                assert controller.standard_follower is None
            else:
                assert not controller.recorder.points, 'lateral corrections must not enter the route'
            odom_x[0], marker_y[0] = origin_x+.02, 240
            pump(1.6)  # Stop at the odometry target, then observe new stopped frames.
            assert not controller.start_alignment_active, controller.start_alignment_state
            if replay:
                assert controller.standard_follower is not None
                anchor = controller.standard_follower.tracker.points[0]
                assert abs(anchor[0]-(origin_x+.02)) < 1e-6, anchor
                assert abs(anchor[1]-.005) < 1e-6, anchor
            else:
                assert controller.recorder.points
                assert abs(controller.recorder.points[0][0]-(origin_x+.02)) < 1e-6, controller.recorder.points[0]
                assert abs(controller.recorder.points[0][1]-.005) < 1e-6, controller.recorder.points[0]
            assert controller.marker_gate.startup_ignored
            if docked:
                # A failed undocking must latch a workflow-visible alignment
                # failure, never fall through to a visual step or retry itself.
                controller.set_parameters([Parameter('enabled', value=False)])
                charging[0] = True
                pump(.1)
                result = controller.set_parameters([Parameter('enabled', value=True)])
                assert result[0].successful, result[0].reason
                pump(.2)
                assert controller.departure.state == 'active'
                controller.departure.started -= controller.departure.timeout+1.
                pump(.2)
                assert controller.start_alignment_state == 'start_alignment_failed_dock_departure_timeout'
                assert commands[-1].linear.x == commands[-1].linear.y == commands[-1].angular.z == 0.
                assert controller.start_aligner.step_count == 0
                charging[0] = False
                pump(.2)
                assert controller.start_alignment_state == 'start_alignment_failed_dock_departure_timeout'
            print('PASS: startup servo is bounded; '+
                  ('replay anchor' if replay else 'route origin')+
                  ' uses the post-alignment odometry pose')
        finally:
            executor.shutdown()
            controller.destroy_node()
            harness.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    reverse_tracking_limits_ros()
    departure_readiness_ros()
    charging_signal_ros()
    route_pause_snapshot_checks()
    background_work_ros()
    main()
    recorded_route_ros()
    recorded_route_camera_ros()
    start_alignment_ros()
    start_alignment_ros(replay=True)
    start_alignment_ros(docked=True)
    start_alignment_ros(replay=True, docked=True)
