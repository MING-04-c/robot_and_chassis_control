#!/usr/bin/env python3
"""Drive the chassis from ruier_camer_diver observations; stop on lost/stale input."""

import json
import math
import os
import time
import copy
from pathlib import Path
from uuid import uuid4
from threading import Event, Lock, Thread
from types import SimpleNamespace
from dataclasses import replace

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, Path as NavPath
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PolygonStamped, Twist
from masc_chassis_msgs.msg import ChassisStatus, CommStatus
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String, Empty
from std_srvs.srv import SetBool, Trigger

from stop_marker import StopMarkerGate, detect_stop_markers
from line_follow_workers import LatestWorker
from reverse_visual import GroundMapping, near_line, aligned as visually_aligned, strafe_command, AlignmentSequencer
from recorded_route import write_route_document, RouteConfig, RouteRecorder, ReverseRouteFollower, ReverseFinishGuard, load_route, wrap
from line_follow_control import (ControlConfig, SteeringDirectionGuard, coordinated_velocity,
                                 line_command, motion_preview, tracking_geometry, straight_image_axis, reverse_visual_command, slew)


class LineFollowNode(Node):
    def __init__(self):
        super().__init__('line_follow_controller')
        cv2.setNumThreads(1)  # 限制掩膜处理线程，避免和控制循环争抢所有 CPU。
        fixed = ParameterDescriptor(read_only=True)
        defaults = ControlConfig()
        values = {name: self.declare_parameter(name, value).value
                  for name, value in vars(defaults).items()}
        self.config = ControlConfig(**values)
        self.config.validate()
        self.reverse_mode = self.declare_parameter('reverse_mode', 'recorded_route', fixed).value
        if self.reverse_mode not in ('recorded_route', 'camera'):
            raise ValueError('reverse_mode must be recorded_route or camera')
        self.route_camera_stop = self.declare_parameter('route_camera_stop', True, fixed).value
        self.route_visual_finish = self.declare_parameter('route_visual_finish', True, fixed).value
        self.return_finish = None
        self.return_observation = self.return_geometry = self.return_axis = None
        self.finish_straight_frames = self.finish_goal_frames = self.strafe_entry_frames = 0
        self.finish_entry_reason = None
        self.return_geometry_reason = 'reverse_finish_line_unavailable'
        self.return_camera_frames = {}
        self.last_return_camera_check = 0.
        self.return_camera_stamp = -1
        self.record_route = self.declare_parameter('record_route', True, fixed).value
        self.route_file = self.declare_parameter('route_file', '~/.ros/line_follow/last_route.json', fixed).value
        self.frozen_route_file = None
        self.route_checkpoint = None
        self.checkpoint_invalid_reason = None
        self.station_log_file = self.declare_parameter(
            'station_log_file', '~/.ros/line_follow/last_station_log.json', fixed).value
        self.station_run_events = []
        self.route_config = RouteConfig(**{name: self.declare_parameter('route_'+name, value, fixed).value
                                          for name, value in vars(RouteConfig()).items()})
        self.route_config.validate()
        mapping_file = self.declare_parameter('reverse_ground_mapping_file', '', fixed).value
        self.ground_mapping = GroundMapping(mapping_file) if mapping_file else None
        self.reverse_feedback = None
        self.reverse_vy = self.previous_vy = 0.
        self.visual_alignment_since = None
        self.alignment_sequencer = AlignmentSequencer()
        self.last_alignment_phase = None
        self.latest_odom = None
        self.odom_fault = None
        self.odom_fault_detail = None
        self.recorder = self.return_tracker = self.route_document = None
        self.route_load_error = None
        self.return_fault = None
        self.record_stop_at = None
        self.last_route_save = self.last_route_log = 0.
        self.route_saved_finished = False
        self.enabled = self.declare_parameter('enabled', True).value
        self.dry_run = self.declare_parameter('dry_run', False, fixed).value
        self.show_window = self.declare_parameter('show_window', False, fixed).value
        self.bridge = CvBridge()
        self.window_name = 'MASC line following - body reference'
        self.geometry = None
        self.marker_gate = StopMarkerGate()
        self.marker_candidates = []
        self.stop_detection = {}
        self.marker_time = None
        self.marker_header = None
        self.marker_stamp = None
        self.control_curve_strength = 0.0
        self.steering_guard = SteeringDirectionGuard()
        self.last_steering_log = 0.
        self.frame_sync = False
        self.frame_cache = {}
        self.committed_stamp = -1
        self.frame_fault = None
        self.recovery_deadline = None
        self.exposure_status = None
        self.last_state = 0.0
        self.last_debug_time = 0.0
        if self.show_window and not (os.getenv('DISPLAY') or os.getenv('WAYLAND_DISPLAY')):
            self.get_logger().warning('No desktop display; use /line_follow/debug_image remotely')
            self.show_window = False
        self.input_timeout = self.declare_parameter('input_timeout_sec', 0.5, fixed).value
        self.comm_timeout = self.declare_parameter('comm_timeout_sec', 2.5, fixed).value
        self.control_period = self.declare_parameter('control_period_sec', 0.02, fixed).value
        self.reacquire_frames = self.declare_parameter('reacquire_frames', 3, fixed).value
        if not 0.01 <= self.control_period <= 0.05:
            raise ValueError('control_period_sec must be 0.01..0.05 (driver timeout is 0.1 s)')
        if not 0.1 <= self.input_timeout <= 1.0 or not 1.0 <= self.comm_timeout <= 5.0:
            raise ValueError('input timeout must be 0.1..1; comm timeout must be 1..5')
        if self.reacquire_frames < 1:
            raise ValueError('reacquire_frames must be >= 1')
        topics = {}
        for name, default in (
            ('line_status_topic', '/camera/line/status'),
            ('centerline_topic', '/camera/line/centerline'),
            ('cmd_vel_topic', '/cmd_vel'),
            ('comm_status_topic', '/masc_chassis/comm_status'),
            ('chassis_status_topic', '/masc_chassis/chassis_status'),
        ):
            topics[name] = self.declare_parameter(name, default, fixed).value
        if self.dry_run:
            topics['cmd_vel_topic'] = '/line_follow/cmd_vel'
        self.status = None
        self.status_time = self.path_time = self.comm_time = self.chassis_time = None
        self.path_header = None
        self.path_points = []
        self.path_valid = False
        self.connected = False
        self.chassis_allowed = False
        self.good_frames = 0
        self.ready_frames = 0
        self.previous_v = self.previous_w = self.previous_vy = 0.0
        self.last_tick = time.monotonic()
        self.last_control_delay_log = 0.
        self.reason = None
        self.input_reason = 'waiting'
        self.last_blocked_reason = None
        self.cmd_pub = self.create_publisher(Twist, topics['cmd_vel_topic'], 10)
        self.state_pub = self.create_publisher(String, '/line_follow/status', 10)
        self.station_event_pub = self.create_publisher(String, '/line_follow/station_events', 10)
        self.debug_pub = self.create_publisher(Image, '/line_follow/debug_image', qos_profile_sensor_data)
        debug_image_topic = self.declare_parameter(
            'debug_image_topic', '/camera/line/debug_image', fixed).value
        self.create_subscription(
            Image, debug_image_topic, self.on_debug_image,
            QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability))
        marker_mask_topic = self.declare_parameter('stop_marker_mask_topic', '/camera/line/stop_marker_mask', fixed).value
        self.create_subscription(Image, marker_mask_topic, self.on_marker_mask,
                                 QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability))
        self.create_subscription(Empty, '/line_follow/resume', self.on_resume_marker, 10)
        self.create_subscription(String, topics['line_status_topic'], self.on_status, 10)
        self.create_subscription(PolygonStamped, topics['centerline_topic'],
                                 self.on_path, qos_profile_sensor_data)
        self.create_subscription(CommStatus, topics['comm_status_topic'], self.on_comm, 10)
        self.create_subscription(ChassisStatus, topics['chassis_status_topic'],
                                 self.on_chassis, 10)
        self.create_subscription(String, '/line_follow/exposure_status', self.on_exposure_status, 10)
        odom_topic = self.declare_parameter('odom_topic', '/wheel_odom', fixed).value
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.route_pub = self.create_publisher(NavPath, '/line_follow/recorded_path', 1)
        self.create_service(SetBool, '~/enable', self.on_enable)
        self.create_service(Trigger, '~/ready', self.on_ready)
        self.create_service(Trigger, '~/save_route', self.on_save_route)
        self.add_on_set_parameters_callback(self.on_parameters)
        # 控制/里程计回调仍串行，避免给状态机引入共享写入竞争。
        # 图像运算及写盘各自使用有界后台槽，不阻塞该执行器。
        self.preview_lock = Lock()
        self.preview_frame = None
        self.preview_closed = Event()
        self.workers_closing = Event()
        self.window_created = False
        self.marker_worker = LatestWorker(self.detect_marker_frame, 'line-marker')
        self.preview_worker = LatestWorker(self.draw_preview, 'line-preview')
        self.route_writer = LatestWorker(write_route_document, 'line-route-save')
        self.station_writer = LatestWorker(write_route_document, 'line-station-save')
        self.create_timer(self.control_period, self.tick)
        if self.uses_recorded_route():
            self.load_return_route()
        elif self.enabled and self.record_route:
            self.recorder = RouteRecorder(self.route_config, 'odom', 'base_link')
            self.flush_route(wait=True)
        self.get_logger().info(
            f'Line following ready: enabled={self.enabled}, '
            f'direction={self.config.drive_direction}, reverse_steering_sign={self.config.reverse_steering_sign:+.0f}, '
            f'finish_lateral_sign={self.route_config.finish_lateral_sign:+.0f}, '
            f'visual_strafe={self.route_config.visual_strafe}, '
            f'geometry={"ground_meters" if self.ground_mapping else "image_uncalibrated"}, '
            f'forward_speed={self.config.forward_speed:.3f} m/s; '
            f'{topics["line_status_topic"]} -> {topics["cmd_vel_topic"]}')

    def uses_recorded_route(self):
        return self.reverse_mode == 'recorded_route' and self.config.drive_direction == 'reverse'

    def return_camera_stop_enabled(self):
        return self.uses_recorded_route() and self.route_camera_stop and self.config.stop_marker_enabled

    def return_camera_reason(self, now):
        if self.marker_time is None or self.marker_header is None or now-self.marker_time > self.input_timeout:
            return 'reverse_stop_camera_timeout'
        stamp = self.marker_header.stamp
        age = (self.get_clock().now().nanoseconds-stamp.sec*10**9-stamp.nanosec)/1e9
        if not -.1 <= age <= self.input_timeout:
            return 'reverse_stop_camera_stale'
        return None

    def stage_return_camera(self, kind, message):
        """倒车配对同帧观测，前段只作横线保险，最后直线使用图像几何纠偏。"""
        if not self.return_camera_stop_enabled():
            return
        if kind == 'status':
            try:
                key = int(message['source_stamp_sec'])*10**9+int(message['source_stamp_nanosec'])
            except (KeyError, TypeError, ValueError):
                return
        else:
            stamp = message.header.stamp
            key = stamp.sec*10**9+stamp.nanosec
        age = (self.get_clock().now().nanoseconds-key)/1e9
        if (key <= self.return_camera_stamp or not -.1 <= age <= self.input_timeout
                or kind != 'status' and message.header.frame_id == 'synthetic_demo' and not self.dry_run):
            return
        entry = self.return_camera_frames.setdefault(key, {})
        entry[kind] = message
        while len(self.return_camera_frames) > 3:
            self.return_camera_frames.pop(min(self.return_camera_frames))
        required = {'path', 'mask', 'status'} if self.route_visual_finish else {'path', 'mask'}
        if not required <= entry.keys():
            return
        if entry['path'].header.frame_id != entry['mask'].header.frame_id:
            return
        now = time.monotonic()
        if now-self.last_return_camera_check < .1:
            return  # 横线检查最多10Hz，不按50Hz控制周期重复处理图像。
        self.last_return_camera_check = now
        if self.return_camera_stamp >= 0 and (key-self.return_camera_stamp)/1e9 > self.input_timeout:
            self.finish_straight_frames = self.finish_goal_frames = self.strafe_entry_frames = 0
        self.return_camera_stamp = key
        self.path_header = entry['path'].header
        self.path_points = [(point.x, point.y) for point in entry['path'].polygon.points]
        if self.route_visual_finish:
            self.status = entry['status']
            if self.route_config.visual_strafe:
                mask = self.bridge.imgmsg_to_cv2(entry['mask'], desired_encoding='mono8')
                try:
                    self.reverse_feedback = near_line(self.path_points, mask, self.config, self.ground_mapping)
                    if self.status.get('lost') is not False or self.status.get('processing_over_budget') is not False:
                        self.reverse_feedback = None
                except ValueError as error:
                    self.reverse_feedback = None
                    self.return_fault = 'reverse_ground_mapping_invalid'
                    self.get_logger().error(str(error))
            self.return_geometry, self.return_geometry_reason = tracking_geometry(
                self.path_points, self.status.get('image_width'), self.status.get('image_height'), self.config)
            self.return_observation = (self.local_observation(self.status, self.return_geometry)
                                       if self.return_geometry_reason is None else None)
            self.return_axis = (straight_image_axis(self.path_points, self.status.get('image_width'),
                                self.status.get('image_height'), self.return_geometry, self.config)
                                if self.return_geometry_reason is None else None)
            feedback = self.return_visual_feedback()
            # 提前接管允许可信近端线段；不能要求反光/横线分割后的整条胶带都拟合成功。
            valid = (feedback is not None and abs(feedback['near_heading_rad']) <= self.route_config.finish_entry_heading
                     and abs(feedback['error_normalized']) <= .35
                     and abs(self.return_geometry.get('bend_angle_rad', math.inf)) <= self.route_config.finish_entry_bend
                     and line_command(self.return_observation, self.config)[2] == 'tracking')
            self.finish_straight_frames = min(3, self.finish_straight_frames+1) if valid else 0
            if self.route_config.visual_strafe:
                valid = (feedback is not None and abs(feedback['near_heading_rad']) <= self.route_config.finish_entry_heading
                         and abs(feedback['error_normalized']) <= .35)
                # 不让远处弯道或交叉处的原始中线斜率覆盖已去横线的局部估计。
                self.finish_straight_frames = min(3, getattr(self, 'strafe_entry_frames', 0)+1) if valid else 0
                self.strafe_entry_frames = self.finish_straight_frames
            centered = (feedback is not None and (visually_aligned(feedback, self.route_config)
                        if self.route_config.visual_strafe else valid and self.route_config.finish_alignment_ok(
                            feedback['error_normalized'], feedback['near_heading_rad'])))
            self.finish_goal_frames = min(3, self.finish_goal_frames+1) if centered else 0
        self.accept_marker_mask(entry['mask'])
        self.return_camera_frames = {k: frame for k, frame in self.return_camera_frames.items() if k > key}

    def flush_route(self, finish=False, wait=False):
        if self.recorder is not None:
            was_finished = self.recorder.finished
            # SIGINT 后不再查询已关闭的ROS时钟；保留最后一次真实停稳确认。
            fresh = rclpy.ok() and self.latest_odom is not None and self.odom_reason(time.monotonic()) is None
            paused_at_marker = self.config.stop_marker_enabled and self.marker_gate.arrived
            if fresh:
                stopped = (abs(self.latest_odom['v']) < .01 and abs(self.latest_odom['vy']) < .01
                           and abs(self.latest_odom['w']) < .05)
                self.recorder.finished = bool((finish or not self.enabled or paused_at_marker) and stopped)
            if self.recorder.fault is not None or self.odom_fault is not None:
                self.recorder.finished = False
            try:
                if hasattr(self, 'route_writer'):
                    ticket = self.route_writer.submit((self.route_file, self.recorder.document()))
                    if wait:
                        self.route_writer.wait(ticket)
                        self.check_route_writer()
                else:
                    self.recorder.save(self.route_file)
                self.last_route_save = time.monotonic()
                if (not hasattr(self, 'route_writer') and self.recorder.finished
                        and not was_finished and self.recorder.document()['valid']):
                    message = (f'前进路线已停稳保存：{os.path.expanduser(self.route_file)}；'
                               f'{self.recorder.document()["length_m"]:.3f}m')
                    if rclpy.ok():
                        self.get_logger().info(message)
                    else:
                        print(message, flush=True)  # SIGINT 后 ROS 上下文已关闭，保留终端保存结果。
            except (OSError, ValueError, TimeoutError) as error:
                self.recorder.fault = 'route_save_failed'
                self.get_logger().error(f'路线保存失败：{error}')

    def recorded_station_order(self):
        """只展示前进记录的期望顺序，不以计数冒充实际站点匹配。旧路线返回空列表。"""
        stations = {}
        for event in (self.route_document or {}).get('station_events', []):
            if not isinstance(event, dict) or event.get('action') not in ('ignored_departure', 'arrived'):
                continue
            identity = event.get('id')
            if type(identity) is int and identity > 0:
                stations[identity] = event.get('kind', 'unknown')
        return [dict(id=identity, kind=kind) for identity, kind in reversed(list(stations.items()))]

    def save_station_log(self, wait=False):
        if not self.marker_gate.ledger.stations:
            return
        payload = dict(version=1, saved_at_unix=time.time(), direction=self.config.drive_direction,
                       route_file=os.path.expanduser(self.route_file), counts=dict(self.marker_gate.ledger.counts),
                       stations=self.marker_gate.ledger.snapshot(), events=list(self.station_run_events),
                       expected_forward_station_order=self.recorded_station_order() if self.uses_recorded_route() else [])
        ticket = self.station_writer.submit((self.station_log_file, payload))
        if wait:
            self.station_writer.wait(ticket)

    def drain_station_events(self):
        pending, self.marker_gate.ledger.pending = self.marker_gate.ledger.pending, []
        names = dict(detected='识别新横线', ignored_departure='忽略出发线', arrived='到站停车',
                     resumed='收到继续，离站', passed='确认通过', released_unseen='漏检行程达到上限，解除去重（非到站）')
        for event in pending:
            event = dict(event, direction=self.config.drive_direction, time_unix=time.time())
            if self.latest_odom is not None:
                event['odom_pose_at_receipt'] = list(self.latest_odom['pose'])
            self.station_run_events.append(event)
            self.station_run_events = self.station_run_events[-512:]
            if self.recorder is not None and self.config.drive_direction == 'forward':
                self.recorder.record_station(event)
            if rclpy.ok():
                self.station_event_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))
                count = event['counts']
                self.get_logger().info(
                    f'横线#{event["id"]} {names.get(event["action"], event["action"])}；'
                    f'识别{count["encountered"]}，忽略{count["ignored"]}，到站{count["arrived"]}，'
                    f'通过{count["passed"]}，通过未确认{count["uncertain"]}')
        if pending:
            self.save_station_log()
            if self.recorder is not None and self.config.drive_direction == 'forward':
                self.flush_route()
        result = self.station_writer.take()
        if result is not None and result[2] is not None and rclpy.ok():
            self.get_logger().warning(f'站点日志保存失败：{result[2]}')

    def load_return_route(self):
        self.return_tracker = None
        self.reverse_feedback = None
        self.reverse_vy = self.previous_vy = 0.
        self.visual_alignment_since = None
        self.alignment_sequencer = AlignmentSequencer()
        self.last_alignment_phase = None
        self.return_finish = None
        self.return_fault = None
        self.finish_entry_reason = None
        self.finish_straight_frames = self.finish_goal_frames = self.strafe_entry_frames = 0
        try:
            self.route_document = load_route(self.frozen_route_file or self.route_file)
            self.route_load_error = None
            known = self.recorded_station_order()
            if known:
                self.get_logger().info(f'前进记录站点逆序（供核对）：{[row["id"] for row in known]}；实际横线按本轮独立编号计数')
            else:
                self.get_logger().info('旧路线无站点事件，不能推断总横线数；本轮从横线#1开始记录')
        except (OSError, ValueError, TypeError, KeyError) as error:
            self.route_document = None
            self.route_load_error = str(error)
            self.get_logger().warning(f'倒车路线不可用：{os.path.expanduser(self.route_file)}；{error}')

    def odom_reason(self, now):
        if self.odom_fault:
            return self.odom_fault
        if self.latest_odom is None or now-self.latest_odom['receipt'] > self.route_config.odom_timeout:
            return 'route_odom_timeout'
        age = (self.get_clock().now().nanoseconds-self.latest_odom['stamp'])/1e9
        if not -.1 <= age <= self.route_config.odom_timeout:
            return 'route_odom_stale'
        return None

    def on_odom(self, message):
        now = time.monotonic()
        position, q = message.pose.pose.position, message.pose.pose.orientation
        values = (position.x, position.y, q.x, q.y, q.z, q.w,
                  message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.angular.z)
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        norm = sum(v*v for v in (q.x, q.y, q.z, q.w))
        if (not all(math.isfinite(v) for v in values) or not math.isfinite(norm) or norm < 1e-8
                or not message.header.frame_id or not message.child_frame_id):
            self.odom_fault = 'route_odom_invalid'
        else:
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y)/norm, 1-2*(q.y*q.y+q.z*q.z)/norm)
            pose = (position.x, position.y, yaw)
            previous = self.latest_odom
            if previous is not None:
                if stamp == previous['stamp']:
                    return  # 重复旧帧不能刷新超时。
                dt = (stamp-previous['stamp'])/1e9
                displacement = math.dist(pose[:2], previous['pose'][:2])
                yaw_delta = abs(wrap(yaw-previous['pose'][2]))
                checks = {
                    'time_backwards': dt <= 0.,
                    'time_gap': dt > 1.,
                    'frame_changed': message.header.frame_id != previous['frame'] or message.child_frame_id != previous['child'],
                    'position_jump': dt > 0. and displacement > .1+.5*dt,
                    'yaw_jump': dt > 0. and yaw_delta > .2+1.5*dt,
                }
                if any(checks.values()):
                    if self.odom_fault is None:
                        self.odom_fault_detail = dict(
                            triggers=[name for name, triggered in checks.items() if triggered],
                            source_dt_sec=dt, receipt_gap_sec=now-previous['receipt'],
                            displacement_m=displacement, yaw_delta_rad=yaw_delta,
                            previous_frame=previous['frame'], current_frame=message.header.frame_id,
                            previous_child=previous['child'], current_child=message.child_frame_id)
                        self.get_logger().error('里程计不连续，路线记录停止：'+json.dumps(self.odom_fault_detail))
                    self.odom_fault = 'route_odom_discontinuity'
            if self.odom_fault is None and previous is not None and self.enabled:
                self.marker_gate.record_odometry_distance(math.dist(pose[:2], previous['pose'][:2]))
            self.latest_odom = {'pose': pose, 'stamp': stamp, 'receipt': now,
                                'frame': message.header.frame_id, 'child': message.child_frame_id,
                                'v': values[-3], 'vy': values[-2], 'w': values[-1]}
            if self.route_checkpoint is not None and not self.enabled:
                checkpoint = self.route_checkpoint
                if (message.header.frame_id != checkpoint['frame'] or message.child_frame_id != checkpoint['child']
                        or math.dist(pose[:2], checkpoint['pose'][:2]) > .025
                        or abs(wrap(pose[2]-checkpoint['pose'][2])) > .05):
                    self.checkpoint_invalid_reason = 'saved_route_endpoint_moved_or_odom_reset'
            if (self.recorder is not None and self.config.drive_direction == 'forward'
                    and (self.enabled or (self.record_stop_at is not None and now < self.record_stop_at))):
                if not self.recorder.points:
                    self.recorder.frame, self.recorder.child = message.header.frame_id, message.child_frame_id
                if self.odom_fault is None and self.recorder.fault is None:
                    was_finished = self.recorder.finished
                    if (abs(values[-3]) >= .01 or abs(values[-2]) >= .01 or abs(values[-1]) >= .05
                            or self.enabled and not self.marker_gate.arrived):
                        self.recorder.finished = False
                    self.recorder.add(pose)
                    if (now-self.last_route_save >= 1. or was_finished and not self.recorder.finished
                            or (not self.enabled or self.marker_gate.arrived) and not self.recorder.finished):
                        self.flush_route()
        if self.odom_fault:
            if self.recorder is not None and self.recorder.fault != self.odom_fault:
                self.recorder.fault = self.odom_fault
                self.flush_route()
            if self.uses_recorded_route() or self.record_route:
                self.stop()

    def return_visual_feedback(self):
        """倒车只读靠近车头的局部切线，不以车前远端弯道代表车尾路线。"""
        if self.route_config.visual_strafe:
            return self.reverse_feedback
        if self.return_geometry_reason is None and self.return_geometry is not None:
            geometry = self.return_geometry
            heading = geometry.get('near_heading_rad')
            if heading is not None and math.isfinite(heading):
                width = self.status.get('image_width', 0)
                if width <= 1:
                    return None
                forward = -1. if self.config.image_forward_direction == 'up' else 1.
                # 同一局部切线投影到既有虚拟车身参考行，去除前置观测点的朝向耦合。
                # 这是像素几何估计，不是地面标定后的米制车尾位置。
                offset = (forward*math.tan(heading)*(geometry['near_y']-geometry['reference_y'])
                          / ((width-1)/2))
                return {'error_normalized': geometry['error_normalized']-offset,
                        'observed_error_normalized': geometry['error_normalized'],
                        'near_heading_rad': heading, 'source': 'near_measured',
                        'span_px': abs(geometry['trusted_end_y']-geometry['near_y'])}
        return None

    def finish_marker_is_final(self):
        # 提前视觉接管只改变转向来源；原30cm终点范围之外仍保留途中站语义。
        if self.config.reverse_return_to_start:
            stations = self.recorded_station_order()
            if any(event.get('action') == 'ignored_departure'
                   for event in (self.route_document or {}).get('station_events', []) if isinstance(event, dict)):
                number = self.marker_gate.current_identity or self.marker_gate.ledger.counts['encountered']+1
                if number != len(stations):
                    return False
        return (self.return_finish is not None and self.return_tracker is not None
                and self.return_tracker.diagnostics.get('remaining_m', math.inf) <= self.route_config.finish_distance)

    def visual_finish_command(self, pose, dt):
        guard = self.return_finish
        fault = guard.check(pose)
        if fault is not None:
            return 0., 0., fault
        if self.route_config.visual_strafe:
            return self.strafe_finish_command()
        if self.marker_gate.arrived:
            if self.marker_gate.latched_kind != 'final':
                return 0., 0., 'stop_marker_intermediate'
            # 横线到位也必须有新鲜的居中/朝向确认；不靠继续越线来修正横向。
            feedback = self.return_visual_feedback()
            aligned = (feedback is not None and self.finish_goal_frames >= 3
                       and self.route_config.finish_alignment_ok(
                           feedback['error_normalized'], feedback['near_heading_rad']))
            guard.diagnostics.update(goal_aligned=aligned, goal_alignment_frames=self.finish_goal_frames,
                                     goal_lateral_limit=self.route_config.finish_goal_lateral,
                                     goal_heading_limit_rad=self.route_config.finish_goal_heading)
            if not aligned:
                return 0., 0., 'reverse_finish_goal_not_centered'
            guard.arrived = True
            return 0., 0., 'reverse_route_complete'
        if self.return_geometry_reason is not None or self.return_observation is None:
            guard.curvature = 0.
            return 0., 0., 'reverse_finish_line_unavailable'
        self.geometry = self.return_geometry
        feedback = self.return_visual_feedback()
        if feedback is None:
            guard.curvature = 0.
            return 0., 0., 'reverse_finish_line_unavailable'
        near_heading = feedback['near_heading_rad']
        guard.diagnostics.update(visual_error_normalized=feedback['error_normalized'],
                                 visual_heading_rad=near_heading, visual_source=feedback['source'],
                                 alignment_error_limit=self.route_config.finish_exit_lateral,
                                 alignment_heading_limit_rad=self.route_config.finish_exit_heading,
                                 alignment_exceeded=[name for name, value, limit in (
                                     ('lateral', feedback['error_normalized'], self.route_config.finish_exit_lateral),
                                     ('heading', near_heading, self.route_config.finish_exit_heading))
                                     if abs(value) > limit])
        if guard.diagnostics['alignment_exceeded']:
            guard.curvature = 0.
            return 0., 0., 'reverse_finish_alignment_out_of_range'
        # 末段回中偏角有界；相机安装方向不因倒车而反转。
        control = replace(self.config, forward_speed=min(self.config.forward_speed, self.route_config.finish_speed),
                          reverse_max_speed=self.route_config.finish_speed,
                          reverse_steering_sign=self.route_config.finish_lateral_sign,
                          steering_gain=self.route_config.finish_center_gain,
                          reverse_heading_gain=self.route_config.finish_heading_gain,
                          error_deadband=self.route_config.finish_center_tolerance, heading_deadband_rad=.02,
                          curve_error_deadband=max(self.config.curve_error_deadband, self.route_config.finish_center_tolerance),
                          max_angular_speed=.06, curve_max_angular_speed=.06)
        observation = dict(self.return_observation, **feedback, curve_strength=0.)
        v, w, reason = reverse_visual_command(observation, control, self.route_config.finish_goal_heading)
        if reason != 'tracking':
            guard.curvature = 0.
            return 0., 0., 'reverse_finish_'+reason
        cap = self.route_config.max_curvature
        reference = self.return_tracker.reference_curvature()
        # 同时记录各项，便于区分横向方向错误、朝向抵消和路径前馈影响。
        side = math.copysign(max(0., abs(feedback['error_normalized'])-control.error_deadband), feedback['error_normalized'])
        heading = math.copysign(max(0., abs(near_heading)-control.heading_deadband_rad), near_heading)
        image_sign = control.steering_sign if control.image_forward_direction == 'up' else -control.steering_sign
        desired_heading = max(-self.route_config.finish_goal_heading, min(self.route_config.finish_goal_heading,
                              control.reverse_steering_sign*math.atan(control.steering_gain*side)/control.reverse_heading_gain))
        lateral_yaw = -image_sign*control.reverse_heading_gain*desired_heading
        heading_yaw = image_sign*control.reverse_heading_gain*heading
        target = max(-cap, min(cap, reference+w/v)) if v else 0.
        # 中等朝向偏差先连续降速纠偏；只有明显越出恢复范围才硬停车。
        fraction = max(0., min(1., (abs(near_heading)-self.route_config.finish_entry_heading)/
                                     (self.route_config.finish_exit_heading-self.route_config.finish_entry_heading)))
        v *= 1.-.5*fraction
        if not self.route_config.finish_alignment_ok(feedback['error_normalized'], near_heading):
            v *= .5  # 末段未摆正时留出低速纠偏距离，最终条件与途中容错分开。
        guard.diagnostics.update(control_terms_valid=True, reference_curvature_per_m=reference,
                                 desired_heading_rad=desired_heading,
                                 observed_error_normalized=feedback['observed_error_normalized'],
                                 lateral_yaw_radps=lateral_yaw, heading_yaw_radps=heading_yaw,
                                 feedback_yaw_radps=w, target_curvature_per_m=target,
                                 lateral_sign=control.reverse_steering_sign,
                                 visual_heading_rad=near_heading, visual_speed_factor=1.-.5*fraction)
        guard.curvature += dt/(.25+dt)*(target-guard.curvature)
        return v, v*guard.curvature, 'reverse_visual_finish_tracking'

    def strafe_finish_command(self):
        """先横移回中并摆正，再纵向靠横线；到横线后只锁住纵向。"""
        self.geometry = self.return_geometry
        feedback = self.return_visual_feedback()
        if feedback is None:
            return 0., 0., 'reverse_finish_line_unavailable'
        route = self.route_config
        error_out = (abs(feedback['lateral_error_m']) > route.max_tracking_error
                     if feedback.get('metric') else abs(feedback['error_normalized']) > route.finish_exit_lateral)
        if error_out or abs(feedback['near_heading_rad']) > route.finish_exit_heading:
            return 0., 0., 'reverse_finish_visual_out_of_range'
        if self.marker_gate.arrived and self.marker_gate.latched_kind != 'final':
            return 0., 0., 'stop_marker_intermediate'
        terminal = self.marker_gate.arrived
        stopped = (abs(self.latest_odom['v']) < .005 and abs(self.latest_odom['vy']) < .005
                   and abs(self.latest_odom['w']) < .015)
        phase = self.alignment_sequencer.update(feedback, route, self.return_camera_stamp,
                                                time.monotonic(), terminal=terminal, stopped=stopped)
        if phase != self.last_alignment_phase:
            self.last_alignment_phase = phase
            labels = dict(settling='等待新帧确认/停稳', heading='只摆正', lateral='只横移', ready='恢复倒车')
            self.get_logger().info('倒车对齐阶段：'+labels[phase])
        vy, w = strafe_command(feedback, self.config, route)
        centered = visually_aligned(feedback, route) and self.finish_goal_frames >= 3
        self.reverse_vy = vy if phase == 'lateral' else 0.
        w = w if phase == 'heading' else 0.
        self.return_finish.diagnostics.update(goal_aligned=centered, goal_alignment_frames=self.finish_goal_frames,
                                              feedback=feedback, lateral_velocity_mps=self.reverse_vy,
                                              alignment_phase=phase)
        if centered and terminal and stopped and phase == 'ready':
            self.return_finish.arrived = True
            return 0., 0., 'reverse_route_complete'
        if phase == 'ready' and not terminal:
            self.visual_alignment_since = None
            return -min(self.config.forward_speed, route.finish_speed), 0., 'reverse_visual_finish_tracking'
        if self.visual_alignment_since is None:
            self.visual_alignment_since = time.monotonic()
        if time.monotonic()-self.visual_alignment_since > 20.:
            self.reverse_vy = 0.
            self.return_fault = 'reverse_finish_alignment_timeout'
            return 0., 0., self.return_fault
        return 0., w, 'reverse_visual_finish_aligning'

    def tick_return_route(self, now, dt):
        self.reverse_vy = 0.
        self.geometry = None
        reason = self.return_fault or self.blocked_reason(now, check_enabled=False)
        if self.return_finish is not None and self.return_finish.arrived:
            reason = 'reverse_route_complete'
        v = w = 0.
        if reason is None:
            if self.return_tracker is None and self.enabled:
                pose = self.latest_odom['pose']
                if abs(self.latest_odom['v']) > .01 or abs(self.latest_odom['vy']) > .01 or abs(self.latest_odom['w']) > .05:
                    reason = 'reverse_start_not_stationary'
                else:
                    self.return_tracker = ReverseRouteFollower(self.route_document, pose, self.route_config)
                    self.get_logger().info(
                        f'记录路线倒车：{self.route_document["length_m"]:.3f}m，终点对齐当前里程计；'
                        '须保持前进停车后的实际位置和朝向，不使用相机倒车反号参数')
            if self.return_tracker is not None:
                v, w, reason = self.return_tracker.command(self.latest_odom['pose'],
                    min(self.config.forward_speed, self.route_config.max_speed))
                if (self.return_finish is None and self.route_visual_finish and self.return_camera_stop_enabled()
                        and reason in ('reverse_route_tracking', 'reverse_route_complete')
                        and self.return_tracker.diagnostics['remaining_m'] <= self.route_config.finish_straight_distance
                        and self.finish_straight_frames >= 3
                        and self.return_tracker.remaining_is_gentle()
                        and abs(self.previous_w) <= self.route_config.finish_entry_yaw_rate
                        and abs(self.latest_odom['w']) <= self.route_config.finish_entry_yaw_rate):
                    try:
                        self.finish_entry_reason = 'confirmed_visual_path'
                        self.return_finish = ReverseFinishGuard(self.return_tracker, self.latest_odom['pose'], self.route_config)
                        if abs(self.previous_v) > .005:
                            cap = self.route_config.max_curvature
                            self.return_finish.curvature = max(-cap, min(cap, self.previous_w/self.previous_v))
                        self.get_logger().info(
                            f'进入最后相机倒车：近端可纠偏、路线曲率可行、转向速率已减小；'
                            f'最多超过记录起点{self.route_config.finish_extra_distance*100:.0f}cm')
                    except ValueError as error:
                        self.return_fault = reason = str(error)
                        v = w = 0.
                if self.return_finish is not None:
                    v, w, reason = self.visual_finish_command(self.latest_odom['pose'], dt)
                elif self.route_visual_finish and self.return_camera_stop_enabled():
                    feedback = self.return_visual_feedback()
                    if (self.route_config.visual_strafe and feedback is not None and feedback.get('metric')
                            and abs(feedback['near_heading_rad']) <= self.route_config.finish_entry_heading
                            and abs(self.return_tracker.reference_curvature()) < .5):
                        vy, _ = strafe_command(feedback, self.config, self.route_config)
                        self.reverse_vy = max(-.01, min(.01, vy))
                    if reason == 'reverse_route_complete':
                        v = w = 0.
                        reason = 'reverse_visual_entry_unconfirmed'
                    elif reason == 'reverse_route_tracking' and self.return_tracker.diagnostics['remaining_m'] <= self.route_config.finish_distance:
                        # 接管前留出慢速对齐距离，不能到30cm就无条件切换。
                        scale = min(1., self.route_config.finish_speed/abs(v)) if v else 0.
                        v, w = v*scale, w*scale
                if self.return_finish is None and reason in ('reverse_route_deviation', 'reverse_target_not_behind'):
                    self.return_fault = reason
            elif reason is None:
                reason = 'reverse_route_ready'
        self.input_reason = reason
        state = reason if self.enabled else 'disabled'
        moving = self.enabled and reason in ('reverse_route_tracking', 'reverse_visual_finish_tracking', 'reverse_visual_finish_aligning')
        if self.enabled and self.return_camera_stop_enabled():
            if self.marker_gate.arrived and reason != 'reverse_route_complete' and self.return_finish is None:
                state, moving = 'stop_marker_intermediate', False
            elif moving and reason != 'reverse_visual_finish_aligning':
                limited, marker_state = self.marker_gate.limit(v, self.config, now=now)
                w *= abs(limited/v) if v else 0.
                v = limited
                if marker_state != 'searching':
                    state = 'stop_marker_'+marker_state.removeprefix('stopped_')
                    moving = marker_state in ('approaching', 'missing_approaching')
        if not moving:
            v = w = 0.
        else:
            target_v = v
            if self.route_config.visual_strafe and self.return_finish is not None:
                v = slew(self.previous_v, v, self.config.linear_acceleration, dt) if v else 0.
                w = slew(self.previous_w, w, self.config.angular_acceleration, dt) if w else 0.
            else:
                v, w = coordinated_velocity(self.previous_v, self.previous_w, v, w, self.config, dt)
            if abs(v) > abs(target_v):
                w *= abs(target_v/v)
                v = target_v
            if not (self.route_config.visual_strafe and self.return_finish is not None):
                cap = abs(v)*self.route_config.max_curvature
                w = max(-cap, min(cap, w))
        if self.enabled and self.return_camera_stop_enabled():
            self.marker_gate.record_motion(v, dt, self.config)
        if moving and self.reverse_vy:
            self.previous_vy += max(-.04*dt, min(.04*dt, self.reverse_vy-self.previous_vy))
        else:
            self.previous_vy = 0.
        self.previous_v, self.previous_w = v, w
        if moving and self.return_finish is not None and reason == 'reverse_visual_finish_tracking':
            parts = self.return_finish.diagnostics
            # 此处是限速、横线门控之后真正发布的指令，不把控制器期望值当成实际输出。
            if parts.get('control_terms_valid'):
                parts['last_tracking_command'] = {
                    'time_unix': time.time(), 'linear_x_mps': v, 'angular_z_radps': w,
                    **{key: parts[key] for key in (
                        'visual_error_normalized', 'visual_heading_rad', 'visual_source',
                        'observed_error_normalized', 'desired_heading_rad',
                        'lateral_yaw_radps', 'heading_yaw_radps', 'feedback_yaw_radps',
                        'reference_curvature_per_m', 'target_curvature_per_m')}}
        if moving and self.return_finish is not None and self.route_config.visual_strafe:
            self.return_finish.diagnostics['last_tracking_command'] = dict(
                time_unix=time.time(), linear_x_mps=v, linear_y_mps=self.previous_vy,
                angular_z_radps=w, feedback=self.return_visual_feedback())
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = v, self.previous_vy, w
        self.cmd_pub.publish(command)
        if state != self.reason:
            self.reason = state
            self.get_logger().info(f'Line following state: {state}')
            if state == 'reverse_finish_goal_not_centered':
                self.get_logger().warning(
                    '终点横线已到，但横向/朝向尚未连续3帧达标；保持停车，未报告返程完成。'
                    f'要求偏差≤{self.route_config.finish_goal_lateral:.3f}（半图宽比例），'
                    f'朝向≤{math.degrees(self.route_config.finish_goal_heading):.1f}度')
            if state == 'reverse_finish_alignment_out_of_range':
                parts = self.return_finish.diagnostics
                self.get_logger().warning(
                    f'末段倒车偏差保护：触发项={parts["alignment_exceeded"]}；'
                    f'图像横向偏差={parts["visual_error_normalized"]:+.3f}'
                    f'（范围±{parts["alignment_error_limit"]:.3f}，非米）；'
                    f'朝向={math.degrees(parts["visual_heading_rad"]):+.1f}度'
                    f'（范围±{math.degrees(parts["alignment_heading_limit_rad"]):.1f}度）；'
                    '上一条运动指令及观测='+json.dumps(parts.get('last_tracking_command'), ensure_ascii=False))
        if now-self.last_state >= .5:
            self.last_state = now
            detail = self.return_tracker.diagnostics if self.return_tracker is not None else {}
            self.state_pub.publish(String(data=json.dumps({
                'state': state, 'enabled': self.enabled, 'input_state': self.input_reason,
                'drive_direction': 'reverse', 'reverse_mode': 'recorded_route',
                'route_file': os.path.expanduser(self.route_file), 'route_snapshot_file': self.frozen_route_file,
                'checkpoint_error': self.checkpoint_invalid_reason, 'route_error': self.route_load_error,
                'route': detail, 'route_camera_stop': self.return_camera_stop_enabled(),
                'station_counts': dict(self.marker_gate.ledger.counts),
                'stop_marker': self.marker_gate.snapshot(self.config),
                'stop_detection': self.stop_detection,
                'visual_finish': self.return_finish.diagnostics if self.return_finish is not None else None,
                'visual_entry_reason': self.finish_entry_reason, 'visual_straight_frames': self.finish_straight_frames,
                'visual_axis': self.return_visual_feedback(),
                'completion_source': ('camera_stop_marker' if self.return_finish is not None and self.return_finish.arrived else None),
                'command_linear_x_mps': v, 'command_linear_y_mps': self.previous_vy, 'command_angular_z_radps': w,
                'visual_geometry_mode': 'ground_meters' if self.ground_mapping else 'image_uncalibrated'})))
        if self.enabled and self.return_tracker is not None and now-self.last_steering_log >= 5.:
            self.last_steering_log = now
            detail = self.return_tracker.diagnostics
            feedback = self.return_visual_feedback()
            visual = (f'，视觉={feedback["source"]}，图像偏移={feedback["error_normalized"]:+.3f}，'
                      f'近端实测偏移={feedback["observed_error_normalized"]:+.3f}，'
                      f'朝向误差={math.degrees(feedback["near_heading_rad"]):+.1f}度，'
                      f'可纠偏确认={self.finish_straight_frames}/3' if feedback is not None else '，视觉近端线不可用')
            if self.return_finish is not None and not self.route_config.visual_strafe:
                parts = self.return_finish.diagnostics
                terms = parts if parts.get('control_terms_valid') else parts.get('last_tracking_command')
                if terms is not None:
                    label = '本次计算' if parts.get('control_terms_valid') else '上次运动（当前已停）'
                    visual += (f'，{label}：横向项={terms["lateral_yaw_radps"]:+.3f}，'
                               f'朝向项={terms["heading_yaw_radps"]:+.3f}rad/s，'
                               f'路径曲率={terms["reference_curvature_per_m"]:+.2f}/m，'
                               f'回中目标偏角={math.degrees(terms["desired_heading_rad"]):+.1f}度')
            self.get_logger().info(
                f'倒车返程：state={state}，剩余弧长={detail.get("remaining_m", 0.):.3f}m，'
                f'距起点={detail.get("goal_distance_m", 0.):.3f}m，指令速度={v:.3f}m/s，'
                f'横移速度={self.previous_vy:+.3f}m/s，角速度={w:+.3f}rad/s'+visual)
        if self.return_tracker is not None and now-self.last_route_log >= 1.:
            self.last_route_log = now
            path = NavPath()
            path.header.frame_id = self.latest_odom['frame']
            path.header.stamp = self.get_clock().now().to_msg()
            points = self.return_tracker.points
            for x, y in points[::max(1, len(points)//500)]:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y = x, y
                pose.pose.orientation.w = 1.
                path.poses.append(pose)
            self.route_pub.publish(path)

    def on_marker_mask(self, message):
        if self.uses_recorded_route():
            self.stage_return_camera('mask', message)
            return
        if self.stage_frame('mask', message):
            return
        self.accept_marker_mask(message)

    @staticmethod
    def detect_marker_frame(request):
        message, points, config, reference, shape = request
        mask = CvBridge().imgmsg_to_cv2(message, desired_encoding='mono8')
        if shape is not None and mask.shape != shape:
            raise ValueError('stop mask resolution mismatch')
        diagnostics = {}
        started = time.monotonic()
        candidates = detect_stop_markers(mask, points, config, reference, diagnostics=diagnostics)
        diagnostics['detector_processing_ms'] = round((time.monotonic()-started)*1000, 1)
        return message.header, config, candidates, diagnostics, mask.shape[0]

    def accept_marker_mask(self, message):
        if not self.config.stop_marker_enabled:
            return
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        if stamp == self.marker_stamp:
            return
        self.marker_stamp = stamp
        shape = ((self.status.get('image_height'), self.status.get('image_width'))
                 if isinstance(self.status, dict) else None)
        self.marker_worker.submit((message, list(self.path_points), self.config,
                                   copy.deepcopy(self.marker_gate.reference_marker), shape))

    def collect_marker_result(self):
        result = self.marker_worker.take()
        if result is None:
            return
        _, observation, error = result
        if error is not None:
            self.marker_time = None
            self.get_logger().warning(f'横线检测失败：{error}')
            return
        header, config, candidates, diagnostics, height = observation
        stamp = header.stamp.sec*10**9+header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        previous = (self.marker_header.stamp.sec*10**9+self.marker_header.stamp.nanosec
                    if self.marker_header is not None else -1)
        # 参数/使能切换后的旧结果不能重新武装停车状态；过期结果不刷新新鲜度。
        if config is not self.config or stamp <= previous or not -.1 <= age <= self.input_timeout:
            return
        diagnostics['result_age_ms'] = round(age*1000, 1)
        self.marker_candidates, self.stop_detection = candidates, diagnostics
        if self.return_camera_stop_enabled():
            self.marker_candidates = [dict(marker, kind='final' if self.finish_marker_is_final() else 'intermediate')
                                      for marker in candidates]
        if self.enabled:
            self.marker_gate.observe(self.marker_candidates, height, self.config)
        self.marker_header = header
        self.marker_time = time.monotonic()

    def check_route_writer(self):
        result = self.route_writer.take()
        if result is not None and result[2] is not None:
            if self.recorder is not None:
                self.recorder.fault = 'route_save_failed'
            self.get_logger().error(f'路线保存失败：{result[2]}')
            self.stop()
        elif result is not None:
            document = result[1]
            finished = document['valid']
            if finished and not self.route_saved_finished:
                message = (f'前进路线已停稳保存：{os.path.expanduser(self.route_file)}；'
                           f'{document["length_m"]:.3f}m')
                if rclpy.ok():
                    self.get_logger().info(message)
                else:
                    print(message, flush=True)
            self.route_saved_finished = finished

    def on_resume_marker(self, _message):
        if (self.enabled and self.config.stop_marker_enabled and self.blocked_reason(time.monotonic(), check_enabled=False) is None
                and self.marker_gate.resume(self.config)):
            if self.recorder is not None:
                self.recorder.finished = False
                self.flush_route(wait=True)  # 在继续发运动指令前撤销暂停快照，不能把行驶中路线用于倒车。
            self.steering_guard.reset()
            self.get_logger().info('收到新的继续消息：离开途中停止点，继续循迹')
        else:
            self.get_logger().info('忽略继续消息：尚未停在途中站点，或当前输入/底盘不可用')

    def on_exposure_status(self, message):
        try:
            value = json.loads(message.data)
            self.exposure_status = value if isinstance(value, dict) else None
        except ValueError:
            self.exposure_status = None

    def on_parameters(self, parameters):
        updates = {}
        enabled = self.enabled
        try:
            for parameter in parameters:
                if parameter.name in vars(self.config):
                    updates[parameter.name] = parameter.value
                elif parameter.name == 'enabled':
                    enabled = parameter.value
            candidate = replace(self.config, **updates)
            candidate.validate()
            if self.enabled and (candidate.drive_direction != self.config.drive_direction
                    or candidate.reverse_steering_sign != self.config.reverse_steering_sign):
                raise ValueError('先停止并禁用循迹，再切换 drive_direction 或 reverse_steering_sign')
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        if (not self.enabled and enabled and candidate.drive_direction == 'reverse'
                and self.checkpoint_invalid_reason is not None):
            return SetParametersResult(successful=False, reason=self.checkpoint_invalid_reason)
        if not self.enabled and enabled and candidate.drive_direction == 'forward' and self.record_route:
            self.frozen_route_file = self.route_checkpoint = self.checkpoint_invalid_reason = None
            recorder = RouteRecorder(self.route_config, 'odom', 'base_link')
            try:
                ticket = self.route_writer.submit((self.route_file, recorder.document()))
                self.route_writer.wait(ticket)
                result = self.route_writer.take()
                if result is not None and result[2] is not None:
                    raise OSError(str(result[2]))
            except (OSError, ValueError, TimeoutError) as error:
                return SetParametersResult(successful=False, reason=f'route_save_failed: {error}')
            self.recorder = recorder
            self.route_saved_finished = False
            self.record_stop_at = None
            self.get_logger().info(f'开始记录前进路线：{os.path.expanduser(self.route_file)}')
        if self.enabled and not enabled:
            self.record_stop_at = time.monotonic()+1.
            self.flush_route(finish=True, wait=True)
        if (candidate.drive_direction == 'reverse' and
                (not self.enabled and enabled or candidate.drive_direction != self.config.drive_direction)):
            self.flush_route(wait=True)
            self.load_return_route()
        marker_was_enabled = self.config.stop_marker_enabled
        self.config = candidate
        if ((candidate.stop_marker_enabled and not marker_was_enabled) or updates.keys() & {
                'stop_marker_target_ratio', 'stop_marker_average_frames',
                'stop_marker_average_max_age_sec', 'stop_marker_position_tolerance_ratio',
                'stop_marker_overshoot_tolerance_ratio', 'forward_ignore_start_marker', 'reverse_ignore_start_marker',
                'reverse_start_marker_max_travel',
                'image_forward_direction', 'drive_direction'}):
            self.marker_gate.reset()
            self.marker_time = self.marker_stamp = None
            self.return_camera_frames.clear()
            self.finish_straight_frames = self.finish_goal_frames = self.strafe_entry_frames = 0
            self.return_axis = None
            self.return_camera_stamp = -1
        if enabled and not self.enabled:
            self.marker_gate.reset()
            self.station_run_events = []
        if enabled != self.enabled or updates.keys() & {
                "image_forward_direction", "drive_direction", "reverse_steering_sign",
                "vehicle_reference_x_ratio", "vehicle_reference_y_ratio"}:
            self.good_frames = 0
            self.recovery_deadline = None
        self.enabled = enabled
        if not enabled or candidate.forward_speed == 0.0:
            self.stop()
        return SetParametersResult(successful=True)

    def on_enable(self, request, response):
        results = self.set_parameters([rclpy.parameter.Parameter('enabled', value=request.data)])
        response.success = results[0].successful
        response.message = 'enabled' if self.enabled else 'disabled'
        return response

    def on_save_route(self, request, response):
        """停稳后封存返程路线；后续机械臂/TCP等待不修改已封存的文件。"""
        del request
        if (self.enabled or self.config.drive_direction != 'forward' or self.recorder is None
                or self.odom_reason(time.monotonic()) is not None):
            response.success, response.message = False, 'disable forward following and wait for fresh stationary odometry first'
            return response
        odom = self.latest_odom
        if abs(odom['v']) > .005 or abs(odom['vy']) > .005 or abs(odom['w']) > .015:
            response.success, response.message = False, 'chassis has not stopped'
            return response
        try:
            self.flush_route(finish=True, wait=True)
            document = load_route(self.route_file)
            checkpoint = dict(pose=list(odom['pose']), frame=odom['frame'], child=odom['child'])
            document['checkpoint'] = checkpoint
            filename = Path(self.route_file).expanduser()
            snapshot = filename.parent/'snapshots'/(filename.stem+'_'+uuid4().hex+'.json')
            ticket = self.route_writer.submit((str(snapshot), document))
            self.route_writer.wait(ticket)
            self.check_route_writer()
            load_route(str(snapshot))
            self.frozen_route_file, self.route_checkpoint = str(snapshot), checkpoint
            self.checkpoint_invalid_reason = None
            self.recorder, self.record_stop_at = None, None
            response.success = True
            response.message = json.dumps(dict(snapshot_file=str(snapshot), length_m=document['length_m']), ensure_ascii=False)
            self.get_logger().info('返程路线已封存，等待机械臂/TCP期间不再追加：'+str(snapshot))
        except (OSError, ValueError, TimeoutError) as error:
            response.success, response.message = False, 'route checkpoint failed: '+str(error)
        return response

    def on_ready(self, request, response):
        """Check real inputs without enabling the follower or issuing motion."""
        del request
        if self.uses_recorded_route():
            if self.route_document is None:
                self.load_return_route()
            reason = self.blocked_reason(time.monotonic(), check_enabled=False)
            response.success = reason is None
            response.message = ('recorded route and odometry ready' if reason is None else
                                f'{reason}; route={self.route_file}; detail={self.route_load_error}')
            return response
        reason = self.blocked_reason(time.monotonic(), check_enabled=False)
        if reason is None:
            observation = dict(self.status) if isinstance(self.status, dict) else {}
            geometry, reason = tracking_geometry(
                self.path_points, observation.get('image_width'),
                observation.get('image_height'), self.config)
            if reason is None:
                observation = self.local_observation(observation, geometry)
                _, _, reason = line_command(observation, self.config)
                if reason == 'tracking' and self.ready_frames < self.reacquire_frames:
                    reason = 'reacquiring'
        response.success = reason == 'tracking'
        response.message = 'chassis and USB tracking inputs ready' if response.success else str(reason)
        return response

    @staticmethod
    def local_observation(status, geometry):
        """按近端实测段及已通过两端几何验证的续接段判断可行性。"""
        observation = dict(status)
        observation.update({key: geometry[key] for key in (
            'error_normalized', 'heading_error_rad', 'target_fallback', 'trusted_speed_factor',
            'path_heading_rad', 'near_heading_rad', 'curve_strength')})
        fraction = observation.get('valid_row_fraction')
        if (not isinstance(fraction, bool) and isinstance(fraction, (int, float))
                and math.isfinite(fraction) and 0 <= fraction <= 1):
            observation['valid_row_fraction'] = min(fraction, geometry['trusted_span_ratio'])
        observation['curve_geometry_valid'] = True
        observation['target_estimated'] = False
        observation['gap_connection_confirmed'] = bool(geometry['bridged_gaps'])
        if observation['gap_connection_confirmed'] or geometry['lookahead_supported']:
            # 前瞻已有实测支撑或缺口已验证时，不因上游疑似遮挡重复降速。
            observation['occlusion_suspected'] = False
        return observation

    def stage_frame(self, kind, message):
        """按源图时间戳收齐状态/中线/掩膜再提交，禁止用新掩膜配上一帧中线。"""
        if kind == 'status':
            stamp = (message.get('source_stamp_sec'), message.get('source_stamp_nanosec'))
            if any(type(value) is not int for value in stamp):
                return False  # 兼容旧识别节点，但新节点始终携带源图时间。
            self.frame_sync = True
        else:
            stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        key = stamp[0]*10**9+stamp[1]
        age = (self.get_clock().now().nanoseconds-key)/1e9
        if self.frame_sync and key <= self.committed_stamp:
            return True  # 已提交的新观测不能被后到的旧消息置为过期故障。
        if self.frame_sync and not -.1 <= age <= self.input_timeout:
            self.frame_fault = 'stale_image_stamp'
            self.stop()
            return True
        entry = self.frame_cache.setdefault(key, {})
        entry[kind] = message
        # 只缓存3帧，掩膜用原ROS消息，不复制整幅图像。
        while len(self.frame_cache) > 3:
            self.frame_cache.pop(min(self.frame_cache))
        required = {'status', 'path', 'mask'} if self.config.stop_marker_enabled else {'status', 'path'}
        if self.frame_sync and key > self.committed_stamp and required <= entry.keys():
            self.committed_stamp = key
            self.frame_fault = None
            self.accept_path(entry['path'])
            self.status = entry['status']
            if self.config.stop_marker_enabled:
                self.accept_marker_mask(entry['mask'])
            self.accept_status(entry['status'])
            self.frame_cache = {stamp: frame for stamp, frame in self.frame_cache.items() if stamp > key}
        return self.frame_sync

    def on_status(self, message):
        if self.uses_recorded_route():
            if self.route_visual_finish:
                try:
                    status = json.loads(message.data)
                    if isinstance(status, dict):
                        self.stage_return_camera('status', status)
                except (ValueError, TypeError):
                    pass
            return
        try:
            status = json.loads(message.data)
        except (ValueError, TypeError):
            status = None
        if isinstance(status, dict) and self.stage_frame('status', status):
            # 明确的丢线/超时不等待其他通道，立即零速。
            key = status['source_stamp_sec']*10**9+status['source_stamp_nanosec']
            if key >= self.committed_stamp and (
                    status.get('lost') is True or status.get('processing_over_budget') is True):
                self.good_frames = self.ready_frames = 0
                self.recovery_deadline = None
                self.stop()
            return
        self.accept_status(status)

    def accept_status(self, status):
        try:
            candidate = dict(status) if isinstance(status, dict) else status
            geometry_reason = None
            if isinstance(candidate, dict):
                geometry, geometry_reason = tracking_geometry(
                    self.path_points, candidate.get('image_width'),
                    candidate.get('image_height'), self.config)
                candidate['curve_geometry_valid'] = (
                    geometry_reason is None and self.path_time is not None
                    and time.monotonic() - self.path_time <= self.input_timeout)
                if candidate['curve_geometry_valid']:
                    candidate = self.local_observation(candidate, geometry)
            _, _, reason = line_command(candidate, self.config)
            if geometry_reason is not None:
                reason = geometry_reason
        except (ValueError, TypeError):
            status, reason = None, 'invalid_json'
        self.status = status
        self.status_time = time.monotonic()
        if reason == 'tracking' and self.blocked_reason(self.status_time, check_enabled=False) is None:
            self.ready_frames = min(self.reacquire_frames, self.ready_frames + 1)
        else:
            self.ready_frames = 0
        if reason == 'tracking':
            quick_recovery = (self.enabled and self.recovery_deadline is not None
                              and self.status_time <= self.recovery_deadline)
            self.good_frames = (self.reacquire_frames if quick_recovery else
                                min(self.reacquire_frames, self.good_frames + 1))
            self.recovery_deadline = None
        else:
            if (self.enabled and self.good_frames >= self.reacquire_frames
                    and reason in ('insufficient_rows', 'insufficient_forward_span', 'no_forward_line')):
                self.recovery_deadline = self.status_time+min(.25, self.input_timeout)
            elif reason not in ('insufficient_rows', 'insufficient_forward_span', 'no_forward_line'):
                self.recovery_deadline = None
            self.good_frames = 0
            self.stop()

    def on_path(self, message):
        if self.uses_recorded_route():
            self.stage_return_camera('path', message)
            return
        if self.stage_frame('path', message):
            return
        self.accept_path(message)

    def accept_path(self, message):
        self.path_header = message.header
        self.path_time = time.monotonic()
        self.path_points = [(point.x, point.y) for point in message.polygon.points]
        self.path_valid = bool(self.path_points)
        if not self.path_valid:
            self.recovery_deadline = None
            self.ready_frames = 0
            self.good_frames = 0
            self.stop()

    def on_comm(self, message):
        self.connected = message.comm_state == CommStatus.COMM_STATE_CONNECTED
        self.comm_time = time.monotonic()
        if not self.connected and not self.dry_run:
            self.recovery_deadline = None
            self.ready_frames = 0
            self.good_frames = 0
            self.stop()

    def on_chassis(self, message):
        # Park is allowed: the driver's auto_unlock_on_cmd releases it.
        self.chassis_allowed = (
            message.chassis_state in (
                ChassisStatus.CHASSIS_STATE_COMM_CONTROL, ChassisStatus.CHASSIS_STATE_PARK)
            and message.chassis_fault == 0
            and message.obstacle_status_flags == 0
            and not message.excessive_tilt
            and not message.manual_charging
            and not message.dock_charging
        )
        self.chassis_time = time.monotonic()
        if not self.chassis_allowed and not self.dry_run:
            self.recovery_deadline = None
            self.ready_frames = 0
            self.good_frames = 0
            self.stop()

    def blocked_reason(self, now, check_enabled=True):
        if check_enabled and not self.enabled:
            return 'disabled'
        if self.frame_fault and not self.uses_recorded_route():
            return self.frame_fault
        if not self.dry_run and (
                not self.connected or self.comm_time is None
                or now - self.comm_time > self.comm_timeout):
            return 'chassis_disconnected'
        if not self.dry_run and (
                not self.chassis_allowed or self.chassis_time is None
                or now - self.chassis_time > self.input_timeout):
            return 'chassis_not_ready'
        if self.record_route and not self.uses_recorded_route() and self.recorder is not None and self.recorder.fault:
            return 'route_record_failed'
        if self.uses_recorded_route() or self.record_route:
            odom_reason = self.odom_reason(now)
            if odom_reason is not None:
                return odom_reason
        if self.uses_recorded_route():
            if self.route_document is None:
                return 'reverse_route_missing_or_invalid'
            if self.latest_odom['child'] != self.route_document['child_frame_id']:
                return 'reverse_route_frame_mismatch'
            if self.return_camera_stop_enabled():
                return self.return_camera_reason(now)
            return None
        for receipt in (self.status_time, self.path_time):
            if receipt is None or now - receipt > self.input_timeout:
                return 'camera_timeout'
        if self.config.stop_marker_enabled:
            if self.marker_time is None or now-self.marker_time > self.input_timeout or self.marker_header is None:
                return 'stop_marker_mask_timeout'
            stamp = self.marker_header.stamp
            age = (self.get_clock().now().nanoseconds-stamp.sec*10**9-stamp.nanosec)/1e9
            if not -0.1 <= age <= self.input_timeout:
                return 'stop_marker_mask_stale'
        if not self.path_valid or self.path_header is None:
            return 'empty_centerline'
        if self.path_header.frame_id == 'synthetic_demo' and not self.dry_run:
            return 'synthetic_input'
        stamp = self.path_header.stamp
        age = (self.get_clock().now().nanoseconds - stamp.sec * 1_000_000_000
               - stamp.nanosec) / 1e9
        if age < -0.1 or age > self.input_timeout:
            return 'stale_image_stamp'
        return None

    def on_debug_image(self, message):
        if not self.show_window and self.debug_pub.get_subscription_count() == 0:
            return
        now = time.monotonic()
        if now-self.last_debug_time < .2:
            return
        self.last_debug_time = now
        fields = ('window_name', 'config', 'reason', 'geometry', 'control_curve_strength',
                  'exposure_status', 'previous_v', 'previous_w', 'input_reason',
                  'last_blocked_reason', 'marker_candidates', 'reverse_feedback', 'previous_vy')
        snapshot = SimpleNamespace(**{name: copy.deepcopy(getattr(self, name)) for name in fields})
        snapshot.station_counts = dict(self.marker_gate.ledger.counts)
        snapshot.station_number = self.marker_gate.current_identity or self.marker_gate.ignored_identity
        snapshot.cancelled = self.workers_closing
        snapshot.bridge = CvBridge()
        snapshot.debug_pub = self.debug_pub
        snapshot.get_logger = self.get_logger
        recorded, camera_stop = self.uses_recorded_route(), self.return_camera_stop_enabled()
        snapshot.uses_recorded_route = lambda: recorded
        snapshot.return_camera_stop_enabled = lambda: camera_stop
        snapshot.return_tracker = (SimpleNamespace(diagnostics=dict(self.return_tracker.diagnostics))
                                   if self.return_tracker is not None else None)
        self.preview_worker.submit((snapshot, message))

    def draw_preview(self, request):
        snapshot, message = request
        image = LineFollowNode.render_debug_image(snapshot, message)
        if image is not None:
            with self.preview_lock:
                self.preview_frame = image

    def display_preview(self):
        """仅在进程主线程调用 HighGUI；桌面卡顿不会阻塞控制执行器。"""
        if not self.show_window or self.preview_closed.is_set():
            return
        with self.preview_lock:
            image, self.preview_frame = self.preview_frame, None
        try:
            if image is not None:
                if not self.window_created:
                    cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(self.window_name, 1100, 700)
                    self.window_created = True
                cv2.imshow(self.window_name, image)
            if self.window_created and cv2.waitKey(1) & 0xFF in (27, ord('q')):
                self.preview_closed.set()
                cv2.destroyWindow(self.window_name)
                self.window_created = False
        except cv2.error:
            self.preview_closed.set()

    def render_debug_image(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8').copy()
            # ruier_camer_diver's grid is two image rows plus a 64-pixel header.
            width = image.shape[1] // 2
            height = (image.shape[0] - 64) // 2
            if width <= 0 or height <= 0:
                return
            text = (f'CONTROL: {self.reason or "waiting"} / {self.config.drive_direction}'
                    f' / front={self.config.image_forward_direction}')
            if self.config.drive_direction == 'forward' and self.geometry and self.geometry['curve_strength'] > 0:
                text += f' / CURVE={self.control_curve_strength:.2f}'
            if self.exposure_status and self.exposure_status.get('ready'):
                mode = 'adaptive' if self.exposure_status.get('enabled') else 'fixed'
                text += f' / EXP={self.exposure_status.get("exposure")} {mode}'
            cv2.rectangle(image, (0, 0), (image.shape[1], 64), (0, 0, 0), -1)
            cv2.putText(image, text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (255, 255, 255), 1, cv2.LINE_AA)
            geometry = self.geometry
            if geometry is not None:
                def pixel(x, y):
                    return (int(max(0, min(width - 1, x))),
                            int(max(0, min(height - 1, y))) + 64)
                reference = pixel(geometry['reference_x'], geometry['reference_y'])
                near = pixel(geometry['near_x'], geometry['near_y'])
                ground = pixel(0, geometry['ground_start_y'])
                cv2.line(image, ground, (width - 1, ground[1]), (180, 180, 180), 1)
                cv2.putText(image, 'GROUND CHECK START', (5, ground[1] - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1, cv2.LINE_AA)
                if self.config.stop_marker_enabled:
                    stop_y = int(self.config.stop_marker_target_ratio*height)+64
                    cv2.line(image, (0, stop_y), (width-1, stop_y), (255, 255, 0), 1)
                    cv2.putText(image, 'STOP TARGET', (5, stop_y-5), cv2.FONT_HERSHEY_SIMPLEX,
                                .35, (255, 255, 0), 1, cv2.LINE_AA)
                    for marker in self.marker_candidates:
                        center = pixel(marker['x'], marker['y'])
                        cv2.circle(image, center, 9, (0, 0, 255), 2)
                        cv2.putText(image, 'STOP '+marker['kind'], (center[0]+10, center[1]),
                                    cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 0, 255), 1, cv2.LINE_AA)
                target = pixel(geometry['target_x'], geometry['target_y'])
                cv2.line(image, (reference[0], 64), (reference[0], height + 63),
                         (255, 0, 255), 1)
                cv2.drawMarker(image, reference, (255, 0, 255),
                               cv2.MARKER_CROSS, 20, 2)
                cv2.circle(image, near, 8, (255, 255, 0), 2)
                cv2.circle(image, target, 10, (0, 140, 255), 2)
                boundary = pixel(geometry['trusted_end_x'], geometry['trusted_end_y'])
                cv2.drawMarker(image, boundary, (0, 0, 255), cv2.MARKER_TILTED_CROSS, 18, 2)
                cv2.putText(image, 'MEASURED SEGMENT END', (max(0, boundary[0] - 100), boundary[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1, cv2.LINE_AA)
                for gap in geometry['bridged_gaps']:
                    start, end = pixel(*gap['from']), pixel(*gap['to'])
                    cv2.line(image, start, end, (0, 165, 255), 3)
                    cv2.putText(image, 'GAP: JOINED', start,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 165, 255), 1, cv2.LINE_AA)
                reversing = self.config.drive_direction == 'reverse'
                if not reversing:
                    cv2.arrowedLine(image, reference, target, (0, 140, 255), 2)
                elif geometry['near_heading_rad'] is not None:
                    front_sign = -1 if self.config.image_forward_direction == 'up' else 1
                    tangent = pixel(geometry['near_x']+40*math.tan(geometry['near_heading_rad']),
                                    geometry['near_y']+front_sign*40)
                    cv2.line(image, near, tangent, (255, 255, 0), 2)
                cv2.line(image, (reference[0], near[1]), near, (255, 255, 0), 2)
                label = 'FRONT OBSERVATION' if reversing else 'GOAL (not wheel angle)'
                cv2.putText(image, label, (target[0] + 12, target[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 140, 255), 1, cv2.LINE_AA)
                forward = -1 if self.config.image_forward_direction == 'up' else 1
                front = (reference[0], max(64, min(height + 63, reference[1] + forward * 70)))
                cv2.arrowedLine(image, reference, front, (210, 210, 210), 1)
                cv2.putText(image, 'FRONT', front, cv2.FONT_HERSHEY_SIMPLEX,
                            0.4, (210, 210, 210), 1, cv2.LINE_AA)
                if self.previous_v > 0.0:
                    offsets = motion_preview(self.previous_v, self.previous_w, self.config)
                    trail = [(int(max(0, min(width - 1, reference[0] + dx))),
                              int(max(64, min(height + 63, reference[1] + dy))))
                             for dx, dy in offsets]
                    for first, second in zip(trail, trail[1:]):
                        cv2.line(image, first, second, (0, 255, 0), 3)
                    cv2.arrowedLine(image, trail[-5], trail[-1], (0, 255, 0), 3, tipLength=0.5)
                    cv2.putText(image, 'MOTION CMD (schematic)', (reference[0] + 8, front[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1, cv2.LINE_AA)
                detail = (f'CONTROL green=motion; orange=goal; magenta=body; cyan=near; '
                          f'near error={geometry["error_normalized"]:+.3f}; yaw={self.previous_w:+.2f} rad/s')
            else:
                detail = 'CONTROL waiting for a fresh near line; green point is the detector target'
                if self.uses_recorded_route():
                    route = self.return_tracker.diagnostics if self.return_tracker is not None else {}
                    detail = (f'RECORDED RETURN: remaining={route.get("remaining_m", 0.):.2f}m; '
                              f'v={self.previous_v:+.3f}m/s; camera stop={self.return_camera_stop_enabled()}')
            if self.return_camera_stop_enabled():
                feedback = self.reverse_feedback
                if feedback is not None:
                    first, last = feedback['axis_pixels']
                    cv2.line(image, (int(first[0]), int(first[1])+64),
                             (int(last[0]), int(last[1])+64), (255, 255, 0), 3)
                    mode = 'GROUND' if feedback.get('metric') else 'IMAGE UNCALIBRATED'
                    detail = f'LOCAL AXIS / {mode} / vy={self.previous_vy:+.3f}m/s'
                stop_y = int(self.config.stop_marker_target_ratio*height)+64
                cv2.line(image, (0, stop_y), (width-1, stop_y), (255, 255, 0), 1)
                cv2.putText(image, 'REVERSE STOP TARGET', (5, stop_y-5), cv2.FONT_HERSHEY_SIMPLEX,
                            .35, (255, 255, 0), 1, cv2.LINE_AA)
                for marker in self.marker_candidates:
                    cv2.circle(image, (int(marker['x']), int(marker['y'])+64), 9, (0, 0, 255), 2)
            counts = self.station_counts
            detail = (f'BARS={counts["encountered"]} SKIP={counts["ignored"]} STOP={counts["arrived"]} '
                      f'PASS={counts["passed"]} ACTIVE=#{self.station_number or "-"} | '+detail)
            cv2.putText(image, detail, (10, 47), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (255, 255, 255), 1, cv2.LINE_AA)
            diagnosis = (f'INPUT: {self.input_reason} / LAST BLOCK: {self.last_blocked_reason or "-"}'
                         f' / v={self.previous_v:+.3f} m/s')
            cv2.putText(image, diagnosis, (10, 62), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (0, 220, 255), 1, cv2.LINE_AA)
            output = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            output.header = message.header
            if not self.cancelled.is_set():
                self.debug_pub.publish(output)
                return image
        except (CvBridgeError, cv2.error) as error:
            self.get_logger().warning(f'Control preview failed: {error}')

    def stop(self):
        self.previous_v = self.previous_w = self.previous_vy = 0.0
        self.steering_guard.reset()
        self.cmd_pub.publish(Twist())

    def tick(self):
        self.collect_marker_result()
        self.drain_station_events()
        self.check_route_writer()
        preview_result = self.preview_worker.take()
        if preview_result is not None and preview_result[2] is not None:
            self.get_logger().warning(f'预览绘制失败：{preview_result[2]}')
        if self.preview_closed.is_set() and self.show_window:
            self.show_window = False
            self.set_parameters([rclpy.parameter.Parameter('enabled', value=False)])
        now = time.monotonic()
        gap = now-self.last_tick
        if gap > .1 and now-self.last_control_delay_log > 5.:
            self.last_control_delay_log = now
            self.get_logger().warning(f'控制循环调度延迟 {gap*1000:.0f}ms；请检查CPU负载和后台任务耗时')
        dt = min(gap, self.control_period * 2.0)
        self.last_tick = now
        if self.uses_recorded_route():
            self.tick_return_route(now, dt)
            return
        observation = dict(self.status) if isinstance(self.status, dict) else {}
        self.geometry, geometry_reason = tracking_geometry(
            self.path_points, observation.get('image_width'),
            observation.get('image_height'), self.config)
        if any(t is None or now - t > self.input_timeout
               for t in (self.status_time, self.path_time)):
            self.geometry = None
            geometry_reason = 'camera_timeout'
        # 独立计算输入是否可走，禁用后仍显示真实输入状态，便于区分识别与任务使能问题。
        input_reason = self.blocked_reason(now, check_enabled=False)
        v = w = 0.0
        if input_reason is None:
            if geometry_reason is not None:
                input_reason = geometry_reason
            else:
                observation = self.local_observation(observation, self.geometry)
                tau = self.config.curve_entry_time_sec
                alpha = -math.expm1(-dt/tau) if tau > 0 else 1.0
                self.control_curve_strength += alpha * (
                    self.geometry['curve_strength']-self.control_curve_strength)
                observation['curve_strength'] = self.control_curve_strength
                self.geometry['control_curve_strength'] = self.control_curve_strength
                v, w, input_reason = line_command(observation, self.config)
        self.input_reason = input_reason
        raw_yaw = w
        reason = input_reason if self.enabled else 'disabled'
        if self.enabled:
            if input_reason != 'tracking':
                self.last_blocked_reason = input_reason
            if reason == 'tracking' and self.good_frames < self.reacquire_frames:
                v = w = 0.0
                reason = 'reacquiring'
        else:
            self.good_frames = 0
        if self.enabled and self.config.stop_marker_enabled and self.marker_gate.arrived:
            # 停车状态锁定；输入诊断仍通过 input_state 报告，恢复视觉不自动开走。
            v = w = 0.0
            self.steering_guard.reset()
            reason = 'stop_marker_'+self.marker_gate.state(self.config).removeprefix('stopped_')
        elif reason == 'tracking':
            stamp = self.path_header.stamp
            sample = (stamp.sec, stamp.nanosec)
            curvature = self.steering_guard.update(w, self.config, sample, dt,
                error_normalized=observation['error_normalized'],
                curve_strength=self.control_curve_strength)
            marker_state = 'searching'
            if self.config.stop_marker_enabled:
                v, marker_state = self.marker_gate.limit(v, self.config, now=now)
                if marker_state != 'searching':
                    reason = 'stop_marker_'+marker_state.removeprefix('stopped_')
            target_v = v
            yaw_limit = self.config.max_angular_speed+self.control_curve_strength*(
                self.config.curve_max_angular_speed-self.config.max_angular_speed)
            if self.config.drive_direction == 'reverse':
                yaw_limit = min(yaw_limit, self.config.max_angular_speed)
            target_w = max(-yaw_limit, min(yaw_limit, curvature*target_v))
            v, w = coordinated_velocity(self.previous_v, self.previous_w,
                target_v, target_w, self.config, dt, self.control_curve_strength)
            if marker_state != 'searching' and abs(v) > abs(target_v):
                # 停车接近限速是硬上限；同步缩小角速度，避免降线速度却突然打大轮向。
                w *= abs(target_v/v)
                v = target_v
        else:
            v = w = 0.0
            self.steering_guard.reset()
            if self.recovery_deadline is None or now > self.recovery_deadline:
                self.control_curve_strength = 0.0
        self.previous_v, self.previous_w = v, w
        if self.enabled:
            self.marker_gate.record_motion(v, dt, self.config)
        command = Twist()
        command.linear.x, command.angular.z = v, w
        self.cmd_pub.publish(command)
        if (self.enabled and reason in ('tracking', 'stop_marker_approaching', 'stop_marker_missing_approaching')
                and now-self.last_steering_log >= 5.):
            self.last_steering_log = now
            self.get_logger().info(
                f'Steering: raw_w={raw_yaw:+.3f}; cmd_v={v:+.3f}; cmd_w={w:+.3f}; '
                f'curvature={self.steering_guard.curvature:+.2f}; guard={self.steering_guard.state}; '
                f'centering={self.steering_guard.center_ready}; offset={observation.get("error_normalized", 0.):+.3f}; '
                f'trusted_speed={observation.get("trusted_speed_factor", 0.):.2f}; '
                f'occlusion={observation.get("occlusion_suspected")}; fallback={observation.get("target_fallback")}; '
                f'stop_candidates={len(self.marker_candidates)}; stop_rejected={self.stop_detection.get("rejected", {})}')
        if reason != self.reason:
            self.reason = reason
            position = ''
            if reason.startswith('stop_marker_') and self.marker_gate.height:
                gate = self.marker_gate
                if gate.error is not None and gate.raw_error is not None:
                    position = (f'; position_error={gate.error*gate.height:+.2f}px'
                                f'; raw_error={gate.raw_error*gate.height:+.2f}px'
                                f'; mean_frames={len(gate.samples)}/{self.config.stop_marker_average_frames}'
                                f'; tolerance=[-{self.config.stop_marker_overshoot_tolerance_ratio*gate.height:.2f},'
                                f'+{self.config.stop_marker_position_tolerance_ratio*gate.height:.2f}]px')
            if reason in ('no_forward_line', 'insufficient_forward_span', 'insufficient_rows'):
                span = (max(p[1] for p in self.path_points)-min(p[1] for p in self.path_points)
                        if self.path_points else 0.)
                position += f'; points={len(self.path_points)}; span={span:.1f}px; frame_sync={self.frame_sync}'
            self.get_logger().info(f'Line following state: {reason}{position}')
        if now - self.last_state >= 0.5:
            self.last_state = now
            self.state_pub.publish(String(data=json.dumps({
                'state': reason, 'enabled': self.enabled,
                'input_state': self.input_reason, 'last_blocked_reason': self.last_blocked_reason,
                'image_forward_direction': self.config.image_forward_direction,
                'drive_direction': self.config.drive_direction,
                'reverse_steering_sign': self.config.reverse_steering_sign,
                'raw_yaw_target_radps': raw_yaw,
                'steering_guard': self.steering_guard.state,
                'straight_centering': self.steering_guard.center_ready,
                'straight_center_frames': self.steering_guard.center_frames,
                'raw_curvature_per_m': self.steering_guard.raw_curvature,
                'filtered_curvature_per_m': self.steering_guard.curvature,
                'frame_synchronized': self.frame_sync, 'source_stamp_ns': self.committed_stamp,
                'geometry': self.geometry,
                'station_counts': dict(self.marker_gate.ledger.counts),
                'stop_marker': self.marker_gate.snapshot(self.config),
                'stop_detection': self.stop_detection,
                'route_record': {'file': os.path.expanduser(self.route_file),
                                 'points': len(self.recorder.points) if self.recorder else 0,
                                 'finished': self.recorder.finished if self.recorder else False,
                                 'fault': self.recorder.fault if self.recorder else None},
                'forward_speed_mps': self.config.forward_speed,
                'command_linear_x_mps': v, 'command_linear_y_mps': self.previous_vy, 'command_angular_z_radps': w,
                'visual_geometry_mode': 'ground_meters' if self.ground_mapping else 'image_uncalibrated',
            }, separators=(',', ':'))))

    def destroy_node(self):
        self.workers_closing.set()
        try:
            self.drain_station_events()
            self.save_station_log(wait=True)
        except (OSError, ValueError, TimeoutError):
            pass  # 记录失败不能阻塞退出；正常运行阶段由事件写盘结果报告。
        for name in ('marker_worker', 'preview_worker', 'route_writer', 'station_writer'):
            worker = getattr(self, name, None)
            if worker is not None:
                worker.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = executor = thread = None
    failures = []
    done = Event()
    try:
        executor = SingleThreadedExecutor()
        node = LineFollowNode()
        executor.add_node(node)

        def spin_control():
            try:
                executor.spin()
            except (ExternalShutdownException, KeyboardInterrupt):
                pass
            except Exception as error:
                failures.append(error)
            finally:
                done.set()

        thread = Thread(target=spin_control, name='line-control')
        thread.start()
        while rclpy.ok() and not done.wait(.02):
            node.display_preview()
        if failures:
            raise failures[0]
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.)
        if thread is not None and thread.ident is not None:
            thread.join(timeout=2.)
        if node is not None:
            if rclpy.ok():
                node.stop()
            if node.window_created:
                cv2.destroyWindow(node.window_name)
            node.flush_route(finish=True, wait=True)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
