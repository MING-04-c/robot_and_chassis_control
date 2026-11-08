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
from start_alignment import StartLineAligner
from charging_dock import ChargingDock
from line_follow_workers import LatestWorker
from reverse_visual import GroundMapping, near_line, aligned as visually_aligned, strafe_command, AlignmentSequencer
from recorded_route import write_route_document, RouteConfig, RouteRecorder, ReverseRouteFollower, ReverseFinishGuard, load_route, wrap
from line_follow_control import (ControlConfig, SteeringDirectionGuard, coordinated_velocity, cruise_velocity,
                                 line_command, motion_preview, tracking_geometry,
                                 straight_image_axis, reverse_visual_command, slew)


from dock_departure import DockDeparture
from recorded_route import StandardRouteFollower
from forward_navigation import ForwardNavigationConfig, ForwardRouteNavigator, VisualCommand


class LineFollowNode(Node):
    def __init__(self):
        super().__init__('line_follow_controller')
        cv_threads = self.declare_parameter('opencv_threads', 2, ParameterDescriptor(read_only=True)).value
        if type(cv_threads) is not int or not 1 <= cv_threads <= 4:
            raise ValueError('opencv_threads must be 1..4')
        cv2.setNumThreads(cv_threads)  # 后台掩膜计算受限并行，控制状态机保持串行。
        self.get_logger().info(f'横线/预览 OpenCV线程数={cv2.getNumThreads()}；控制状态机串行执行')
        fixed = ParameterDescriptor(read_only=True)
        defaults = ControlConfig()
        values = {name: self.declare_parameter(name, value).value
                  for name, value in vars(defaults).items()}
        self.config = ControlConfig(**values)
        self.config.validate()
        self.navigation_mode = self.declare_parameter('navigation_mode', 'vision', fixed).value
        if self.navigation_mode not in ('vision', 'replay'):
            raise ValueError('navigation_mode must be vision or replay')
        self.standard_file = self.declare_parameter('standard_route_file', '~/.ros/line_follow/standards/route_01/standard_route.json', fixed).value
        self.standard_document = self.standard_follower = None
        self.standard_fault = None
        self.standard_correction = 0.
        self.standard_finish = None
        self.standard_visual_loss = None
        self.standard_navigator = None
        self.standard_visual_loss_timeout = float(self.declare_parameter('standard_visual_loss_timeout', 5., fixed).value)
        self.standard_visual_loss_distance = float(self.declare_parameter('standard_visual_loss_distance', .30, fixed).value)
        if not 0. <= self.standard_visual_loss_timeout <= 10. or not 0. <= self.standard_visual_loss_distance <= .50:
            raise ValueError('标准重放丢线续行限制需为0..10s、0..0.50m，0表示丢线即停')
        self.standard_finish_distance = float(self.declare_parameter('standard_visual_finish_distance', .15, fixed).value)
        self.standard_extra = float(self.declare_parameter('standard_finish_extra_distance', .05, fixed).value)
        if not .05 <= self.standard_finish_distance <= .3 or not 0. <= self.standard_extra <= .1:
            raise ValueError('标准前进末段视觉距离需0.05..0.30m，额外找线范围需0..0.10m')
        self.forward_navigation_config = ForwardNavigationConfig(
            finish_distance=self.standard_finish_distance, finish_extra_distance=self.standard_extra,
            loss_timeout=self.standard_visual_loss_timeout, loss_distance=self.standard_visual_loss_distance,
            assist_gain=float(self.declare_parameter('standard_visual_assist_gain', .5, fixed).value),
            curve_min_speed=float(self.declare_parameter('standard_curve_min_speed', .05, fixed).value),
            assist_max_yaw=float(self.declare_parameter('standard_visual_assist_max_yaw', .06, fixed).value))
        self.forward_navigation_config.validate()
        self.reverse_mode = self.declare_parameter('reverse_mode', 'recorded_route', fixed).value
        if self.reverse_mode not in ('recorded_route', 'camera'):
            raise ValueError('reverse_mode must be recorded_route or camera')
        self.departure_enabled = self.declare_parameter('dock_departure_enabled', True, fixed).value
        self.departure_speed = float(self.declare_parameter('dock_departure_speed', .02, fixed).value)
        if not .01 <= self.departure_speed <= .03:
            raise ValueError('离桩速度需为0.01..0.03m/s')
        self.departure = DockDeparture(
            float(self.declare_parameter('dock_departure_max_distance', .30, fixed).value),
            float(self.declare_parameter('dock_departure_timeout', 25., fixed).value))
        self.departure_charging = self.departure_base_safe = False
        self.departure_logged_state = None
        self.charging_mode = self.declare_parameter('charging_mode', False, fixed).value
        self.charging_layout = self.declare_parameter('charging_status_layout', 'byte_flags', fixed).value
        if self.charging_layout not in ('bitfield32', 'byte_flags'):
            raise ValueError('充电状态布局必须是bitfield32或byte_flags')
        self.dock_limit = float(self.declare_parameter('charging_search_distance', .10, fixed).value)
        self.dock = None
        self.charge_value, self.charge_at, self.charge_stamp = None, None, None
        self.charge_frames = 0
        self.charge_safe = False
        self.charge_candidate_at = None
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
        if Path(self.standard_file).expanduser().resolve() == Path(self.route_file).expanduser().resolve():
            raise ValueError('标准路线与当次路线必须分别保存，防止重放时覆盖标准数据')
        self.frozen_route_file = None
        self.route_checkpoint = None
        self.checkpoint_invalid_reason = None
        self.station_log_file = self.declare_parameter(
            'station_log_file', '~/.ros/line_follow/last_station_log.json', fixed).value
        self.station_run_events = []
        self.route_config = RouteConfig(**{name: self.declare_parameter('route_'+name, value, fixed).value
                                          for name, value in vars(RouteConfig()).items()})
        self.route_config.validate()
        self.forward_route_config = replace(self.route_config, max_curvature=float(
            self.declare_parameter('forward_route_max_curvature', 5., fixed).value))
        self.forward_route_config.validate()
        self.reverse_route_lookahead = float(self.declare_parameter(
            'reverse_route_lookahead', .26, fixed).value)
        self.reverse_route_max_curvature = float(self.declare_parameter(
            'reverse_route_max_curvature', 5., fixed).value)
        replace(self.route_config, lookahead=self.reverse_route_lookahead,
                max_curvature=self.reverse_route_max_curvature).validate()
        self.dock = ChargingDock(self.route_config, self.dock_limit)
        if self.charging_mode and (self.reverse_mode != 'recorded_route' or not self.route_visual_finish
                                   or not self.route_camera_stop or not self.route_config.visual_strafe):
            raise ValueError('充电模式需要记录路线、末段视觉及横移回中功能')
        mapping_file = self.declare_parameter('reverse_ground_mapping_file', '', fixed).value
        self.ground_mapping = GroundMapping(mapping_file) if mapping_file else None
        self.start_aligner = StartLineAligner(
            self.config, self.ground_mapping,
            max_distance_m=self.config.start_alignment_max_distance,
            decision_frames=self.config.start_alignment_observation_frames)
        self.start_alignment_active = False
        self.start_alignment_departing = False
        self.start_alignment_state = 'inactive'
        self.pending_recorder = None
        self.start_alignment_context = None
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
        self.cmd_pub = self.create_publisher(Twist, topics['cmd_vel_topic'], 1)
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
        self.create_subscription(String, topics['line_status_topic'], self.on_status, 1)
        self.create_subscription(PolygonStamped, topics['centerline_topic'],
                                 self.on_path, QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability))
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
        self.last_marker_latency_log = float('-inf')
        self.marker_worker = LatestWorker(self.detect_marker_frame, 'line-marker')
        self.preview_worker = LatestWorker(self.draw_preview, 'line-preview')
        self.route_writer = LatestWorker(write_route_document, 'line-route-save')
        self.station_writer = LatestWorker(write_route_document, 'line-station-save')
        self.create_timer(self.control_period, self.tick)
        if self.uses_recorded_route():
            self.load_return_route()
        elif self.enabled and self.config.drive_direction == 'forward':
            if self.record_route:
                initial = RouteRecorder(self.route_config, 'odom', 'base_link')
                if self.start_alignment_required(self.config):
                    self.recorder = initial
                    self.pending_recorder = initial
                    self.begin_start_alignment(initial)
                    self.invalidate_route(initial)
                else:
                    self.recorder = initial
                    self.flush_route(wait=True)
            elif self.navigation_mode == 'replay' and self.start_alignment_required(self.config):
                self.begin_start_alignment()
        self.get_logger().info(
            f'Line following ready: enabled={self.enabled}, '
            f'direction={self.config.drive_direction}, reverse_steering_sign={self.config.reverse_steering_sign:+.0f}, '
            f'finish_lateral_sign={self.route_config.finish_lateral_sign:+.0f}, '
            f'visual_strafe={self.route_config.visual_strafe}, '
            f'geometry={"ground_meters" if self.ground_mapping else "image_uncalibrated"}, '
            f'forward_speed={self.config.forward_speed:.3f} m/s; '
            f'reverse_max_speed={self.config.reverse_max_speed:.3f} m/s; '
            f'{topics["line_status_topic"]} -> {topics["cmd_vel_topic"]}')

    def uses_recorded_route(self):
        return self.reverse_mode == 'recorded_route' and self.config.drive_direction == 'reverse'

    def start_alignment_required(self, config):
        """Only align when establishing a recorded or standard-route origin."""
        return (config.start_alignment_enabled and config.stop_marker_enabled
                and config.drive_direction == 'forward'
                and (self.record_route or self.navigation_mode == 'replay'))

    def begin_start_alignment(self, recorder=None, config=None):
        config = self.config if config is None else config
        self.start_alignment_departing = False
        self.pending_recorder = recorder
        pose = self.latest_odom['pose'] if self.latest_odom is not None else None
        self.start_aligner.reset(time.monotonic(), pose)
        self.start_alignment_active = True
        self.start_alignment_state = 'start_alignment_waiting_for_marker'
        self.last_start_alignment_step = 0
        self.last_start_alignment_log = float('-inf')
        self.marker_gate.reset(allow_start_marker=False)
        self.start_alignment_context = dict(
            enabled=True, target_ratio=config.stop_marker_target_ratio,
            strategy='stop_observe_adaptive_odometry_step', settle_sec=config.start_alignment_settle_sec,
            recheck_settle_sec=config.start_alignment_recheck_settle_sec,
            decision_frames=config.start_alignment_observation_frames,
            coarse_step_distance_limit_m=config.start_alignment_coarse_step_distance,
            step_distance_limit_m=config.start_alignment_step_distance,
            coordinate_source=('ground_mapping' if self.ground_mapping else 'image_ratio'),
            ground_mapping_fit_error_m=(self.ground_mapping.fit_error_m if self.ground_mapping else None),
            correction_max_distance_m=config.start_alignment_max_distance,
            correction_max_reverse_distance_m=config.start_alignment_max_reverse_distance)

    def invalidate_route(self, recorder):
        """Replace a stale previous route with an explicitly invalid empty recording."""
        ticket = self.route_writer.submit((self.route_file, recorder.document()))
        self.route_writer.wait(ticket)
        result = self.route_writer.take()
        if result is not None and result[2] is not None:
            raise OSError(f'route_save_failed: {result[2]}')

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
            valid = (self.route_config.visual_entry_ok(feedback)
                     and abs(self.return_geometry.get('bend_angle_rad', math.inf)) <= self.route_config.finish_entry_bend
                     and line_command(self.return_observation, self.config)[2] == 'tracking')
            self.finish_straight_frames = min(3, self.finish_straight_frames+1) if valid else 0
            if self.route_config.visual_strafe:
                valid = (self.route_config.visual_entry_ok(feedback))
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
        self.charge_candidate_at = None
        self.dock = ChargingDock(self.route_config, self.dock_limit)
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
            if (self.recorder is not None and not self.start_alignment_active
                    and self.config.drive_direction == 'forward'
                    and (self.enabled or (self.record_stop_at is not None and now < self.record_stop_at))):
                if not self.recorder.points:
                    self.recorder.frame, self.recorder.child = message.header.frame_id, message.child_frame_id
                if self.odom_fault is None and self.recorder.fault is None:
                    was_finished = self.recorder.finished
                    if (abs(values[-3]) >= .01 or abs(values[-2]) >= .01 or abs(values[-1]) >= .05
                            or self.enabled and not self.marker_gate.arrived):
                        self.recorder.finished = False
                    self.recorder.add(pose, stamp=stamp/1e9, velocity=values[-3:])
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
            if self.charging_mode:
                self.return_fault = fault
            return 0., 0., fault
        if self.charging_mode:
            return self.charging_finish_command(pose)
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
                          reverse_max_speed=min(self.config.reverse_max_speed, self.route_config.finish_speed),
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

    def charging_signal(self, now):
        if self.charge_at is None or now-self.charge_at > 1. or self.charge_stamp is None:
            return None
        age = (self.get_clock().now().nanoseconds-self.charge_stamp)/1e9
        return self.charge_value if -.1 <= age <= 1. else None

    def charging_finish_command(self, pose):
        terminal = self.marker_gate.arrived and self.marker_gate.latched_kind == 'final'
        if self.marker_gate.arrived and not terminal:
            return 0., 0., 'stop_marker_intermediate'
        stopped = (abs(self.latest_odom['v']) < .005 and abs(self.latest_odom['vy']) < .005
                   and abs(self.latest_odom['w']) < .015)
        feedback = self.return_visual_feedback()
        v, self.reverse_vy, w, reason = self.dock.command(pose, feedback, self.return_camera_stamp,
            time.monotonic(), self.config, stopped, terminal, self.marker_gate.error)
        if reason.startswith('charging_dock_'):
            self.return_fault = reason
        self.return_finish.diagnostics['charging_dock'] = self.dock.snapshot()
        return v, w, reason

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
            return -min(self.config.forward_speed, self.config.reverse_max_speed, route.finish_speed), 0., 'reverse_visual_finish_tracking'
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
        dock_active = self.charging_mode and self.config.drive_direction == 'reverse'
        charge = self.charging_signal(now) if dock_active else None
        if dock_active and (self.return_fault or self.dock.fault):
            reason = self.return_fault or self.dock.fault
        elif dock_active and self.enabled and charge is True:
            # 第一次充电位立即发零速，连续新CAN帧确认后才报告完成；不因图像超时漏停。
            pose = self.latest_odom.get('pose') if self.latest_odom is not None else None
            marker_near = (pose is not None and self.dock.marker_anchor is not None
                           and math.dist(pose[:2], self.dock.marker_anchor) <= self.dock_limit
                           and self.dock.travel-self.dock.marker_travel+
                               math.dist(pose[:2], self.dock.previous) <= self.dock_limit)
            route_near = (pose is not None and self.return_tracker is not None
                          and math.dist(pose[:2], self.return_tracker.points[-1]) <= self.dock_limit
                          and self.return_tracker.diagnostics.get('remaining_m', math.inf) <= self.dock_limit)
            near = (marker_near or route_near) and self.odom_reason(now) is None
            near = near and self.connected and self.comm_time is not None and now-self.comm_time <= self.input_timeout
            if self.dock.done:
                reason = 'reverse_route_complete'
            elif not self.charge_safe:
                self.return_fault = reason = 'charging_dock_unsafe_charge_state'
            elif not near:
                self.return_fault = reason = 'charging_dock_unexpected_charge_location'
            elif self.charge_frames >= 3:
                if self.dock.fault or self.return_fault or (self.return_finish and self.return_finish.fault):
                    reason = self.dock.fault or self.return_fault or self.return_finish.fault
                else:
                    self.dock.done, self.dock.state = True, 'charged'
                    reason = 'reverse_route_complete'
            else:
                reason = 'charging_dock_confirming'
        elif dock_active and self.dock.done:
            reason = 'reverse_route_complete'  # 已完成保持停车，充电位抖动不重新发车。
        elif dock_active and self.charge_candidate_at is not None:
            reason = 'charging_dock_confirming'
            if now-self.charge_candidate_at > 2.:
                self.return_fault = reason = 'charging_dock_charge_unstable'
        elif dock_active and charge is None:
            reason = 'charging_dock_status_unavailable'
            if self.enabled and self.charge_at is not None:
                self.return_fault = reason
        if self.return_finish is not None and self.return_finish.arrived and not dock_active:
            reason = 'reverse_route_complete'
        v = w = 0.
        if reason is None:
            if self.return_tracker is None and self.enabled:
                pose = self.latest_odom['pose']
                if abs(self.latest_odom['v']) > .01 or abs(self.latest_odom['vy']) > .01 or abs(self.latest_odom['w']) > .05:
                    reason = 'reverse_start_not_stationary'
                else:
                    self.return_tracker = ReverseRouteFollower(self.route_document, pose,
                        replace(self.route_config, lookahead=self.reverse_route_lookahead,
                                max_curvature=self.reverse_route_max_curvature))
                    self.get_logger().info(
                        f'记录路线倒车：{self.route_document["length_m"]:.3f}m，终点对齐当前里程计；'
                        f'跟踪曲率上限={self.reverse_route_max_curvature:.2f}/m；'
                        '须保持前进停车后的实际位置和朝向，不使用相机倒车反号参数')
            if self.return_tracker is not None:
                v, w, reason = self.return_tracker.command(self.latest_odom['pose'],
                    min(self.config.forward_speed, self.config.reverse_max_speed, self.route_config.max_speed))
                if (self.return_finish is None and self.route_visual_finish and self.return_camera_stop_enabled()
                        and reason in ('reverse_route_tracking', 'reverse_route_complete')
                        and self.return_tracker.diagnostics['remaining_m'] <= self.route_config.finish_straight_distance
                        and self.finish_straight_frames >= 3
                        and self.return_tracker.remaining_is_gentle()
                        and abs(self.previous_w) <= self.route_config.finish_entry_yaw_rate
                        and abs(self.latest_odom['w']) <= self.route_config.finish_entry_yaw_rate):
                    try:
                        self.finish_entry_reason = 'confirmed_visual_path'
                        self.return_finish = ReverseFinishGuard(self.return_tracker, self.latest_odom['pose'],
                            replace(self.route_config, max_tracking_error=.10,
                                    finish_extra_distance=self.dock_limit) if dock_active else self.route_config)
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
        if dock_active and self.return_fault:
            self.dock.state = self.dock.fault = self.return_fault
        self.input_reason = reason
        state = reason if self.enabled else 'disabled'
        moving = self.enabled and reason in ('reverse_route_tracking', 'reverse_visual_finish_tracking', 'reverse_visual_finish_aligning')
        if self.enabled and self.return_camera_stop_enabled() and not (dock_active and self.return_finish is not None):
            if self.marker_gate.arrived and reason != 'reverse_route_complete' and self.return_finish is None:
                state, moving = 'stop_marker_intermediate', False
            elif moving and reason != 'reverse_visual_finish_aligning':
                limited, marker_state = self.marker_gate.limit(v, self.config, now=now, dt=dt)
                w *= abs(limited/v) if v else 0.
                v = limited
                if marker_state != 'searching':
                    state = 'stop_marker_'+marker_state.removeprefix('stopped_')
                    moving = marker_state in ('approaching', 'missing_approaching')
        if not moving:
            v = w = 0.
        else:
            # 途中允许跟随记录中的急弯，但不能超过底盘角速度上限。
            # 同比例降低线/角速度，保持几何曲率；末段仍沿用原有小曲率限制。
            if self.return_finish is None and abs(w) > self.config.max_angular_speed:
                scale = self.config.max_angular_speed/abs(w)
                v, w = v*scale, w*scale
            target_v = v
            normal_cruise = self.return_finish is None and state == 'reverse_route_tracking'
            if normal_cruise:
                hard_limit = min(self.config.forward_speed, self.config.reverse_max_speed, self.route_config.max_speed)
                curvature = w/v if v else 0.
                if abs(curvature) > 1e-9:
                    hard_limit = min(hard_limit, self.config.max_angular_speed/abs(curvature),
                        math.sqrt(self.route_config.max_lateral_accel/abs(curvature)))
                if self.return_tracker.diagnostics['remaining_m'] <= self.route_config.finish_distance:
                    hard_limit = min(hard_limit, abs(target_v))
                v, w = cruise_velocity(self.previous_v, self.previous_w, v, w,
                    self.config, dt, hard_speed_limit=hard_limit)
                if abs(w) > self.config.max_angular_speed:
                    scale = self.config.max_angular_speed/abs(w)
                    v, w = v*scale, w*scale
            elif self.route_config.visual_strafe and self.return_finish is not None:
                v = slew(self.previous_v, v, self.config.linear_acceleration, dt) if v else 0.
                w = slew(self.previous_w, w, self.config.angular_acceleration, dt) if w else 0.
            else:
                v, w = coordinated_velocity(self.previous_v, self.previous_w, v, w, self.config, dt)
            if not normal_cruise and abs(v) > abs(target_v):
                w *= abs(target_v/v)
                v = target_v
            if not (self.route_config.visual_strafe and self.return_finish is not None):
                curvature_cap = (self.return_tracker.config.max_curvature
                                 if self.return_finish is None else self.route_config.max_curvature)
                cap = abs(v)*curvature_cap
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
            if state in ('reverse_route_deviation', 'reverse_target_not_behind'):
                self.get_logger().error('倒车路径跟踪保护：'+json.dumps(
                    self.return_tracker.diagnostics, ensure_ascii=False))
            if dock_active and state.startswith('charging_dock_') and state != 'charging_dock_confirming':
                self.get_logger().error('倒车充电已停止：'+state+'；诊断='+json.dumps(self.dock.snapshot(), ensure_ascii=False))
            if dock_active and state == 'reverse_route_complete' and self.dock.done:
                self.get_logger().info('连续新反馈确认已充电，保持停车，不因充电位抖动重新发车')
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
                'standard_route': dict(mode=self.navigation_mode, file=self.standard_file,
                    diagnostics=self.standard_follower.diagnostics if self.standard_follower else None,
                    visual_correction_radps=self.standard_correction, visual_loss=self.standard_visual_loss,
                    finish=self.standard_finish, fault=self.standard_fault,
                    controller=self.standard_navigator.diagnostics if self.standard_navigator else None),
                'station_counts': dict(self.marker_gate.ledger.counts),
                'stop_marker': self.marker_gate.snapshot(self.config),
                'stop_detection': self.stop_detection,
                'visual_finish': self.return_finish.diagnostics if self.return_finish is not None else None,
                'charging_mode': dock_active, 'charging_dock': self.dock.snapshot(),
                'charging_signal': charge, 'charging_status_layout': self.charging_layout,
                'visual_entry_reason': self.finish_entry_reason, 'visual_straight_frames': self.finish_straight_frames,
                'visual_axis': self.return_visual_feedback(),
                'completion_source': ('charging_signal' if dock_active and self.dock.done else
                                      'camera_stop_marker' if self.return_finish is not None and self.return_finish.arrived else None),
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
        message, points, config, reference, shape, submitted_at, upstream_age_ms, tracker_ms = request
        started = time.monotonic()
        mask = CvBridge().imgmsg_to_cv2(message, desired_encoding='mono8')
        if shape is not None and mask.shape != shape:
            raise ValueError('stop mask resolution mismatch')
        diagnostics = {}
        candidates = detect_stop_markers(mask, points, config, reference, diagnostics=diagnostics)
        diagnostics['upstream_age_ms'] = upstream_age_ms
        diagnostics['tracker_processing_ms'] = tracker_ms
        diagnostics['worker_wait_ms'] = round((started-submitted_at)*1000, 1)
        diagnostics['detector_processing_ms'] = round((time.monotonic()-started)*1000, 1)
        diagnostics['worker_finished_at'] = time.monotonic()
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
        stamp_ns = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        upstream_age_ms = round((self.get_clock().now().nanoseconds-stamp_ns)/1e6, 1)
        # 不占用后台槽处理已过期的源图；过期结果仍由原来的新鲜度检查拦截。
        if not -100. <= upstream_age_ms <= self.input_timeout*1000:
            return
        self.marker_worker.submit((message, list(self.path_points), self.config,
                                   copy.deepcopy(self.marker_gate.reference_marker), shape,
                                   time.monotonic(), upstream_age_ms,
                                   self.status.get('processing_ms') if isinstance(self.status, dict) else None))

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
        finished_at = diagnostics.pop('worker_finished_at')
        diagnostics['result_collect_ms'] = round((time.monotonic()-finished_at)*1000, 1)
        stamp = header.stamp.sec*10**9+header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        previous = (self.marker_header.stamp.sec*10**9+self.marker_header.stamp.nanosec
                    if self.marker_header is not None else -1)
        # 参数/使能切换后的旧结果不能重新武装停车状态；过期结果不刷新新鲜度。
        if config is not self.config or stamp <= previous or not -.1 <= age <= self.input_timeout:
            now = time.monotonic()
            if age > self.input_timeout and now-self.last_marker_latency_log > 5.:
                self.last_marker_latency_log = now
                self.get_logger().warning(
                    f'横线结果已过期：源图到结果={age*1000:.0f}ms，'
                    f'上游={diagnostics["upstream_age_ms"]:.0f}ms，'
                    f'后台等待={diagnostics["worker_wait_ms"]:.0f}ms，'
                    f'横线检测计算={diagnostics.get("detector_processing_ms", 0):.0f}ms，'
                    f'结果回收={diagnostics["result_collect_ms"]:.0f}ms；'
                    '未使用过期结果，需检查上游处理/调度延迟')
            return
        diagnostics['result_age_ms'] = round(age*1000, 1)
        self.marker_candidates, self.stop_detection = candidates, diagnostics
        if self.return_camera_stop_enabled():
            self.marker_candidates = [dict(marker, kind='final' if self.finish_marker_is_final() else 'intermediate')
                                      for marker in candidates]
        if self.enabled and not self.start_alignment_active:
            self.marker_gate.observe(self.marker_candidates, height, self.config,
                                     start_alignment=False)
        if self.start_alignment_active:
            width = (self.status.get('image_width') if isinstance(self.status, dict) else None)
            if isinstance(width, int):
                stamp_ns = header.stamp.sec*10**9+header.stamp.nanosec
                received_at = time.monotonic()
                self.start_aligner.observe(self.marker_candidates, width, height,
                                           stamp_ns, received_at, source_time=received_at-age)
        self.marker_header = header
        self.marker_time = time.monotonic()

    def alignment_marker_pending(self, now, input_reason):
        """Only a marker-result wait with a fresh synchronized mainline qualifies."""
        if (input_reason not in ('stop_marker_mask_timeout', 'stop_marker_mask_stale')
                or not self.frame_sync or not self.path_valid or self.path_header is None
                or any(t is None or now-t > self.input_timeout
                       for t in (self.status_time, self.path_time))):
            return False
        stamp = self.path_header.stamp
        path_age = (self.get_clock().now().nanoseconds-stamp.sec*10**9-stamp.nanosec)/1e9
        return -.1 <= path_age <= self.input_timeout

    def tick_start_alignment(self, now, dt, input_reason):
        """Run stopped visual observations separated by bounded odometry steps.

        The marker gate is still fed for diagnostics, but its normal stop output
        is bypassed here.  The route origin is committed only after fresh visual
        evidence and a stationary odometry sample agree.
        """
        departure_command = self.tick_alignment_departure(now, dt, input_reason)
        if departure_command is not None:
            return departure_command
        pose = self.latest_odom['pose'] if self.latest_odom is not None else None
        odom_fresh = self.odom_reason(now) is None
        stopped = (self.latest_odom is not None
                   and abs(self.latest_odom['v']) < .005
                   and abs(self.latest_odom['vy']) < .005
                   and abs(self.latest_odom['w']) < .015)
        line_ready = (input_reason is None and self.status_time is not None
                      and self.path_time is not None and self.marker_time is not None)
        # Do not move the capture cutoff forward merely because the previous
        # asynchronous marker result expired. A fresh mainline and a stopped
        # chassis can still contribute new, individually fresh marker samples.
        marker_pending = self.alignment_marker_pending(now, input_reason)
        self.start_aligner.input_block_reason = input_reason
        previous_state = self.start_alignment_state
        state = self.start_aligner.command(now, pose, odom_fresh=odom_fresh,
                                           line_ready=line_ready, stopped=stopped,
                                           marker_pending=marker_pending)
        self.start_alignment_state = state
        if (now-self.last_start_alignment_log >= 2.
                or state.startswith('start_alignment_failed') and state != previous_state):
            self.last_start_alignment_log = now
            snapshot = self.start_aligner.snapshot()
            velocity = ({key: self.latest_odom[key] for key in ('v', 'vy', 'w')}
                        if self.latest_odom is not None else None)
            self.get_logger().info('起步对齐诊断：'+json.dumps(dict(
                state=state, axis=snapshot['axis'], phase=snapshot['phase'],
                input_block=input_reason, odometry_reason=self.odom_reason(now),
                marker_pending=snapshot['marker_pending'],
                stopped=stopped, actual_velocity=velocity,
                last_command=dict(v=self.previous_v, vy=self.previous_vy, w=self.previous_w),
                step_target_m=self.start_aligner.step_distance or self.start_aligner.step_lateral_m,
                step_actual_m=snapshot['step_actual_m'], step_remaining_m=snapshot['step_remaining_m'],
                step_actual_rad=snapshot['step_actual_rad'],
                step_limit_m=snapshot['step_limit_m'], step_timeout_sec=snapshot['step_timeout_sec'],
                distance_estimate_source=snapshot['distance_estimate_source'],
                estimated_remaining_m=snapshot['estimated_remaining_m'],
                step_age_sec=(now-self.start_aligner.step_started_at
                              if self.start_aligner.step_started_at is not None else None),
                lateral_min_speed=self.config.start_alignment_min_lateral_speed,
                marker_candidates=len(self.marker_candidates),
                marker_rejected=self.stop_detection.get('rejected', {}),
                detector_processing_ms=self.stop_detection.get('detector_processing_ms'),
                upstream_age_ms=self.stop_detection.get('upstream_age_ms'),
                tracker_processing_ms=self.stop_detection.get('tracker_processing_ms'),
                worker_wait_ms=self.stop_detection.get('worker_wait_ms'),
                result_collect_ms=self.stop_detection.get('result_collect_ms'),
                marker_result_age_ms=self.stop_detection.get('result_age_ms'),
                samples=snapshot['samples'], required_samples=snapshot['decision_frames'],
                observation_stability=snapshot['observation_stability'],
                confirmations=snapshot['stable_frames'], required_confirmations=snapshot['confirmation_frames'],
                longitudinal_error=(self.start_aligner.position_error_m if self.start_aligner.metric
                                    else self.start_aligner.position_error),
                longitudinal_unit='m' if self.start_aligner.metric else 'image_ratio',
                lateral_error=self.start_aligner.lateral_error,
                lateral_unit=snapshot['lateral_error_unit'],
                heading_error_rad=self.start_aligner.heading_error,
                observation_age_sec=snapshot['observation_age_sec']), ensure_ascii=False))
        if self.start_aligner.step_count > self.last_start_alignment_step:
            self.last_start_alignment_step = self.start_aligner.step_count
            estimate = self.start_aligner.estimated_remaining_m
            estimate_text = ('未建立可靠距离比例，先小步试探' if estimate is None else
                             f'估计距目标={estimate*1000:.1f}mm')
            if self.start_aligner.step_axis == 'heading':
                estimate_text = '先修正朝向，再估计平移距离'
            self.get_logger().info(
                f'起步对齐第{self.start_aligner.step_count}步：'
                f'轴={self.start_aligner.step_axis}，'
                f'{estimate_text}，估计来源={self.start_aligner.distance_estimate_source}，'
                f'比例检查={self.start_aligner.response_reason}，'
                f'前后目标={self.start_aligner.step_distance*1000:.1f}mm，'
                f'横移目标={self.start_aligner.step_lateral_m*1000:.1f}mm，'
                f'目标转角={math.degrees(self.start_aligner.step_angle):.1f}deg；'
                '里程计到位后停车，等待新画面稳定再复核')
        if state.startswith('start_alignment_failed'):
            self._latch_start_alignment_failure(state)
            self.stop()
            return 0., 0., 0., state
        if state == 'start_alignment_complete' and stopped:
            if self._complete_start_alignment(now):
                return 0., 0., 0., 'start_alignment_complete'
            self._latch_start_alignment_failure(
                'start_alignment_failed_route_init: '+(self.standard_fault or 'route initialization failed'))
            self.stop()
            return 0., 0., 0., 'start_alignment_failed_route_init'
        v, vy, w = self.start_aligner.velocity()
        if self.start_aligner.phase != 'moving':
            self.start_aligner.record_command(0.)
            return 0., 0., 0., state
        # Inactive axes go directly to zero. Never slew a previous axis into a
        # newly selected one; every phase change already includes a stopped dwell.
        v = slew(self.previous_v, v, self.config.linear_acceleration, dt) if v else 0.
        vy = slew(self.previous_vy, vy, self.config.linear_acceleration, dt) if vy else 0.
        w = slew(self.previous_w, w, self.config.angular_acceleration, dt) if w else 0.
        v = max(-self.config.start_alignment_speed,
                min(self.config.start_alignment_speed, v))
        w = max(-self.config.start_alignment_max_angular_speed,
                min(self.config.start_alignment_max_angular_speed, w))
        vy = max(-self.config.start_alignment_lateral_speed,
                 min(self.config.start_alignment_lateral_speed, vy))
        self.start_aligner.record_command(v)
        return v, vy, w, state

    def tick_alignment_departure(self, now, dt, input_reason):
        """Finish bounded straight undocking before starting the visual servo.

        The existing departure gate supplies charging confirmation, distance and
        time limits. Never use its charging exception for a lateral/yaw step.
        """
        if self.start_aligner.fault is not None:
            return None
        if self.departure.fault:
            reason = 'start_alignment_failed_'+self.departure.fault
            self._latch_start_alignment_failure(reason)
            self.stop()
            return 0., 0., 0., reason
        if self.departure.state == 'active' and not self.start_alignment_departing:
            self.start_alignment_departing = True
            self.get_logger().info(
                '起步仍在充电桩：先低速直线前进离桩，禁止横移和旋转；'
                '连续3帧确认脱离后停车，再进行视觉对齐和路线记录')
        if not self.start_alignment_departing:
            return None
        if self.departure.state == 'active':
            self.start_alignment_state = 'start_alignment_departing_dock'
            self.start_aligner.input_block_reason = input_reason
            if input_reason is not None or not self.departure_permitted():
                return 0., 0., 0., self.start_alignment_state
            target = min(self.departure_speed, self.config.forward_speed)
            v = max(0., min(target, slew(self.previous_v, target,
                                       self.config.linear_acceleration, dt)))
            return v, 0., 0., self.start_alignment_state
        # Discard every pre-undocking observation and start the normal stopped
        # dwell. Neither undocking nor its observation wait enters the route.
        self.start_alignment_departing = False
        pose = self.latest_odom['pose'] if self.latest_odom is not None else None
        self.start_aligner.reset(now, pose)
        self.start_alignment_context['dock_departure'] = self.departure.snapshot()
        self.start_alignment_state = 'start_alignment_settling_after_departure'
        self.get_logger().info('已连续3帧确认脱离充电桩：先停车，再等待新画面进行起步对齐')
        return 0., 0., 0., self.start_alignment_state

    def _latch_start_alignment_failure(self, reason):
        self.start_alignment_active = True  # Keep publishing zero until explicit disable/re-enable.
        self.start_alignment_state = reason
        self.start_aligner.fault = reason
        self.start_aligner.state = reason
        if self.recorder is not None:
            self.recorder.fault = reason
            self.flush_route()
        self.pending_recorder = None

    def _complete_start_alignment(self, now):
        """Latch the aligned odometry pose as the route origin."""
        if self.latest_odom is None or self.odom_reason(now) is not None:
            return False
        pose = self.latest_odom['pose']
        marker = self.start_aligner.marker
        if self.navigation_mode == 'replay':
            try:
                reason = self.standard_start_reason(self.config)
                if reason is not None:
                    self.standard_fault = reason
                    return False
                tracker = StandardRouteFollower(self.standard_document, pose, self.forward_route_config)
            except (OSError, ValueError, KeyError) as error:
                self.standard_fault = 'standard_route_start: '+str(error)
                return False
        else:
            tracker = None
        if not self.marker_gate.complete_start_alignment(self.config, marker=marker):
            return False
        if self.pending_recorder is not None:
            recorder = self.pending_recorder
            recorder.frame = self.latest_odom['frame']
            recorder.child = self.latest_odom['child']
            recorder.context['start_alignment'] = self.start_aligner.snapshot()
            recorder.context['start_alignment_context'] = self.start_alignment_context
            observation = self.status if isinstance(self.status, dict) else {}
            geometry, _ = tracking_geometry(self.path_points, observation.get('image_width'),
                                            observation.get('image_height'), self.config)
            if geometry is not None:
                recorder.context['start_visual'] = {name: geometry[name] for name in
                    ('error_normalized', 'near_heading_rad')}
            recorder.add(pose, stamp=self.latest_odom['stamp']/1e9,
                         velocity=(self.latest_odom['v'], self.latest_odom['vy'], self.latest_odom['w']))
            self.recorder = recorder
            self.pending_recorder = None
        if tracker is not None:
            self.standard_follower = tracker
            self.standard_fault = None
        self.start_alignment_active = False
        self.start_alignment_state = 'start_alignment_complete'
        self.start_alignment_context = None
        self.route_saved_finished = False
        return True

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
        align_start = self.start_alignment_required(candidate)
        if (not self.enabled and enabled and candidate.drive_direction == 'forward'
                and self.navigation_mode == 'replay'):
            try:
                document = self.read_standard()
                if self.odom_reason(time.monotonic()) is not None:
                    raise ValueError('重放需要有效里程计')
                if (document['frame_id'] != self.latest_odom['frame']
                        or document['child_frame_id'] != self.latest_odom['child']):
                    raise ValueError('标准路线与当前里程计坐标系名称不一致')
                if abs(self.latest_odom['v']) > .005 or abs(self.latest_odom['vy']) > .005 or abs(self.latest_odom['w']) > .015:
                    raise ValueError('重放前必须停稳')
                if not candidate.stop_marker_enabled:
                    raise ValueError('标准重放必须保留横线检测')
                if not align_start:
                    reason = self.standard_start_reason(candidate)
                    if reason is not None:
                        raise ValueError(reason)
                tracker = None if align_start else StandardRouteFollower(
                    document, self.latest_odom['pose'], self.forward_route_config)
            except (OSError, ValueError, KeyError) as error:
                return SetParametersResult(successful=False, reason='standard_route_start: '+str(error))
            self.standard_document, self.standard_follower, self.standard_fault = document, tracker, None
            self.standard_finish = None
            self.standard_visual_loss = None
            self.standard_navigator = None
            if align_start and not self.record_route:
                self.begin_start_alignment(config=candidate)
        if (not self.enabled and enabled and candidate.drive_direction == 'forward'
                and self.record_route):
            self.frozen_route_file = self.route_checkpoint = self.checkpoint_invalid_reason = None
            recorder = RouteRecorder(self.route_config, 'odom', 'base_link')
            recorder.context = dict(control=vars(candidate), route=vars(self.route_config),
                image_size=[self.status.get('image_width'), self.status.get('image_height')]
                    if isinstance(self.status, dict) else None)
            if isinstance(self.status, dict) and not align_start:
                initial_geometry, _ = tracking_geometry(self.path_points, self.status.get('image_width'),
                                                        self.status.get('image_height'), candidate)
                if initial_geometry is not None:
                    recorder.context['start_visual'] = {name: initial_geometry[name] for name in
                        ('error_normalized', 'near_heading_rad')}
            try:
                ticket = self.route_writer.submit((self.route_file, recorder.document()))
                self.route_writer.wait(ticket)
                result = self.route_writer.take()
                if result is not None and result[2] is not None:
                    raise OSError(str(result[2]))
            except (OSError, ValueError, TimeoutError) as error:
                return SetParametersResult(successful=False, reason=f'route_save_failed: {error}')
            if align_start:
                self.recorder = recorder
                self.pending_recorder = recorder
                self.begin_start_alignment(recorder, config=candidate)
            else:
                self.recorder = recorder
                self.pending_recorder = None
                self.start_alignment_active = False
            self.route_saved_finished = False
            self.record_stop_at = None
            self.get_logger().info(
                ('起步视觉对齐已启用；对齐确认后才开始记录路线：' if align_start else '开始记录前进路线：')
                + os.path.expanduser(self.route_file))
        if self.enabled and not enabled:
            self.record_stop_at = time.monotonic()+1.
            if self.start_alignment_active and self.pending_recorder is not None:
                self.pending_recorder.fault = 'start_alignment_cancelled'
                self.pending_recorder.finished = False
                self.recorder = self.pending_recorder
            self.flush_route(finish=True, wait=True)
            self.start_alignment_active = False
            self.pending_recorder = None
            self.start_aligner.reset()
            self.start_alignment_state = 'inactive'
        if (candidate.drive_direction == 'reverse' and
                (not self.enabled and enabled or candidate.drive_direction != self.config.drive_direction)):
            self.flush_route(wait=True)
            self.load_return_route()
        marker_was_enabled = self.config.stop_marker_enabled
        self.config = candidate
        self.start_aligner.config = candidate
        self.start_aligner.decision_frames = candidate.start_alignment_observation_frames
        self.start_aligner.max_distance_m = candidate.start_alignment_max_distance
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
            self.departure.reset()
            self.marker_gate.reset()
            self.station_run_events = []
            if self.start_alignment_active:
                self.marker_gate.reset(allow_start_marker=False)
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
        response.message = (('enabled' if self.enabled else 'disabled') if response.success
                            else results[0].reason or '循迹使能参数被拒绝，状态保持不变')
        if not response.success:
            self.get_logger().warning('循迹使能请求被拒绝：'+response.message)
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

    def read_standard(self):
        document = load_route(self.standard_file)
        info = document.get('standard_route', {})
        if info.get('schema') != 'robot_standard_route_v1' or not info.get('quality', {}).get('accepted'):
            raise ValueError('文件不是已通过质量检查的标准路线，请先运行teach模式')
        self.standard_document = document
        return document

    def standard_start_reason(self, config):
        status = self.status if isinstance(self.status, dict) else {}
        geometry, reason = tracking_geometry(self.path_points,
            status.get('image_width'), status.get('image_height'), config)
        if reason is not None:
            return reason
        recorded = (self.standard_document or {}).get('recording_config', {})
        reference = recorded.get('start_visual')
        if reference is not None and (abs(geometry['error_normalized']-reference['error_normalized']) > .08
                or abs(wrap(geometry['near_heading_rad']-reference['near_heading_rad'])) > .07):
            return 'standard_start_differs_from_teach: 当前起步对线与录制时差异过大，请重新摆正'
        for key in ('image_forward_direction', 'steering_sign', 'vehicle_reference_x_ratio', 'vehicle_reference_y_ratio'):
            if key in recorded.get('control', {}) and getattr(config, key) != recorded['control'][key]:
                return 'standard_camera_config_changed: 相机方向/安装参考已变化，请重新录制'
        # A separate near-line fit cross-checks the bar aligner's filtered fit.
        # Allow a small fit discrepancy, but reject the logged crooked departure.
        if abs(geometry['error_normalized']) > .04 or abs(geometry['near_heading_rad']) > .07:
            return 'standard_start_not_aligned: 近端主线复核未通过，请摆正或检查视觉拟合；横向需小于半图宽4%、图像切线约4度'
        return None

    def on_ready(self, request, response):
        """Check real inputs without enabling the follower or issuing motion."""
        del request
        if self.uses_recorded_route():
            if self.route_document is None:
                self.load_return_route()
            now = time.monotonic()
            reason = self.blocked_reason(now, check_enabled=False)
            if reason is None and self.charging_mode and self.charging_signal(now) is None:
                reason = 'charging_dock_status_unavailable'
            response.success = reason is None
            response.message = ('recorded route and odometry ready' if reason is None else
                                f'{reason}; route={self.route_file}; detail={self.route_load_error}')
            return response
        reason = self.blocked_reason(time.monotonic(), check_enabled=False)
        if reason is None and self.navigation_mode == 'replay' and not self.start_alignment_required(self.config):
            try:
                self.read_standard()
                reason = self.standard_start_reason(self.config)
            except (OSError, ValueError, KeyError) as error:
                reason = 'standard_route_unavailable: '+str(error)
        elif reason is None and self.navigation_mode == 'replay':
            try:
                document = self.read_standard()
                if self.odom_reason(time.monotonic()) is not None:
                    reason = 'route_odom_unavailable'
                elif (document['frame_id'] != self.latest_odom['frame']
                      or document['child_frame_id'] != self.latest_odom['child']):
                    reason = 'standard_route_frame_mismatch'
            except (OSError, ValueError, KeyError) as error:
                reason = 'standard_route_unavailable: '+str(error)
        if reason is None and not self.start_alignment_required(self.config):
            observation = dict(self.status) if isinstance(self.status, dict) else {}
            geometry, reason = tracking_geometry(
                self.path_points, observation.get('image_width'),
                observation.get('image_height'), self.config)
            if reason is None:
                observation = self.local_observation(observation, geometry)
                _, _, reason = line_command(observation, self.config)
                if reason == 'tracking' and self.ready_frames < self.reacquire_frames:
                    reason = 'reacquiring'
        if self.start_alignment_required(self.config):
            response.success = reason is None
            response.message = ('chassis, odometry and departure-bar alignment inputs ready'
                                if response.success else str(reason))
        else:
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
            # 处理超预算立即停车；重放中段的单纯丢线交给有界里程计续行。
            key = status['source_stamp_sec']*10**9+status['source_stamp_nanosec']
            if key >= self.committed_stamp and (
                    status.get('processing_over_budget') is True or
                    status.get('lost') is True and not self.standard_middle_active()):
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
            if not self.standard_middle_active():
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
            if not self.standard_middle_active():
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
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        now = time.monotonic()
        current = (bool(message.system_status_flags & (1 << 3)) if self.charging_layout == 'bitfield32'
                   else bool(message.manual_charging or message.dock_charging))
        self.charge_safe = (message.chassis_fault == 0 and
            (not (message.system_status_flags & 3) if self.charging_layout == 'bitfield32' else
             message.chassis_state not in (ChassisStatus.CHASSIS_STATE_ESTOP,
                 ChassisStatus.CHASSIS_STATE_FAULT, ChassisStatus.CHASSIS_STATE_TOW)
             and not message.excessive_tilt and message.obstacle_status_flags == 0))
        if stamp != self.charge_stamp:
            continuous = (self.charge_at is not None and now-self.charge_at < 1.
                          and self.charge_stamp is not None and 0 < stamp-self.charge_stamp <= 10**9)
            self.charge_frames = self.charge_frames+1 if current and continuous and self.charge_value else int(current)
            self.charge_value, self.charge_at, self.charge_stamp = current, now, stamp
        if self.charging_mode and self.config.drive_direction == 'reverse' and current:
            if self.enabled and self.charge_candidate_at is None:
                self.charge_candidate_at = now
            self.stop()
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
        self.departure_charging = bool(message.dock_charging) or bool(current)
        self.departure_base_safe = (
            message.chassis_state in (ChassisStatus.CHASSIS_STATE_COMM_CONTROL, ChassisStatus.CHASSIS_STATE_PARK)
            and message.chassis_fault == 0 and message.obstacle_status_flags == 0
            and not message.excessive_tilt and not message.manual_charging)
        self.departure.observe(self.departure_charging, stamp)
        self.chassis_time = time.monotonic()
        if not self.chassis_allowed and not self.departure_permitted() and not self.dry_run:
            self.recovery_deadline = None
            self.ready_frames = 0
            self.good_frames = 0
            self.stop()

    def departure_permitted(self):
        """只允许前进离桩；完成后再出现充电信号不重新豁免。"""
        return (self.departure_enabled and self.config.drive_direction == 'forward'
                and self.departure_base_safe and self.departure.state in ('idle', 'active')
                and (self.departure_charging or self.departure.state == 'active'))

    def blocked_reason(self, now, check_enabled=True):
        if check_enabled and not self.enabled:
            return 'disabled'
        if self.frame_fault and not self.uses_recorded_route():
            return self.frame_fault
        if not self.dry_run and (
                not self.connected or self.comm_time is None
                or now - self.comm_time > self.comm_timeout):
            return 'chassis_disconnected'
        if self.departure.fault:
            return self.departure.fault
        if not self.dry_run and (
                (not self.chassis_allowed and not self.departure_permitted()) or self.chassis_time is None
                or now - self.chassis_time > self.input_timeout):
            return 'chassis_not_ready'
        if self.record_route and not self.uses_recorded_route() and self.recorder is not None and self.recorder.fault:
            return 'route_record_failed'
        if self.uses_recorded_route() or self.record_route or self.departure_permitted() or self.navigation_mode == 'replay':
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
        if self.path_header is None or not self.path_valid and not self.standard_middle_active():
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

    def standard_middle_active(self):
        """Line geometry may be absent only after anchoring and before final approach."""
        return (self.enabled and self.navigation_mode == 'replay'
                and self.config.drive_direction == 'forward' and not self.start_alignment_active
                and self.standard_follower is not None and self.standard_fault is None
                and self.standard_finish is None
                and self.standard_follower.diagnostics.get('remaining_m', 0.) > self.standard_finish_distance)

    def standard_forward_command(self, now, geometry_reason):
        """ROS inputs/interlocks stay here; phase arbitration has one owner."""
        if self.standard_fault or self.standard_follower is None:
            return 0., 0., self.standard_fault or 'standard_route_not_initialized'
        if (self.standard_navigator is None
                or self.standard_navigator.follower is not self.standard_follower):
            self.standard_navigator = ForwardRouteNavigator(
                self.standard_follower, self.forward_navigation_config)
        navigator = self.standard_navigator
        observation = dict(self.status) if isinstance(self.status, dict) else {}
        if self.geometry is not None:
            observation = self.local_observation(observation, self.geometry)
        vision_v, vision_w, visual_reason = line_command(observation, self.config)
        if observation.get('processing_over_budget') is not False:
            visual_reason = 'processing_over_budget'
        elif geometry_reason is not None:
            visual_reason = geometry_reason
        geometry = self.geometry or {}
        visual = VisualCommand(visual_reason, vision_v, vision_w,
            geometry.get('error_normalized', 0.), geometry.get('near_heading_rad', 0.),
            self.good_frames >= self.reacquire_frames)
        old_phase, old_loss = navigator.phase, navigator.visual_loss
        v, w, reason = navigator.command(now, self.latest_odom['pose'], self.config.forward_speed, visual)
        self.standard_fault = navigator.fault
        self.standard_finish = navigator.finish
        self.standard_visual_loss = navigator.visual_loss
        self.standard_correction = navigator.correction
        if old_phase != navigator.phase:
            self.get_logger().info('标准重放阶段：'+old_phase+' -> '+navigator.phase)
        if old_loss is None and navigator.visual_loss is not None:
            self.get_logger().info(
                f'标准重放暂时丢线：里程计限速续行，最多{self.standard_visual_loss_timeout:g}秒/'
                f'{self.standard_visual_loss_distance*100:g}厘米，末段仍需视觉确认')
        elif old_loss is not None and navigator.visual_loss is None and navigator.phase == 'route':
            self.get_logger().info('标准重放主线已稳定恢复，恢复视觉辅助')
        self.control_curve_strength = min(1., abs(w/max(v, .005))/self.forward_route_config.max_curvature)
        return v, w, reason

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
        if (self.enabled and self.config.drive_direction == 'forward'
                and self.departure_enabled and self.odom_reason(now) is None):
            self.departure.advance(now, self.latest_odom['pose'], self.departure_charging)
        if self.departure.state != self.departure_logged_state:
            self.departure_logged_state = self.departure.state
            if self.departure.state != 'idle':
                self.get_logger().info('充电桩离站状态：'+json.dumps(self.departure.snapshot(), ensure_ascii=False))
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
        v = vy = w = 0.0
        alignment_running = self.start_alignment_active
        if alignment_running:
            v, vy, w, input_reason = self.tick_start_alignment(now, dt, input_reason)
        elif input_reason is None and self.navigation_mode == 'replay' and self.enabled:
            v, w, input_reason = self.standard_forward_command(now, geometry_reason)
            if self.geometry is not None:
                observation = self.local_observation(observation, self.geometry)
        elif input_reason is None:
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
            if reason == 'tracking' and self.good_frames < self.reacquire_frames and not self.standard_middle_active():
                v = w = 0.0
                reason = 'reacquiring'
        else:
            self.good_frames = 0
        if (self.enabled and self.config.stop_marker_enabled and self.marker_gate.arrived
                and not alignment_running):
            # 停车状态锁定；输入诊断仍通过 input_state 报告，恢复视觉不自动开走。
            v = vy = w = 0.0
            self.steering_guard.reset()
            reason = 'stop_marker_'+self.marker_gate.state(self.config).removeprefix('stopped_')
        elif reason == 'tracking':
            stamp = self.path_header.stamp
            sample = (stamp.sec, stamp.nanosec)
            if self.standard_middle_active():
                # The recorded metric path supplies curvature, independently of
                # camera frame rate and image steering deadbands.
                curvature = w/max(v, .005)
                cap = self.forward_route_config.max_curvature
                curvature = max(-cap, min(cap, curvature))
                self.steering_guard.raw_curvature = self.steering_guard.curvature = curvature
                self.steering_guard.state = 'standard_route'
            else:
                curvature = self.steering_guard.update(w, self.config, sample, dt,
                    error_normalized=observation['error_normalized'],
                    curve_strength=self.control_curve_strength)
            marker_state = 'searching'
            if self.config.stop_marker_enabled:
                v, marker_state = self.marker_gate.limit(v, self.config, now=now, dt=dt)
                if marker_state != 'searching':
                    reason = 'stop_marker_'+marker_state.removeprefix('stopped_')
            target_v = v
            yaw_limit = self.config.max_angular_speed+self.control_curve_strength*(
                self.config.curve_max_angular_speed-self.config.max_angular_speed)
            if self.config.drive_direction == 'reverse':
                yaw_limit = min(yaw_limit, self.config.max_angular_speed)
            if self.standard_middle_active() and abs(curvature*target_v) > yaw_limit:
                # RPP-style regulation: keep the requested radius when yaw is capped.
                target_v = min(target_v, yaw_limit/abs(curvature))
            target_w = max(-yaw_limit, min(yaw_limit, curvature*target_v))
            if self.navigation_mode == 'replay':
                hard_limit = min(self.config.forward_speed, self.forward_route_config.max_speed)
                if abs(curvature) > 1e-9:
                    hard_limit = min(hard_limit, yaw_limit/abs(curvature),
                        math.sqrt(self.forward_route_config.max_lateral_accel/abs(curvature)))
                if self.standard_visual_loss is not None:
                    hard_limit = min(hard_limit, self.forward_navigation_config.loss_speed)
                if marker_state != 'searching' or self.standard_finish is not None:
                    hard_limit = min(hard_limit, abs(target_v))
                v, w = cruise_velocity(self.previous_v, self.previous_w,
                    target_v, target_w, self.config, dt, self.control_curve_strength, hard_limit)
                if abs(w) > yaw_limit:
                    scale = yaw_limit/abs(w)
                    v, w = v*scale, w*scale
            else:
                v, w = coordinated_velocity(self.previous_v, self.previous_w,
                    target_v, target_w, self.config, dt, self.control_curve_strength)
                if marker_state != 'searching' and abs(v) > abs(target_v):
                    w *= abs(target_v/v)
                    v = target_v
        elif reason.startswith('start_alignment_'):
            # tick_start_alignment has already applied bounded velocity/turn rates.
            pass
        else:
            v = vy = w = 0.0
            self.steering_guard.reset()
            if self.recovery_deadline is None or now > self.recovery_deadline:
                self.control_curve_strength = 0.0
        if self.departure.state == 'active' and v > self.departure_speed:
            w *= self.departure_speed/v
            v = self.departure_speed
        self.previous_v, self.previous_vy, self.previous_w = v, vy, w
        if self.enabled:
            self.marker_gate.record_motion(v, dt, self.config)
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = v, vy, w
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
                'dock_departure': self.departure.snapshot(),
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
                'start_alignment': (self.start_aligner.snapshot()
                                    if self.start_alignment_active or
                                    self.start_alignment_state != 'inactive' else None),
                'standard_route': dict(mode=self.navigation_mode, file=self.standard_file,
                    diagnostics=self.standard_follower.diagnostics if self.standard_follower else None,
                    visual_correction_radps=self.standard_correction, visual_loss=self.standard_visual_loss,
                    finish=self.standard_finish, fault=self.standard_fault,
                    controller=self.standard_navigator.diagnostics if self.standard_navigator else None),
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
