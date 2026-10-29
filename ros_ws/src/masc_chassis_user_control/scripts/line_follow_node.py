#!/usr/bin/env python3
"""Drive the chassis from ruier_camer_diver observations; stop on lost/stale input."""

import json
import math
import os
import time
from dataclasses import replace

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Twist
from masc_chassis_msgs.msg import ChassisStatus, CommStatus
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String, Empty
from std_srvs.srv import SetBool, Trigger

from stop_marker import StopMarkerGate, detect_stop_markers
from line_follow_control import ControlConfig, line_command, motion_preview, slew, smooth_yaw, tracking_geometry


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
        self.enabled = self.declare_parameter('enabled', True).value
        self.dry_run = self.declare_parameter('dry_run', False, fixed).value
        self.show_window = self.declare_parameter('show_window', False, fixed).value
        self.bridge = CvBridge()
        self.window_name = 'MASC line following - body reference'
        self.geometry = None
        self.marker_gate = StopMarkerGate()
        self.marker_candidates = []
        self.marker_time = None
        self.marker_header = None
        self.marker_stamp = None
        self.control_curve_strength = 0.0
        self.exposure_status = None
        self.last_state = 0.0
        self.last_debug_time = 0.0
        if self.show_window and not (os.getenv('DISPLAY') or os.getenv('WAYLAND_DISPLAY')):
            self.get_logger().warning('No desktop display; use /line_follow/debug_image remotely')
            self.show_window = False
        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1100, 700)
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
        self.previous_v = self.previous_w = 0.0
        self.last_tick = time.monotonic()
        self.reason = None
        self.input_reason = 'waiting'
        self.last_blocked_reason = None
        self.cmd_pub = self.create_publisher(Twist, topics['cmd_vel_topic'], 10)
        self.state_pub = self.create_publisher(String, '/line_follow/status', 10)
        self.debug_pub = self.create_publisher(Image, '/line_follow/debug_image', qos_profile_sensor_data)
        debug_image_topic = self.declare_parameter(
            'debug_image_topic', '/camera/line/debug_image', fixed).value
        self.create_subscription(
            Image, debug_image_topic, self.on_debug_image,
            QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability))
        marker_mask_topic = self.declare_parameter('stop_marker_mask_topic', '/camera/line/mask', fixed).value
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
        self.create_service(SetBool, '~/enable', self.on_enable)
        self.create_service(Trigger, '~/ready', self.on_ready)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.create_timer(self.control_period, self.tick)
        self.get_logger().info(
            f'Line following ready: enabled={self.enabled}, '
            f'forward_speed={self.config.forward_speed:.3f} m/s; '
            f'{topics["line_status_topic"]} -> {topics["cmd_vel_topic"]}')

    def on_marker_mask(self, message):
        if not self.config.stop_marker_enabled:
            return
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        if stamp == self.marker_stamp:
            return
        self.marker_stamp = stamp
        try:
            mask = self.bridge.imgmsg_to_cv2(message, desired_encoding='mono8')
            if isinstance(self.status, dict) and (
                    mask.shape != (self.status.get('image_height'), self.status.get('image_width'))):
                self.marker_time = None
                return
            self.marker_candidates = detect_stop_markers(mask, self.path_points, self.config)
            self.marker_gate.observe(self.marker_candidates, mask.shape[0], self.config)
            self.marker_header = message.header
            self.marker_time = time.monotonic()
        except (CvBridgeError, ValueError, cv2.error):
            self.marker_time = None

    def on_resume_marker(self, _message):
        if (self.enabled and self.config.stop_marker_enabled and self.blocked_reason(time.monotonic(), check_enabled=False) is None
                and self.marker_gate.resume(self.config)):
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
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        marker_was_enabled = self.config.stop_marker_enabled
        self.config = candidate
        if candidate.stop_marker_enabled and not marker_was_enabled:
            self.marker_gate.reset()
            self.marker_time = self.marker_stamp = None
        if enabled and not self.enabled:
            self.marker_gate.reset()
        if enabled != self.enabled or updates.keys() & {
                "image_forward_direction", "vehicle_reference_x_ratio", "vehicle_reference_y_ratio"}:
            self.good_frames = 0
        self.enabled = enabled
        if not enabled or candidate.forward_speed == 0.0:
            self.stop()
        return SetParametersResult(successful=True)

    def on_enable(self, request, response):
        results = self.set_parameters([rclpy.parameter.Parameter('enabled', value=request.data)])
        response.success = results[0].successful
        response.message = 'enabled' if self.enabled else 'disabled'
        return response

    def on_ready(self, request, response):
        """Check real inputs without enabling the follower or issuing motion."""
        del request
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
            'path_heading_rad', 'curve_strength')})
        fraction = observation.get('valid_row_fraction')
        if (not isinstance(fraction, bool) and isinstance(fraction, (int, float))
                and math.isfinite(fraction) and 0 <= fraction <= 1):
            observation['valid_row_fraction'] = min(fraction, geometry['trusted_span_ratio'])
        observation['curve_geometry_valid'] = True
        observation['target_estimated'] = False
        observation['gap_connection_confirmed'] = bool(geometry['bridged_gaps'])
        if observation['gap_connection_confirmed']:
            # 上游 Canny 的疑似遮挡不能对已经验证的同一缺口重复降速。
            observation['occlusion_suspected'] = False
        return observation

    def on_status(self, message):
        try:
            status = json.loads(message.data)
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
            self.good_frames = min(self.reacquire_frames, self.good_frames + 1)
        else:
            self.good_frames = 0
            self.stop()

    def on_path(self, message):
        self.path_header = message.header
        self.path_time = time.monotonic()
        self.path_points = [(point.x, point.y) for point in message.polygon.points]
        self.path_valid = bool(self.path_points)
        if not self.path_valid:
            self.ready_frames = 0
            self.good_frames = 0
            self.stop()

    def on_comm(self, message):
        self.connected = message.comm_state == CommStatus.COMM_STATE_CONNECTED
        self.comm_time = time.monotonic()
        if not self.connected and not self.dry_run:
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
            self.ready_frames = 0
            self.good_frames = 0
            self.stop()

    def blocked_reason(self, now, check_enabled=True):
        if check_enabled and not self.enabled:
            return 'disabled'
        if not self.dry_run and (
                not self.connected or self.comm_time is None
                or now - self.comm_time > self.comm_timeout):
            return 'chassis_disconnected'
        if not self.dry_run and (
                not self.chassis_allowed or self.chassis_time is None
                or now - self.chassis_time > self.input_timeout):
            return 'chassis_not_ready'
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
        if now - self.last_debug_time < 0.2:
            return
        self.last_debug_time = now
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8').copy()
            # ruier_camer_diver's grid is two image rows plus a 64-pixel header.
            width = image.shape[1] // 2
            height = (image.shape[0] - 64) // 2
            if width <= 0 or height <= 0:
                return
            text = f'CONTROL: {self.reason or "waiting"} / forward={self.config.image_forward_direction}'
            if self.geometry and self.geometry['curve_strength'] > 0:
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
                cv2.arrowedLine(image, reference, target, (0, 140, 255), 2)
                cv2.line(image, (reference[0], near[1]), near, (255, 255, 0), 2)
                cv2.putText(image, 'GOAL (not wheel angle)', (target[0] + 12, target[1]),
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
            cv2.putText(image, detail, (10, 47), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (255, 255, 255), 1, cv2.LINE_AA)
            diagnosis = (f'INPUT: {self.input_reason} / LAST BLOCK: {self.last_blocked_reason or "-"}'
                         f' / v={self.previous_v:+.3f} m/s')
            cv2.putText(image, diagnosis, (10, 62), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (0, 220, 255), 1, cv2.LINE_AA)
            output = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            output.header = message.header
            self.debug_pub.publish(output)
            if self.show_window:
                cv2.imshow(self.window_name, image)
                if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                    self.set_parameters([rclpy.parameter.Parameter('enabled', value=False)])
                    self.show_window = False
                    cv2.destroyWindow(self.window_name)
        except (CvBridgeError, cv2.error) as error:
            self.get_logger().warning(f'Control preview failed: {error}')

    def stop(self):
        self.previous_v = self.previous_w = 0.0
        self.cmd_pub.publish(Twist())

    def tick(self):
        now = time.monotonic()
        dt = min(now - self.last_tick, self.control_period * 2.0)
        self.last_tick = now
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
        reason = input_reason if self.enabled else 'disabled'
        if self.enabled:
            if input_reason != 'tracking':
                self.last_blocked_reason = input_reason
            if reason == 'tracking' and self.good_frames < self.reacquire_frames:
                v = w = 0.0
                reason = 'reacquiring'
        else:
            self.good_frames = 0
        if reason == 'tracking':
            v = slew(self.previous_v, v, self.config.linear_acceleration, dt)
            w = smooth_yaw(self.previous_w, w, self.config, dt, self.control_curve_strength)
            if self.config.stop_marker_enabled:
                v, marker_state = self.marker_gate.limit(v, self.config)
                # 横线接近限速和停车必须盖过平滑器，不能为了缓降而滑过目标。
                if marker_state != 'searching':
                    reason = 'stop_marker_'+marker_state.removeprefix('stopped_')
                    if v == 0:
                        w = 0.0
        else:
            v = w = 0.0
            self.control_curve_strength = 0.0
        self.previous_v, self.previous_w = v, w
        command = Twist()
        command.linear.x, command.angular.z = v, w
        self.cmd_pub.publish(command)
        if reason != self.reason:
            self.reason = reason
            self.get_logger().info(f'Line following state: {reason}')
        if now - self.last_state >= 0.5:
            self.last_state = now
            self.state_pub.publish(String(data=json.dumps({
                'state': reason, 'enabled': self.enabled,
                'input_state': self.input_reason, 'last_blocked_reason': self.last_blocked_reason,
                'image_forward_direction': self.config.image_forward_direction,
                'geometry': self.geometry,
                'stop_marker': self.marker_gate.snapshot(self.config),
                'forward_speed_mps': self.config.forward_speed,
                'command_linear_x_mps': v, 'command_angular_z_radps': w,
            }, separators=(',', ':'))))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LineFollowNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
            if node.show_window:
                cv2.destroyWindow(node.window_name)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
