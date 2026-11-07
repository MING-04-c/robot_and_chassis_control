"""Pure control logic for ruier_camer_diver normalized image errors (no ROS dependency)."""

import math
from statistics import median
from dataclasses import dataclass


@dataclass(frozen=True)
class ControlConfig:
    forward_speed: float = 0.075
    max_forward_speed: float = 0.30
    drive_direction: str = 'forward'
    reverse_max_speed: float = 0.05
    reverse_heading_gain: float = 1.0
    reverse_steering_sign: float = 1.0
    image_forward_direction: str = 'down'
    vehicle_reference_x_ratio: float = 0.5
    vehicle_reference_y_ratio: float = -0.05
    lookahead_distance_ratio: float = 0.45
    curve_lookahead_distance_ratio: float = 0.40
    max_near_gap_ratio: float = 0.40
    near_edge_margin_ratio: float = 0.25
    heading_gain: float = 0.35
    curve_entry_time_sec: float = 0.20
    curve_detection_angle_rad: float = 0.12
    curve_error_deadband: float = 0.30
    curve_center_gain_factor: float = 0.20
    curve_heading_gain: float = 0.70
    curve_max_angular_speed: float = 0.34
    curve_angular_acceleration: float = 0.50
    max_heading_error_rad: float = 1.2
    steering_gain: float = 0.65
    steering_sign: float = -1.0
    max_angular_speed: float = 0.25
    error_deadband: float = 0.04
    heading_deadband_rad: float = 0.05
    steering_filter_time_sec: float = 0.16
    steering_curvature_enter: float = 0.30
    steering_curvature_exit: float = 0.15
    straight_center_curvature_enter: float = 0.10
    straight_center_confirmation_frames: int = 5
    steering_reverse_frames: int = 3
    steering_fast_reverse_curvature: float = 1.5
    curve_slowdown: float = 0.4
    min_speed_factor: float = 0.3
    estimated_speed_factor: float = 0.3
    fallback_speed_factor: float = 0.5
    linear_acceleration: float = 0.08
    angular_acceleration: float = 0.35
    min_valid_row_fraction: float = 0.06
    require_bottom_connected: bool = False
    allow_curved_candidates: bool = True
    min_forward_span_ratio: float = 0.05
    max_trusted_gap_ratio: float = 0.06
    max_bridge_gap_ratio: float = 0.30
    bridge_support_span_ratio: float = 0.10
    bridge_lateral_tolerance_ratio: float = 0.04
    bridge_heading_tolerance_rad: float = 0.25
    bridge_intersection_angle_rad: float = 0.85
    bridge_speed_factor: float = 1.0

    # 起步横线对齐：停车观察，分轴摆正、横移、纵向接近，
    # 只在建立路线原点时使用，不替代正常循迹控制。
    # Low-level unit/integration users opt in explicitly; the packaged launch
    # profile enables this for repeatable physical departures.
    start_alignment_enabled: bool = False
    start_alignment_timeout_sec: float = 45.0
    start_alignment_settle_sec: float = 0.6
    start_alignment_step_distance: float = 0.015
    start_alignment_speed: float = 0.03
    start_alignment_lateral_speed: float = 0.03
    start_alignment_min_lateral_speed: float = 0.02
    start_alignment_min_linear_speed: float = 0.012
    start_alignment_min_angular_speed: float = 0.02
    start_alignment_max_angular_speed: float = 0.12
    start_alignment_lateral_gain: float = 0.45
    start_alignment_lateral_tolerance_ratio: float = 0.02
    start_alignment_heading_tolerance_rad: float = 0.035
    start_alignment_tolerance_ratio: float = 0.015
    start_alignment_max_distance: float = 0.30
    start_alignment_reverse_correction: bool = True
    start_alignment_max_reverse_distance: float = 0.10

    stop_marker_enabled: bool = True
    forward_ignore_start_marker: bool = True
    reverse_ignore_start_marker: bool = True
    reverse_return_to_start: bool = False
    reverse_start_marker_max_travel: float = 0.01
    stop_marker_departure_hold_distance: float = 0.25
    stop_marker_target_ratio: float = 0.5
    stop_marker_position_tolerance_ratio: float = 0.03
    stop_marker_slowdown_ratio: float = 0.25
    stop_marker_creep_speed: float = 0.012
    stop_marker_dropout_grace_sec: float = 0.6
    stop_marker_dropout_max_distance: float = 0.005
    stop_marker_average_frames: int = 3
    stop_marker_average_max_age_sec: float = 1.0
    stop_marker_overshoot_tolerance_ratio: float = 0.06
    stop_marker_min_span_ratio: float = 0.12
    stop_marker_min_arm_ratio: float = 0.035
    stop_marker_angle_tolerance_rad: float = 0.26
    stop_marker_confirmation_frames: int = 2
    stop_marker_continuation_ratio: float = 0.10
    stop_marker_release_ratio: float = 0.08

    def validate(self):
        if self.drive_direction not in ('forward', 'reverse'):
            raise ValueError('drive_direction must be forward or reverse')
        if isinstance(self.reverse_steering_sign, bool) or self.reverse_steering_sign not in (-1., 1.):
            raise ValueError('reverse_steering_sign must be -1 or 1')
        if self.image_forward_direction not in ('up', 'down'):
            raise ValueError('image_forward_direction must be up or down')
        for name, value in vars(self).items():
            if name in ('image_forward_direction', 'drive_direction'):
                continue
            if isinstance(value, bool):
                continue
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        if not 0.0 <= self.forward_speed <= self.max_forward_speed:
            raise ValueError('forward_speed must be between 0 and max_forward_speed (m/s)')
        if not 0 < self.reverse_max_speed <= self.max_forward_speed or self.reverse_heading_gain <= 0:
            raise ValueError('reverse speed limit and heading gain must be positive')
        if not .05 <= self.curve_lookahead_distance_ratio <= 1.:
            raise ValueError('curve lookahead ratio must be .05..1')
        for name in ('max_forward_speed', 'max_angular_speed',
                     'linear_acceleration', 'angular_acceleration', 'max_heading_error_rad',
                     'curve_max_angular_speed', 'curve_angular_acceleration'):
            if getattr(self, name) <= 0.0:
                raise ValueError(f'{name} must be positive')
        if self.heading_gain < 0.0 or self.steering_gain < 0.0 or self.steering_sign not in (-1.0, 1.0):
            raise ValueError('steering_gain >= 0; steering_sign must be -1 or 1')
        for name in ('error_deadband', 'curve_slowdown', 'min_speed_factor',
                     'estimated_speed_factor', 'fallback_speed_factor',
                     'min_valid_row_fraction', 'vehicle_reference_x_ratio',
                     'lookahead_distance_ratio', 'max_near_gap_ratio', 'min_forward_span_ratio',
                     'near_edge_margin_ratio', 'max_trusted_gap_ratio',
                     'max_bridge_gap_ratio', 'bridge_support_span_ratio',
                     'bridge_lateral_tolerance_ratio', 'bridge_speed_factor',
                     'curve_error_deadband', 'curve_center_gain_factor'):
            if not 0.0 <= getattr(self, name) <= 1.0:
                raise ValueError(f'{name} must be between 0 and 1')
        if not all(isinstance(value, bool) for value in (self.stop_marker_enabled,
                self.forward_ignore_start_marker, self.reverse_ignore_start_marker, self.reverse_return_to_start)):
            raise ValueError('stop marker enable/start marker policy must be boolean')
        if not all(isinstance(value, bool) for value in (
                self.start_alignment_enabled, self.start_alignment_reverse_correction)):
            raise ValueError('start alignment policies must be boolean')
        if not 1.0 <= self.start_alignment_timeout_sec <= 120.0:
            raise ValueError('start_alignment_timeout_sec must be 1..120')
        if not 0.2 <= self.start_alignment_settle_sec <= 3.0:
            raise ValueError('start_alignment_settle_sec must be 0.2..3.0')
        if not 0.003 <= self.start_alignment_step_distance <= 0.03:
            raise ValueError('start_alignment_step_distance must be 0.003..0.03m')
        if not 0.005 <= self.start_alignment_speed <= self.max_forward_speed:
            raise ValueError('start_alignment_speed must be 0.005..max_forward_speed')
        if not 0.01 < self.start_alignment_min_linear_speed <= self.start_alignment_speed:
            raise ValueError('start_alignment_min_linear_speed must be >0.01 and <=start_alignment_speed')
        if not 0.01 < self.start_alignment_min_lateral_speed <= self.start_alignment_lateral_speed <= self.max_forward_speed:
            raise ValueError('start alignment lateral speeds must satisfy 0.01 < min <= max <= max_forward_speed')
        lateral_braking_distance = self.start_alignment_min_lateral_speed**2/(2.*self.linear_acceleration)
        if self.start_alignment_step_distance < lateral_braking_distance+.001:
            raise ValueError('start alignment step must allow nominal lateral braking distance plus 1mm tolerance')
        if not 0.01 < self.start_alignment_min_angular_speed <= self.start_alignment_max_angular_speed:
            raise ValueError('start_alignment_min_angular_speed must be >0.01 and <=max_angular_speed')
        if not 0.01 <= self.start_alignment_max_angular_speed <= self.max_angular_speed:
            raise ValueError('start_alignment_max_angular_speed must be 0.01..max_angular_speed')
        if not 0.0 <= self.start_alignment_lateral_gain <= 2.0:
            raise ValueError('start_alignment_lateral_gain must be 0..2')
        if not 0.01 <= self.start_alignment_lateral_tolerance_ratio <= 0.25:
            raise ValueError('start_alignment_lateral_tolerance_ratio must be 0.01..0.25')
        if not 0.02 <= self.start_alignment_heading_tolerance_rad <= 0.5:
            raise ValueError('start_alignment_heading_tolerance_rad must be 0.02..0.5')
        if not 0.002 <= self.start_alignment_tolerance_ratio <= 0.05:
            raise ValueError('start_alignment_tolerance_ratio must be 0.002..0.05')
        if not 0.05 <= self.start_alignment_max_distance <= 1.0:
            raise ValueError('start_alignment_max_distance must be 0.05..1.0m')
        if not 0.01 <= self.start_alignment_max_reverse_distance <= 0.5:
            raise ValueError('start_alignment_max_reverse_distance must be 0.01..0.5m')
        if not .001 <= self.reverse_start_marker_max_travel <= .05:
            raise ValueError('reverse start marker travel must be .001..0.05 m')
        if (isinstance(self.stop_marker_departure_hold_distance, bool)
                or not math.isfinite(self.stop_marker_departure_hold_distance)
                or not .05 <= self.stop_marker_departure_hold_distance <= .5):
            raise ValueError('departure hold distance must be .05..0.5 m')
        if (type(self.stop_marker_average_frames) is not int
                or not 3 <= self.stop_marker_average_frames <= 15):
            raise ValueError('stop_marker_average_frames must be integer 3..15')
        if (not .2 <= self.stop_marker_average_max_age_sec <= 2.
                or not self.stop_marker_position_tolerance_ratio <= self.stop_marker_overshoot_tolerance_ratio <= .1):
            raise ValueError('invalid stop marker average/overshoot limits')
        if (isinstance(self.stop_marker_confirmation_frames, bool)
                or not isinstance(self.stop_marker_confirmation_frames, int)
                or not 2 <= self.stop_marker_confirmation_frames <= 20):
            raise ValueError('stop_marker_confirmation_frames must be integer 2..20')
        for name in ('stop_marker_slowdown_ratio', 'stop_marker_min_span_ratio',
                     'stop_marker_min_arm_ratio', 'stop_marker_continuation_ratio', 'stop_marker_release_ratio'):
            if not 0.01 <= getattr(self, name) <= 0.5:
                raise ValueError(f'{name} must be 0.01..0.5')
        if (not 0.1 <= self.stop_marker_target_ratio <= 0.9
                or not 0.002 <= self.stop_marker_position_tolerance_ratio <= 0.05
                or self.stop_marker_position_tolerance_ratio >= self.stop_marker_slowdown_ratio
                or not 0.01 < self.stop_marker_creep_speed <= min(.03, self.max_forward_speed)
                or not 0.05 <= self.stop_marker_angle_tolerance_rad <= 0.5
                or not 0.0 <= self.curve_entry_time_sec <= 1.0):
            raise ValueError('invalid stop marker geometry/speed or curve entry time')
        if not 0. <= self.stop_marker_dropout_grace_sec <= .6:
            raise ValueError('stop marker dropout grace must be 0..0.6 seconds')
        if not 0. <= self.stop_marker_dropout_max_distance <= .005:
            raise ValueError('stop marker dropout distance must be 0..0.005 meters')
        if not 0.02 <= self.curve_detection_angle_rad <= 0.6:
            raise ValueError('curve_detection_angle_rad must be 0.02..0.6')
        if self.curve_error_deadband < self.error_deadband or self.curve_heading_gain < 0:
            raise ValueError('curve deadband must not shrink; curve_heading_gain must be nonnegative')
        if not -0.5 <= self.vehicle_reference_y_ratio <= 1.5:
            raise ValueError('vehicle_reference_y_ratio must be -0.5..1.5')
        if not 0.0 <= self.heading_deadband_rad <= 0.2:
            raise ValueError('heading_deadband_rad must be 0..0.2')
        if not 0.0 <= self.steering_filter_time_sec <= 1.0:
            raise ValueError('steering_filter_time_sec must be 0..1')
        if not 0. <= self.steering_curvature_exit < self.steering_curvature_enter <= 2.:
            raise ValueError('steering curvature requires 0 <= exit < enter <= 2 (1/m)')
        if not 0. < self.straight_center_curvature_enter <= self.steering_curvature_enter:
            raise ValueError('straight center curvature must be positive and <= normal enter threshold')
        if type(self.straight_center_confirmation_frames) is not int or not 3 <= self.straight_center_confirmation_frames <= 20:
            raise ValueError('straight center confirmation requires integer 3..20 frames')
        if type(self.steering_reverse_frames) is not int or not 2 <= self.steering_reverse_frames <= 6:
            raise ValueError('steering_reverse_frames must be integer 2..6')
        if not self.steering_curvature_enter < self.steering_fast_reverse_curvature <= 10.:
            raise ValueError('fast reverse curvature must exceed enter threshold and be <=10')
        if self.lookahead_distance_ratio <= 0.0:
            raise ValueError('lookahead_distance_ratio must be positive')
        if self.max_trusted_gap_ratio <= 0.0:
            raise ValueError('max_trusted_gap_ratio must be positive')
        if not 0.0 < self.bridge_heading_tolerance_rad <= math.pi / 2:
            raise ValueError('bridge_heading_tolerance_rad must be 0..pi/2')
        if not 0.0 < self.bridge_intersection_angle_rad <= math.pi / 2:
            raise ValueError('bridge_intersection_angle_rad must be 0..pi/2')
        if self.bridge_support_span_ratio <= 0.0:
            raise ValueError('bridge_support_span_ratio must be positive')


def line_command(status, config):
    """Return (forward m/s, yaw rad/s, reason); invalid evidence means zero."""
    if not isinstance(status, dict) or status.get('lost') is not False:
        return 0.0, 0.0, 'line_lost'
    if status.get('processing_over_budget') is not False:
        return 0.0, 0.0, 'processing_over_budget'
    orientation = status.get('orientation')
    if orientation not in ('vertical', 'horizontal'):
        return 0.0, 0.0, 'horizontal_or_unknown_line'
    if orientation == 'horizontal' and not (
            config.allow_curved_candidates and status.get('curve_geometry_valid') is True):
        return 0.0, 0.0, 'horizontal_or_unknown_line'
    if config.require_bottom_connected and status.get('bottom_connected') is not True:
        return 0.0, 0.0, 'bottom_not_connected'
    error = status.get('error_normalized')
    fraction = status.get('valid_row_fraction')
    for value in (error, fraction):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return 0.0, 0.0, 'invalid_observation'
        if not math.isfinite(value):
            return 0.0, 0.0, 'invalid_observation'
    if not -1.0 <= error <= 1.0 or not 0.0 <= fraction <= 1.0:
        return 0.0, 0.0, 'invalid_observation'
    if fraction < config.min_valid_row_fraction:
        return 0.0, 0.0, 'insufficient_rows'
    curve_strength = status.get('curve_strength', 0.0)
    if (isinstance(curve_strength, bool) or not isinstance(curve_strength, (int, float))
            or not math.isfinite(curve_strength) or not 0 <= curve_strength <= 1):
        return 0.0, 0.0, 'invalid_curve_geometry'
    deadband = config.error_deadband + curve_strength * (
        config.curve_error_deadband - config.error_deadband)
    # 弯道允许较大横向偏差，避免一边沿前方转弯，一边过度减速追回画面中心。
    speed_error = (1.0 - curve_strength) * abs(error) + curve_strength * max(0.0, abs(error) - deadband)
    factor = max(config.min_speed_factor, 1.0 - config.curve_slowdown * speed_error)
    if status.get('target_estimated') is True or status.get('occlusion_suspected') is True:
        factor *= config.estimated_speed_factor
    if status.get('target_fallback') is True:
        factor *= config.fallback_speed_factor
    trusted_factor = status.get('trusted_speed_factor', 1.0)
    if (isinstance(trusted_factor, bool) or not isinstance(trusted_factor, (int, float))
            or not math.isfinite(trusted_factor) or not 0 <= trusted_factor <= 1):
        return 0.0, 0.0, 'invalid_observation'
    factor *= trusted_factor
    heading = status.get('heading_error_rad', 0.0)
    if isinstance(heading, bool) or not isinstance(heading, (int, float)) or not math.isfinite(heading):
        return 0.0, 0.0, 'invalid_heading'
    if abs(heading) > config.max_heading_error_rad:
        return 0.0, 0.0, 'heading_out_of_range'
    reversing = config.drive_direction == 'reverse'
    if reversing:
        # 唯一的前置相机不随倒车翻转：用实测近端切线对齐车身，
        # 不能追逐运动方向相反的远处前瞻点，更不能把两项控制一起反号。
        heading = status.get('near_heading_rad')
        if (isinstance(heading, bool) or not isinstance(heading, (int, float))
                or not math.isfinite(heading) or abs(heading) > config.max_heading_error_rad):
            return 0.0, 0.0, 'reverse_heading_unavailable'
    else:
        near_heading = status.get('near_heading_rad')
        if near_heading is not None:
            if (isinstance(near_heading, bool) or not isinstance(near_heading, (int, float))
                    or not math.isfinite(near_heading) or abs(near_heading) > config.max_heading_error_rad):
                return 0., 0., 'invalid_near_heading'
            # 横向项已经纠正了位置偏移；直线方向项用实测切线，
            # 不再把“指向偏移前瞻点”的位置误差重复算成航向纠偏。
            heading = near_heading
    if not reversing and curve_strength > 0:
        path_heading = status.get('path_heading_rad')
        if (isinstance(path_heading, bool) or not isinstance(path_heading, (int, float))
                or not math.isfinite(path_heading) or abs(path_heading) > config.max_heading_error_rad):
            return 0.0, 0.0, 'invalid_curve_geometry'
        heading += curve_strength * (path_heading - heading)
    # 弯道按几何强度连续过渡，避免直线/弯道切换引起突然转向。
    yaw_error = math.copysign(max(0.0, abs(error) - deadband), error)
    heading = math.copysign(max(0.0, abs(heading) - config.heading_deadband_rad), heading)
    # Map image left/right to body yaw using the forward direction from the reference.
    sign = config.steering_sign if config.image_forward_direction == 'up' else -config.steering_sign
    center_gain = config.steering_gain * (1.0 - curve_strength * (1.0 - config.curve_center_gain_factor))
    heading_gain = config.heading_gain + curve_strength * (config.curve_heading_gain - config.heading_gain)
    if reversing:
        heading_gain = config.reverse_heading_gain
    yaw_limit = config.max_angular_speed + curve_strength * (
        config.curve_max_angular_speed - config.max_angular_speed)
    if reversing:
        yaw_limit = min(yaw_limit, config.max_angular_speed)
    # Stanley式分项反馈的图像适配：atan有界纠偏；输入不是米制距离，
    # 输出直接是底盘角速度，不是原论文的阿克曼转角。
    center_term = math.atan(center_gain * yaw_error)
    yaw = sign * ((-center_term if reversing else center_term) + heading_gain * heading)
    yaw = max(-yaw_limit, min(yaw_limit, yaw))
    if reversing:
        # 独立的实车倒车方向标定；标准运动模型使用+1，现场反向时配置-1。
        # 不改变图像安装方向、横线运动方向或横向/朝向分项控制本身。
        yaw *= config.reverse_steering_sign
    if config.forward_speed == 0.0:
        return 0.0, 0.0, 'speed_zero'
    speed = min(config.forward_speed, config.reverse_max_speed) if reversing else config.forward_speed
    return (-speed if reversing else speed) * factor, yaw, 'tracking'


def start_alignment_command(marker, image_width, image_height, config, position_error=None,
                            position_tolerance=None, lateral_error=None, heading_error=None,
                            overshoot_tolerance=None):
    """Return a bounded visual-servo command for centering the departure bar.

    The sign convention follows ``StopMarkerGate``: a positive longitudinal error
    means that the bar is still ahead in the current travel direction.  A negative
    error means the vehicle has passed it; a short reverse correction is allowed
    only when explicitly enabled.  The marker's x error is used only for a small
    yaw correction, while the normal line geometry can provide a more stable yaw
    estimate when available.

    This is the same separation used by mature visual-servo/Pure-Pursuit
    controllers: bounded position error, bounded heading correction, and no
    unbounded pixel-to-metre extrapolation.
    """
    if not isinstance(marker, dict) or image_width <= 1 or image_height <= 1:
        return 0.0, 0.0, 'start_marker_unavailable'
    try:
        x, y = float(marker['x']), float(marker['y'])
    except (KeyError, TypeError, ValueError):
        return 0.0, 0.0, 'start_marker_invalid'
    if not all(math.isfinite(value) for value in (x, y)):
        return 0.0, 0.0, 'start_marker_invalid'
    target = config.stop_marker_target_ratio
    forward = 1.0 if config.image_forward_direction == 'down' else -1.0
    if position_error is None:
        position_error = forward * (y / float(image_height) - target)
    if isinstance(position_error, bool) or not isinstance(position_error, (int, float)) or not math.isfinite(position_error):
        return 0.0, 0.0, 'start_marker_invalid'
    tolerance = (config.stop_marker_position_tolerance_ratio
                 if position_tolerance is None else float(position_tolerance))
    overshoot = (config.stop_marker_overshoot_tolerance_ratio
                 if overshoot_tolerance is None else float(overshoot_tolerance))
    lateral = (2.0 * (x / float(image_width - 1) - config.vehicle_reference_x_ratio)
               if lateral_error is None else float(lateral_error))
    slope = float(marker.get('main_slope', 0.0))
    heading = math.atan(forward * slope) if heading_error is None else float(heading_error)
    if not all(math.isfinite(value) for value in (lateral, heading)):
        return 0.0, 0.0, 'start_marker_invalid'
    sign = config.steering_sign if config.image_forward_direction == 'up' else -config.steering_sign
    center = math.atan(config.start_alignment_lateral_gain * lateral)
    yaw = sign * (center + config.heading_gain * heading)
    if position_error < -tolerance:
        yaw = sign * (-center + config.reverse_heading_gain * heading) * config.reverse_steering_sign
    yaw = max(-config.start_alignment_max_angular_speed,
              min(config.start_alignment_max_angular_speed, yaw))
    if position_error > tolerance:
        return min(config.start_alignment_speed, config.forward_speed), yaw, 'start_alignment_approaching'
    if position_error < -tolerance:
        if config.start_alignment_reverse_correction:
            return -min(config.start_alignment_speed, config.reverse_max_speed), yaw, 'start_alignment_reverse'
        if position_error < -overshoot:
            return 0.0, 0.0, 'start_alignment_overshoot'
        # Without reverse correction, stop inside the bounded overshoot band.
        return 0.0, yaw, 'start_alignment_settling'
    return 0.0, yaw, 'start_alignment_centered'


def reverse_visual_command(status, config, lateral_heading_limit=math.radians(6.)):
    """末段视觉倒车分项反馈：横向标定反号只作用横向项，朝向仍沿用前进相机映射。

    返回 (v,w,state)，单位m/s、rad/s；先复用已有新鲜观测/有效性检查，不追逐车前远点。
    """
    v, _, reason = line_command(status, config)
    if reason != 'tracking' or config.drive_direction != 'reverse':
        return 0., 0., reason if reason != 'tracking' else 'reverse_direction_required'
    error, heading = status['error_normalized'], status['near_heading_rad']
    side = math.copysign(max(0., abs(error)-config.error_deadband), error)
    heading = math.copysign(max(0., abs(heading)-config.heading_deadband_rad), heading)
    sign = config.steering_sign if config.image_forward_direction == 'up' else -config.steering_sign
    # 前置相机观察点不在倒车前方。横向误差只能请求有限的回中偏角，
    # 不能持续压过朝向反馈，使车头越摆越斜；上下安装映射保持不变。
    desired_heading = max(-lateral_heading_limit, min(lateral_heading_limit,
                           config.reverse_steering_sign*math.atan(config.steering_gain*side)/config.reverse_heading_gain))
    yaw = sign*config.reverse_heading_gain*(heading-desired_heading)
    return v, max(-config.max_angular_speed, min(config.max_angular_speed, yaw)), reason


def slew(previous, target, acceleration, dt):
    limit = acceleration * max(0.0, dt)
    return previous + max(-limit, min(limit, target - previous))


def smooth_yaw(previous, target, config, dt, curve_strength=0.0):
    """Low-pass steering while tracking; stop conditions bypass this helper."""
    dt = max(0.0, dt)
    tau = config.steering_filter_time_sec
    alpha = -math.expm1(-dt / tau) if tau > 0.0 else 1.0
    filtered = previous + alpha * (target - previous)
    acceleration = config.angular_acceleration + curve_strength * (
        config.curve_angular_acceleration - config.angular_acceleration)
    return slew(previous, filtered, acceleration, dt)


class SteeringDirectionGuard:
    """对底盘转弯曲率做迟滞和新帧换向确认，不直接控制单轮舵机角度。"""
    def __init__(self):
        self.reset()

    def reset(self):
        self.direction = self.pending_direction = self.pending_frames = 0
        self.active = False
        self.sample = None
        self.curvature = self.raw_curvature = 0.
        self.state = 'straight'
        self.center_direction = self.center_frames = 0
        self.center_ready = False

    def update(self, raw_yaw, config, sample, dt, error_normalized=None, curve_strength=0.):
        # raw_yaw是在配置速度下算出的需求，不用已经减速的v作分母，
        # 否则低速接近或视觉减速会放大曲率，要求轮子突然转大角度。
        nominal = min(config.forward_speed, config.reverse_max_speed) if config.drive_direction == 'reverse' else config.forward_speed
        nominal = -nominal if config.drive_direction == 'reverse' else nominal
        if abs(nominal) < 1e-9:
            self.reset()
            return 0.
        self.raw_curvature = raw_yaw/nominal
        magnitude = abs(self.raw_curvature)
        direction = 1 if self.raw_curvature > 0 else -1
        target = 0.
        new_frame = sample != self.sample
        self.sample = sample
        # 小转向只在直线持续偏向同一侧时放行，不整体降低抗摆动门槛。
        # 只累计新图；进入弯道、回到中心带、失效或复位时撤销回中余量。
        valid_error = (not isinstance(error_normalized, bool)
                       and isinstance(error_normalized, (int, float))
                       and math.isfinite(error_normalized) and abs(error_normalized) <= 1.)
        eligible = (valid_error and config.drive_direction == 'forward'
                    and 0. <= curve_strength <= .25
                    and abs(error_normalized) > config.error_deadband)
        if not eligible:
            self.center_direction = self.center_frames = 0
            self.center_ready = False
        elif new_frame:
            side = 1 if error_normalized > 0. else -1
            if side != self.center_direction:
                self.center_direction, self.center_frames = side, 0
                self.center_ready = False
            if abs(error_normalized) >= 1.25*config.error_deadband:
                self.center_frames += 1
                self.center_ready = self.center_frames >= config.straight_center_confirmation_frames
            elif not self.center_ready:
                self.center_frames = 0
        enter = config.straight_center_curvature_enter if self.center_ready else config.steering_curvature_enter
        exit_threshold = min(config.steering_curvature_exit, enter*.5)
        if magnitude <= exit_threshold:
            self.pending_direction = self.pending_frames = 0
            self.active = False
            self.state = 'deadband'
        elif self.direction == 0:
            if magnitude >= enter:
                self.direction = direction
                self.active = True
                target = self.raw_curvature
                self.state = 'turning'
            else:
                self.state = 'deadband'
        elif direction == self.direction:
            self.pending_direction = self.pending_frames = 0
            if self.active or magnitude >= enter:
                self.active = True
                target = self.raw_curvature
                self.state = 'turning'
            else:
                self.state = 'deadband'
        elif magnitude >= enter:
            self.active = False
            if direction != self.pending_direction:
                self.pending_direction, self.pending_frames = direction, int(new_frame)
            elif new_frame:
                self.pending_frames += 1
            required = (2 if magnitude >= config.steering_fast_reverse_curvature
                        else config.steering_reverse_frames)
            self.state = 'confirming_reverse'
            # 确认期间向直行衰减，不继续维持旧转向；强反向弯也至少要两张新图。
            if self.pending_frames >= required:
                self.direction = direction
                self.active = True
                self.pending_direction = self.pending_frames = 0
                target = self.raw_curvature
                self.state = 'turning'
        else:
            self.active = False
            self.pending_direction = self.pending_frames = 0
            self.state = 'deadband'
        tau = config.steering_filter_time_sec
        alpha = -math.expm1(-max(0., dt)/tau) if tau > 0. else 1.
        self.curvature += alpha*(target-self.curvature)
        if target == 0. and abs(self.curvature) < exit_threshold*.5:
            self.curvature = 0.
        return self.curvature


def coordinated_velocity(previous_v, previous_w, target_v, target_w, config, dt, curve_strength=0.):
    """统一插值比例满足线/角加速度约束，避免起步/减速时独立滤波改变转弯比例。"""
    if abs(target_v) < 1e-9:
        return 0., 0.
    dv, dw = target_v-previous_v, target_w-previous_w
    angular_limit = config.angular_acceleration+curve_strength*(
        config.curve_angular_acceleration-config.angular_acceleration)
    scale = 1.
    for delta, limit in ((dv, config.linear_acceleration), (dw, angular_limit)):
        if abs(delta) > 1e-12:
            scale = min(scale, limit*max(0., dt)/abs(delta))
    return previous_v+scale*dv, previous_w+scale*dw


def lookahead_target(ahead, bridges, ry, forward, distance_ratio, height, config):
    """在可信实测段或已验证的缺口内连续取点，不向不可见路径外推。"""
    requested_y = ry + forward * distance_ratio * (height - 1)
    target_x, target_y = min(ahead, key=lambda p: abs(p[1] - requested_y))
    # 在相邻实测行之间连续取前瞻点，避免目标在两行之间来回跳而导致转向顿挫。
    # 未批准的缺口仍由 trusted 前缀截断；这里不向可见范围以外延长。
    for first, second in zip(ahead, ahead[1:]):
        if (min(first[1], second[1]) <= requested_y <= max(first[1], second[1])
                and abs(second[1]-first[1]) <= config.max_trusted_gap_ratio * (height-1)):
            fraction = (requested_y-first[1])/(second[1]-first[1]) if second[1] != first[1] else 0.0
            target_x = first[0] + fraction*(second[0]-first[0])
            target_y = requested_y
            break
    target_gap_interpolated = False
    for bridge in bridges:
        start, end = bridge['from'], bridge['to']
        if min(start[1], end[1]) <= requested_y <= max(start[1], end[1]):
            # 只在已通过两端几何验证的缺口内部插值，不向无实测支撑的区域外推。
            fraction = (requested_y - start[1]) / (end[1] - start[1])
            target_x = start[0] + fraction * (end[0] - start[0])
            target_y = requested_y
            target_gap_interpolated = True
            break
    return target_x, target_y, requested_y, target_gap_interpolated


def tracking_geometry(points, width, height, config):
    """Choose measured near/forward points relative to a virtual body reference.

    Ratios describe image coordinates only. The off-image reference is a tuning
    model, not a calibrated ground-plane projection of the real chassis centre.
    Missing line sections are never extrapolated to the invisible chassis.
    """
    if (not isinstance(width, (int, float)) or not isinstance(height, (int, float))
            or not math.isfinite(width) or not math.isfinite(height)
            or width <= 1 or height <= 1):
        return None, 'invalid_image_size'
    if len(points) < 2 or any(not math.isfinite(v) for p in points for v in p):
        return None, 'insufficient_heading_points'
    if any(x < 0 or x >= width or y < 0 or y >= height for x, y in points):
        return None, 'invalid_centerline'
    forward = -1.0 if config.image_forward_direction == 'up' else 1.0
    rx = config.vehicle_reference_x_ratio * (width - 1)
    ry = config.vehicle_reference_y_ratio * (height - 1)
    visible_reference_y = max(0.0, min(height - 1, ry))
    ground_start_y = max(0.0, min(height - 1, visible_reference_y
                                + forward * config.near_edge_margin_ratio * (height - 1)))
    # 车轮遮挡区只用于显示，不能让其中几颗蓝点成为控制段起点，
    # 否则全图有几十个实测点，可信前缀仍可能在第一处小缺口被截成1~2点。
    ahead = [(x, y) for x, y in points if forward * (y - ry) > 1.0
             and forward*(y-ground_start_y) >= 0.]
    # 较短缺口通过两端实测线段的几何验证后，作为同一条线续接，不能仅凭单端延长中线。
    ahead.sort(key=lambda point: forward * (point[1] - ry))
    minimum_span = max(10., config.min_forward_span_ratio*(height-1),
                       config.min_valid_row_fraction*(height-1))
    ignored_fragments = 0
    # 选第一个自身可用的连续实测段，跳过本来就无法控制的前导碎点。
    # 不跳过有效近端线段，不把空白补成线，后面的近端距离上限仍生效。
    start = 0
    while start < len(ahead):
        end = start+1
        while (end < len(ahead) and
               abs(ahead[end][1]-ahead[end-1][1]) <= config.max_trusted_gap_ratio*(height-1)):
            end += 1
        span = abs(ahead[end-1][1]-ahead[start][1])
        if end-start >= 2 and span >= minimum_span:
            break
        if end == len(ahead):
            break  # 全部只有短线时，保留原来的不足/停车判断。
        next_end = end+1
        while (next_end < len(ahead) and
               abs(ahead[next_end][1]-ahead[next_end-1][1]) <= config.max_trusted_gap_ratio*(height-1)):
            next_end += 1
        fit = fit_axis_line(ahead[end:min(next_end, end+10)])
        if (fit is None or abs(ahead[next_end-1][1]-ahead[end][1]) < minimum_span
                or abs(ahead[end][1]-ahead[end-1][1]) > config.max_bridge_gap_ratio*(height-1)
                or any(abs(x-(fit[0]*y+fit[1])) > config.bridge_lateral_tolerance_ratio*(width-1)
                       for x, y in ahead[start:end])):
            break  # 不把短近端线跳到另一根偏移的远线；必须与后段的实测走向相容。
        ignored_fragments += end-start
        start = end
    ahead = ahead[start:]
    trusted = []
    bridges = []
    bridged_pixels = 0.0
    for index, point in enumerate(ahead):
        gap = forward * (point[1] - trusted[-1][1]) if trusted else 0.0
        if gap > config.max_trusted_gap_ratio * (height - 1):
            if (bridged_pixels + gap > config.max_bridge_gap_ratio * (height - 1)
                    or not bridge_supported(trusted, ahead[index:], width, height, config)):
                break
            bridges.append({'from': trusted[-1], 'to': point})
            bridged_pixels += gap
        trusted.append(point)
    ahead = trusted
    if len(ahead) < 2 or max(y for _, y in ahead) - min(y for _, y in ahead) < 10:
        return None, 'no_forward_line'
    if max(y for _, y in ahead) - min(y for _, y in ahead) < config.min_forward_span_ratio * (height - 1):
        return None, 'insufficient_forward_span'
    near_x, near_y = min(ahead, key=lambda p: abs(p[1] - ry))
    near_gap = max(0.0, forward * (near_y - ground_start_y))
    if near_gap > config.max_near_gap_ratio * (height - 1):
        return None, 'near_line_missing'
    target_x, target_y, requested_y, target_gap_interpolated = lookahead_target(
        ahead, bridges, ry, forward, config.lookahead_distance_ratio, height, config)
    heading = math.atan2(target_x - rx, forward * (target_y - ry))
    # 近端实测切线与近端到前瞻点的走向相比较，区分真弯道与整条直线平移。
    support = [ahead[0]]
    for point in ahead[1:]:
        if abs(point[1] - support[-1][1]) > config.max_trusted_gap_ratio * (height - 1):
            break
        support.append(point)
        if abs(point[1] - support[0][1]) >= max(12.0, 0.06 * (height - 1)):
            break
    path_heading = heading
    bend = curve_strength = 0.0
    fit = fit_axis_line(support)
    if (fit is not None and len(support) >= 3
            and abs(support[-1][1] - support[0][1]) >= max(12.0, 0.06 * (height - 1))
            and forward * (target_y - near_y) > 1.0):
        slope, intercept = fit
        path_heading = math.atan2(target_x - (intercept + slope * near_y), forward * (target_y - near_y))
        bend = path_heading - math.atan(forward * slope)
        # 斜直线表示车身尚未对准，不代表路径弯曲。将绝对航向当弯道会
        # 放宽横向死区并削弱回中，摆正后又突然恢复强回中，引发反复转向。
        # 航向反馈仍由 near_heading 保留；只有前后切线变化才放宽弯道余量。
        turn_evidence = abs(bend)
        curve_strength = min(1.0, max(0.0,
            (turn_evidence - config.curve_detection_angle_rad) / config.curve_detection_angle_rad))
    near_heading = (math.atan(forward * fit[0]) if fit is not None and len(support) >= 3
                    and abs(support[-1][1]-support[0][1]) >= max(12., .06*(height-1)) else None)
    # 用固定预览判别弯道，再连续缩短跟踪前瞻，避免反复切换识别强度。
    lookahead_ratio = config.lookahead_distance_ratio - curve_strength * max(
        0., config.lookahead_distance_ratio-config.curve_lookahead_distance_ratio)
    if config.drive_direction == 'forward' and curve_strength > 0:
        target_x, target_y, requested_y, target_gap_interpolated = lookahead_target(
            ahead, bridges, ry, forward, lookahead_ratio, height, config)
        heading = math.atan2(target_x-rx, forward*(target_y-ry))
        if fit is not None and forward*(target_y-near_y) > 1.:
            path_heading = math.atan2(target_x-(fit[1]+fit[0]*near_y), forward*(target_y-near_y))
        elif near_heading is not None:
            path_heading = near_heading
    error = (near_x - rx) / ((width - 1) / 2.0)
    supported_lookahead = forward*(ahead[-1][1]-requested_y) >= 0. and forward*(requested_y-near_y) > 0.
    # 只按可见近端到前瞻点所需的支撑长度调速，不能把遮挡区的虚拟距离也算成缺线。
    visible_lookahead = max(minimum_span, forward*(requested_y-near_y))
    return {
        'reference_x': rx, 'reference_y': ry,
        'ground_start_y': ground_start_y, 'near_gap_pixels': near_gap,
        'near_x': near_x, 'near_y': near_y,
        'target_x': target_x, 'target_y': target_y,
        'trusted_end_x': ahead[-1][0], 'trusted_end_y': ahead[-1][1],
        'bridged_gaps': bridges,
        'ignored_leading_points': ignored_fragments,
        'lookahead_supported': supported_lookahead,
        'measured_points': len(ahead),
        'target_gap_interpolated': target_gap_interpolated,
        'path_heading_rad': path_heading, 'near_heading_rad': near_heading,
        'effective_lookahead_ratio': lookahead_ratio if config.drive_direction == 'forward' else None,
        'bend_angle_rad': bend, 'curve_strength': curve_strength,
        'trusted_span_ratio': abs(ahead[-1][1] - near_y) / (height - 1),
        'trusted_speed_factor': min(1.0, max(config.min_speed_factor,
            abs(ahead[-1][1] - near_y) / visible_lookahead))
            * (config.bridge_speed_factor if bridges else 1.0),
        'error_normalized': max(-1.0, min(1.0, error)),
        'heading_error_rad': heading,
        'target_fallback': abs(target_y - requested_y) > 0.1 * (height - 1),
    }, None


def fit_axis_line(samples):
    """输入实测 (x,y) 点，输出稳健的 x=slope*y+intercept；缺少跨度时返回 None。"""
    slopes = [(a[0] - b[0]) / (a[1] - b[1])
              for i, a in enumerate(samples) for b in samples[i + 1:]
              if abs(a[1] - b[1]) >= 1.0]
    if not slopes:
        return None
    slope = median(slopes)
    return slope, median(x - slope * y for x, y in samples)


def straight_image_axis(points, width, height, geometry, config):
    """从可信实测段拟合直线；返回近端居中误差/轴向角，弯曲或支撑不足返回None。

    使用图像归一化坐标，不把像素当米，也不向相机视野以外推算轨迹。
    """
    if geometry is None or geometry['bridged_gaps']:
        return None
    low, high = sorted((geometry['near_y'], geometry['trusted_end_y']))
    samples = sorted(((x, y) for x, y in points if low <= y <= high), key=lambda point: point[1])
    if len(samples) < 6 or high-low < .20*(height-1):
        return None
    # 限制稳健拟合点数，避免在50Hz控制循环中进行大规模两两斜率计算。
    samples = [samples[round(i*(len(samples)-1)/15)] for i in range(16)] if len(samples) > 16 else samples
    fit = fit_axis_line(samples)
    if fit is None:
        return None
    slope, intercept = fit
    residual = max(abs(x-(slope*y+intercept)) for x, y in samples)
    if residual > .0125*(width-1):
        return None
    forward = -1. if config.image_forward_direction == 'up' else 1.
    return {'error_normalized': (slope*geometry['near_y']+intercept-geometry['reference_x'])/((width-1)/2),
            'near_heading_rad': math.atan(forward*slope), 'fit_residual_px': residual,
            'span_px': high-low}


def bridge_supported(near, far, width, height, config):
    """缺口两端必须各有实测段，切向接近且外推位置相容；这些是图像几何约束。"""
    def tangent(samples):
        origin = samples[0]
        previous = origin
        support = [origin]
        for sample in samples[1:]:
            if abs(sample[1] - previous[1]) > config.max_trusted_gap_ratio * (height - 1):
                return None
            previous = sample
            support.append(sample)
            if abs(sample[1] - origin[1]) >= config.bridge_support_span_ratio * (height - 1):
                return fit_axis_line(support)
        return None

    near_fit = tangent(list(reversed(near)))
    far_fit = tangent(far)
    if near_fit is None or far_fit is None:
        return False
    near_slope, near_intercept = near_fit
    far_slope, far_intercept = far_fit
    near_x = near_intercept + near_slope * near[-1][1]
    far_x = far_intercept + far_slope * far[0][1]
    turn = abs(math.atan(near_slope) - math.atan(far_slope))
    dx = far_x - near_x
    dy = far[0][1] - near[-1][1]
    tolerance = config.bridge_lateral_tolerance_ratio * (width - 1)
    # 同时检查两端切线，防止两条平行但错开的胶带被误接成一条线。
    aligned = (turn <= config.bridge_heading_tolerance_rad and
               abs(dx - near_slope * dy) <= tolerance and
               abs(dx - far_slope * dy) <= tolerance)
    if aligned:
        return True
    # 两段切线确实在缺口内部相交，才作为一条有转角的线续接。
    if turn > config.bridge_intersection_angle_rad or abs(near_slope - far_slope) < 1e-6:
        return False
    intersection_y = (far_intercept - near_intercept) / (near_slope - far_slope)
    intersection_x = near_intercept + near_slope * intersection_y
    return (min(near[-1][1], far[0][1]) <= intersection_y <= max(near[-1][1], far[0][1])
            and 0 <= intersection_x < width)


def motion_preview(v, w, config, seconds=1.0, steps=20, scale=1500.0):
    """Schematic pixel offsets from commanded body twist, not a ground projection."""
    forward = -1.0 if config.image_forward_direction == 'up' else 1.0
    yaw_sign = config.steering_sign if forward < 0 else -config.steering_sign
    dt = seconds / steps
    x = y = heading = 0.0
    points = [(x, y)]
    for _ in range(steps):
        middle = heading + yaw_sign * w * dt / 2.0
        x += scale * v * math.sin(middle) * dt
        y += forward * scale * v * math.cos(middle) * dt
        heading += yaw_sign * w * dt
        points.append((x, y))
    return points
