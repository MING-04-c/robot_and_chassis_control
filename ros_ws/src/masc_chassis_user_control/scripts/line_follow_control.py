"""Pure control logic for ruier_camer_diver normalized image errors (no ROS dependency)."""

import math
from statistics import median
from dataclasses import dataclass


@dataclass(frozen=True)
class ControlConfig:
    forward_speed: float = 0.075
    max_forward_speed: float = 0.30
    image_forward_direction: str = 'down'
    vehicle_reference_x_ratio: float = 0.5
    vehicle_reference_y_ratio: float = -0.05
    lookahead_distance_ratio: float = 0.45
    max_near_gap_ratio: float = 0.40
    near_edge_margin_ratio: float = 0.25
    heading_gain: float = 0.35
    curve_entry_time_sec: float = 0.35
    curve_detection_angle_rad: float = 0.12
    curve_error_deadband: float = 0.18
    curve_center_gain_factor: float = 0.35
    curve_heading_gain: float = 0.55
    curve_max_angular_speed: float = 0.28
    curve_angular_acceleration: float = 0.30
    max_heading_error_rad: float = 1.2
    steering_gain: float = 0.65
    steering_sign: float = -1.0
    max_angular_speed: float = 0.25
    error_deadband: float = 0.06
    heading_deadband_rad: float = 0.05
    steering_filter_time_sec: float = 0.2
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

    stop_marker_enabled: bool = True
    stop_marker_target_ratio: float = 0.5
    stop_marker_position_tolerance_ratio: float = 0.008
    stop_marker_slowdown_ratio: float = 0.25
    stop_marker_creep_speed: float = 0.008
    stop_marker_average_frames: int = 5
    stop_marker_average_max_age_sec: float = 1.0
    stop_marker_overshoot_tolerance_ratio: float = 0.02
    stop_marker_min_span_ratio: float = 0.12
    stop_marker_min_arm_ratio: float = 0.035
    stop_marker_angle_tolerance_rad: float = 0.26
    stop_marker_confirmation_frames: int = 3
    stop_marker_continuation_ratio: float = 0.10
    stop_marker_release_ratio: float = 0.08

    def validate(self):
        if self.image_forward_direction not in ('up', 'down'):
            raise ValueError('image_forward_direction must be up or down')
        for name, value in vars(self).items():
            if name == 'image_forward_direction':
                continue
            if isinstance(value, bool):
                continue
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        if not 0.0 <= self.forward_speed <= self.max_forward_speed:
            raise ValueError('forward_speed must be between 0 and max_forward_speed (m/s)')
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
        if not isinstance(self.stop_marker_enabled, bool):
            raise ValueError('stop_marker_enabled must be boolean')
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
                or not 0.001 <= self.stop_marker_creep_speed <= self.max_forward_speed
                or not 0.05 <= self.stop_marker_angle_tolerance_rad <= 0.5
                or not 0.0 <= self.curve_entry_time_sec <= 1.0):
            raise ValueError('invalid stop marker geometry/speed or curve entry time')
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
    if curve_strength > 0:
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
    yaw_limit = config.max_angular_speed + curve_strength * (
        config.curve_max_angular_speed - config.max_angular_speed)
    yaw = sign * (center_gain * yaw_error + heading_gain * heading)
    yaw = max(-yaw_limit, min(yaw_limit, yaw))
    if config.forward_speed == 0.0:
        return 0.0, 0.0, 'speed_zero'
    return config.forward_speed * factor, yaw, 'tracking'


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
    ahead = [(x, y) for x, y in points if forward * (y - ry) > 1.0]
    # 较短缺口通过两端实测线段的几何验证后，作为同一条线续接，不能仅凭单端延长中线。
    ahead.sort(key=lambda point: forward * (point[1] - ry))
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
    visible_reference_y = max(0.0, min(height - 1, ry))
    ground_start_y = max(0.0, min(height - 1, visible_reference_y
                                + forward * config.near_edge_margin_ratio * (height - 1)))
    near_gap = max(0.0, forward * (near_y - ground_start_y))
    if near_gap > config.max_near_gap_ratio * (height - 1):
        return None, 'near_line_missing'
    requested_y = ry + forward * config.lookahead_distance_ratio * (height - 1)
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
        # 出弯后线条可能已是直线，但车身尚未对准；仍按实测走向优先转向。
        turn_evidence = max(abs(bend), abs(path_heading))
        curve_strength = min(1.0, max(0.0,
            (turn_evidence - config.curve_detection_angle_rad) / config.curve_detection_angle_rad))
    error = (near_x - rx) / ((width - 1) / 2.0)
    return {
        'reference_x': rx, 'reference_y': ry,
        'ground_start_y': ground_start_y, 'near_gap_pixels': near_gap,
        'near_x': near_x, 'near_y': near_y,
        'target_x': target_x, 'target_y': target_y,
        'trusted_end_x': ahead[-1][0], 'trusted_end_y': ahead[-1][1],
        'bridged_gaps': bridges,
        'target_gap_interpolated': target_gap_interpolated,
        'path_heading_rad': path_heading, 'bend_angle_rad': bend, 'curve_strength': curve_strength,
        'trusted_span_ratio': abs(ahead[-1][1] - near_y) / (height - 1),
        'trusted_speed_factor': min(1.0, max(config.min_speed_factor,
            abs(ahead[-1][1] - near_y) / (config.lookahead_distance_ratio * (height - 1))))
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
