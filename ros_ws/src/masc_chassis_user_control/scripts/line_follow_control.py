"""Pure control logic for ruier_camer_diver normalized image errors (no ROS dependency)."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ControlConfig:
    forward_speed: float = 0.05
    max_forward_speed: float = 0.30
    image_forward_direction: str = 'down'
    vehicle_reference_x_ratio: float = 0.5
    vehicle_reference_y_ratio: float = -0.05
    lookahead_distance_ratio: float = 0.45
    max_near_gap_ratio: float = 0.35
    near_edge_margin_ratio: float = 0.25
    heading_gain: float = 0.5
    max_heading_error_rad: float = 1.2
    steering_gain: float = 1.2
    steering_sign: float = -1.0
    max_angular_speed: float = 0.5
    error_deadband: float = 0.03
    heading_deadband_rad: float = 0.035
    steering_filter_time_sec: float = 0.2
    curve_slowdown: float = 0.7
    min_speed_factor: float = 0.3
    estimated_speed_factor: float = 0.3
    fallback_speed_factor: float = 0.5
    linear_acceleration: float = 0.15
    angular_acceleration: float = 0.5
    min_valid_row_fraction: float = 0.2
    require_bottom_connected: bool = False
    allow_curved_candidates: bool = True
    min_forward_span_ratio: float = 0.15

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
                     'linear_acceleration', 'angular_acceleration', 'max_heading_error_rad'):
            if getattr(self, name) <= 0.0:
                raise ValueError(f'{name} must be positive')
        if self.heading_gain < 0.0 or self.steering_gain < 0.0 or self.steering_sign not in (-1.0, 1.0):
            raise ValueError('steering_gain >= 0; steering_sign must be -1 or 1')
        for name in ('error_deadband', 'curve_slowdown', 'min_speed_factor',
                     'estimated_speed_factor', 'fallback_speed_factor',
                     'min_valid_row_fraction', 'vehicle_reference_x_ratio',
                     'lookahead_distance_ratio', 'max_near_gap_ratio', 'min_forward_span_ratio',
                     'near_edge_margin_ratio'):
            if not 0.0 <= getattr(self, name) <= 1.0:
                raise ValueError(f'{name} must be between 0 and 1')
        if not -0.5 <= self.vehicle_reference_y_ratio <= 1.5:
            raise ValueError('vehicle_reference_y_ratio must be -0.5..1.5')
        if not 0.0 <= self.heading_deadband_rad <= 0.2:
            raise ValueError('heading_deadband_rad must be 0..0.2')
        if not 0.0 <= self.steering_filter_time_sec <= 1.0:
            raise ValueError('steering_filter_time_sec must be 0..1')
        if self.lookahead_distance_ratio <= 0.0:
            raise ValueError('lookahead_distance_ratio must be positive')


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
    factor = max(config.min_speed_factor, 1.0 - config.curve_slowdown * abs(error))
    if status.get('target_estimated') is True or status.get('occlusion_suspected') is True:
        factor *= config.estimated_speed_factor
    if status.get('target_fallback') is True:
        factor *= config.fallback_speed_factor
    heading = status.get('heading_error_rad', 0.0)
    if isinstance(heading, bool) or not isinstance(heading, (int, float)) or not math.isfinite(heading):
        return 0.0, 0.0, 'invalid_heading'
    if abs(heading) > config.max_heading_error_rad:
        return 0.0, 0.0, 'heading_out_of_range'
    # Continuous deadbands avoid a command jump when noise crosses a boundary.
    yaw_error = math.copysign(max(0.0, abs(error) - config.error_deadband), error)
    heading = math.copysign(max(0.0, abs(heading) - config.heading_deadband_rad), heading)
    # Map image left/right to body yaw using the forward direction from the reference.
    sign = config.steering_sign if config.image_forward_direction == 'up' else -config.steering_sign
    yaw = sign * (config.steering_gain * yaw_error + config.heading_gain * heading)
    yaw = max(-config.max_angular_speed, min(config.max_angular_speed, yaw))
    if config.forward_speed == 0.0:
        return 0.0, 0.0, 'speed_zero'
    return config.forward_speed * factor, yaw, 'tracking'


def slew(previous, target, acceleration, dt):
    limit = acceleration * max(0.0, dt)
    return previous + max(-limit, min(limit, target - previous))


def smooth_yaw(previous, target, config, dt):
    """Low-pass steering while tracking; stop conditions bypass this helper."""
    dt = max(0.0, dt)
    tau = config.steering_filter_time_sec
    alpha = -math.expm1(-dt / tau) if tau > 0.0 else 1.0
    filtered = previous + alpha * (target - previous)
    return slew(previous, filtered, config.angular_acceleration, dt)


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
    heading = math.atan2(target_x - rx, forward * (target_y - ry))
    error = (near_x - rx) / ((width - 1) / 2.0)
    return {
        'reference_x': rx, 'reference_y': ry,
        'ground_start_y': ground_start_y, 'near_gap_pixels': near_gap,
        'near_x': near_x, 'near_y': near_y,
        'target_x': target_x, 'target_y': target_y,
        'error_normalized': max(-1.0, min(1.0, error)),
        'heading_error_rad': heading,
        'target_fallback': abs(target_y - requested_y) > 0.1 * (height - 1),
    }, None


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
