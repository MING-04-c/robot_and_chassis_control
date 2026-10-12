"""Pure control logic for camer_diver normalized image errors (no ROS dependency)."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ControlConfig:
    forward_speed: float = 0.05
    max_forward_speed: float = 0.30
    steering_gain: float = 0.8
    steering_sign: float = -1.0
    max_angular_speed: float = 0.5
    error_deadband: float = 0.02
    curve_slowdown: float = 0.7
    min_speed_factor: float = 0.3
    estimated_speed_factor: float = 0.3
    fallback_speed_factor: float = 0.5
    linear_acceleration: float = 0.15
    angular_acceleration: float = 1.0
    min_valid_row_fraction: float = 0.2
    require_bottom_connected: bool = True

    def validate(self):
        for name, value in vars(self).items():
            if isinstance(value, bool):
                continue
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        if not 0.0 <= self.forward_speed <= self.max_forward_speed:
            raise ValueError('forward_speed must be between 0 and max_forward_speed (m/s)')
        for name in ('max_forward_speed', 'max_angular_speed',
                     'linear_acceleration', 'angular_acceleration'):
            if getattr(self, name) <= 0.0:
                raise ValueError(f'{name} must be positive')
        if self.steering_gain < 0.0 or self.steering_sign not in (-1.0, 1.0):
            raise ValueError('steering_gain >= 0; steering_sign must be -1 or 1')
        for name in ('error_deadband', 'curve_slowdown', 'min_speed_factor',
                     'estimated_speed_factor', 'fallback_speed_factor',
                     'min_valid_row_fraction'):
            if not 0.0 <= getattr(self, name) <= 1.0:
                raise ValueError(f'{name} must be between 0 and 1')


def line_command(status, config):
    """Return (forward m/s, yaw rad/s, reason); invalid evidence means zero."""
    if not isinstance(status, dict) or status.get('lost') is not False:
        return 0.0, 0.0, 'line_lost'
    if status.get('processing_over_budget') is not False:
        return 0.0, 0.0, 'processing_over_budget'
    if status.get('orientation') != 'vertical':
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
    yaw_error = 0.0 if abs(error) <= config.error_deadband else error
    yaw = config.steering_sign * config.steering_gain * yaw_error
    yaw = max(-config.max_angular_speed, min(config.max_angular_speed, yaw))
    if config.forward_speed == 0.0:
        return 0.0, 0.0, 'speed_zero'
    return config.forward_speed * factor, yaw, 'tracking'


def slew(previous, target, acceleration, dt):
    limit = acceleration * max(0.0, dt)
    return previous + max(-limit, min(limit, target - previous))
