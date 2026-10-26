"""Bounded exposure policy driven by blue-line quality and image brightness."""

import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class ExposureConfig:
    exposure_min: int = 20
    exposure_max: int = 200
    change_ratio: float = 0.15
    minimum_step: int = 5
    cooldown_sec: float = 1.5
    confirm_bad_checks: int = 3
    max_adjustments_per_episode: int = 8
    quality_min: float = 0.35
    blue_value_low: float = 20.0
    blue_value_high: float = 180.0
    scene_dark_value: float = 55.0
    clipped_fraction_high: float = 0.12
    min_blue_pixels: int = 25
    hue_low: int = 100
    hue_high: int = 130
    saturation_min: int = 70
    blue_excess_min: float = 0.12
    roi_top: float = 0.20
    roi_bottom: float = 0.95
    roi_left: float = 0.05
    roi_right: float = 0.95

    def validate(self):
        for name, value in vars(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name} must be a finite number')
        if not 1 <= self.exposure_min < self.exposure_max:
            raise ValueError('exposure bounds must be positive and ordered')
        if not 0.01 <= self.change_ratio <= 0.5 or not 0.2 <= self.cooldown_sec <= 30:
            raise ValueError('change_ratio must be 0.01..0.5; cooldown_sec must be 0.2..30')
        for name in ('minimum_step', 'confirm_bad_checks', 'max_adjustments_per_episode', 'min_blue_pixels'):
            if getattr(self, name) < 1:
                raise ValueError(f'{name} must be positive')
        if not 0 <= self.hue_low < self.hue_high <= 179:
            raise ValueError('invalid HSV hue interval')
        if not 1 <= self.saturation_min <= 255 or not 0 <= self.blue_excess_min <= 1:
            raise ValueError('invalid blue evidence thresholds')
        if not 0 <= self.blue_value_low < self.blue_value_high <= 255:
            raise ValueError('invalid blue brightness interval')
        if not 0 <= self.scene_dark_value <= 255:
            raise ValueError('invalid scene brightness threshold')
        for name in ('quality_min', 'clipped_fraction_high'):
            if not 0 <= getattr(self, name) <= 1:
                raise ValueError(f'{name} must be 0..1')
        if not 0 <= self.roi_top < self.roi_bottom <= 1 or not 0 <= self.roi_left < self.roi_right <= 1:
            raise ValueError('invalid exposure ROI')


def image_metrics(image, config):
    """Use strong blue evidence, including dark tape, and floor ROI brightness."""
    if image is None or image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError('expected uint8 BGR image')
    width = min(320, image.shape[1])
    small = cv2.resize(image, (width, max(1, round(image.shape[0] * width / image.shape[1]))))
    h, w = small.shape[:2]
    roi = small[int(h * config.roi_top):max(1, int(h * config.roi_bottom)),
                int(w * config.roi_left):max(1, int(w * config.roi_right))]
    if roi.size == 0:
        raise ValueError('empty exposure ROI')
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    blue, green, red = [roi[:, :, index].astype(np.float32) for index in range(3)]
    excess = (2 * blue - green - red) / (blue + green + red + 1)
    selected = ((hsv[:, :, 0] >= config.hue_low) & (hsv[:, :, 0] <= config.hue_high)
                & (hsv[:, :, 1] >= config.saturation_min) & (hsv[:, :, 2] >= 2)
                & (blue > green) & (blue > red) & (excess >= config.blue_excess_min))
    values = hsv[:, :, 2]
    count = int(np.count_nonzero(selected))
    return {
        'blue_pixels': count,
        'blue_value_median': float(np.median(values[selected])) if count else None,
        'scene_value_p70': float(np.percentile(values, 70)),
        'clipped_fraction': float(np.mean(values >= 245)),
    }


def quality_from_status(data, config):
    if not isinstance(data, dict) or not isinstance(data.get('lost'), bool):
        return None
    fraction = data.get('valid_row_fraction')
    if isinstance(fraction, bool) or not isinstance(fraction, (float, int)) or not math.isfinite(fraction):
        return None
    if not 0 <= fraction <= 1:
        return None
    return {
        'lost': data['lost'], 'fraction': fraction,
        'processing_over_budget': data.get('processing_over_budget') is True,
        'stable': (not data['lost'] and fraction >= config.quality_min
                   and data.get('target_estimated') is not True
                   and data.get('target_fallback') is not True
                   and data.get('occlusion_suspected') is not True
                   and data.get('inferred_rows', 0) == 0),
    }


class ExposurePolicy:
    def __init__(self, config):
        config.validate()
        self.config = config
        self.bad_checks = 0
        self.good_checks = 0
        self.adjustments = 0
        self.last_change = -math.inf

    def decide(self, current, metrics, quality, now):
        if quality is None or metrics is None:
            return None, 'waiting_for_observation'
        if quality['processing_over_budget']:
            return None, 'processing_over_budget'
        if quality['stable']:
            self.bad_checks = 0
            self.good_checks += 1
            if self.good_checks >= self.config.confirm_bad_checks:
                self.adjustments = 0
            return None, 'stable_hold'
        self.good_checks = 0
        self.bad_checks += 1
        if self.bad_checks < self.config.confirm_bad_checks:
            return None, 'confirming_uncertainty'
        if now - self.last_change < self.config.cooldown_sec:
            return None, 'settling'
        if self.adjustments >= self.config.max_adjustments_per_episode:
            return None, 'adjustment_limit'
        direction = 0
        value = metrics['blue_value_median']
        if metrics['blue_pixels'] >= self.config.min_blue_pixels and value is not None:
            if value < self.config.blue_value_low:
                direction, reason = 1, 'dark_blue_increase'
            elif value > self.config.blue_value_high:
                direction, reason = -1, 'bright_blue_decrease'
            else:
                return None, 'brightness_ok_uncertain_line'
        elif metrics['clipped_fraction'] > self.config.clipped_fraction_high:
            direction, reason = -1, 'clipped_scene_decrease'
        elif metrics['scene_value_p70'] < self.config.scene_dark_value:
            direction, reason = 1, 'dark_scene_increase'
        else:
            # Missing tape in a normally lit image is not evidence of bad exposure.
            return None, 'no_photometric_cause'
        step = max(self.config.minimum_step, round(current * self.config.change_ratio))
        target = max(self.config.exposure_min, min(self.config.exposure_max, current + direction * step))
        if target == current:
            return None, 'exposure_bound'
        return target, reason

    def applied(self, now):
        self.last_change = now
        self.adjustments += 1
        self.bad_checks = 0
