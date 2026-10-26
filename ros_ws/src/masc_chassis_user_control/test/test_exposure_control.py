#!/usr/bin/env python3
"""Photometric decisions and hardware adapter tests without touching a camera."""

import sys
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from exposure_backend import V4L2Exposure  # noqa: E402
from exposure_control import ExposureConfig, ExposurePolicy, image_metrics, quality_from_status  # noqa: E402


def quality(stable=False):
    return {'stable': stable, 'lost': not stable, 'fraction': 0.0 if not stable else 0.8,
            'processing_over_budget': False}


def metrics(blue=5.0, count=100, scene=100, clipped=0.0):
    return {'blue_pixels': count, 'blue_value_median': blue,
            'scene_value_p70': scene, 'clipped_fraction': clipped}


class ExposureTests(unittest.TestCase):
    def setUp(self):
        self.config = ExposureConfig(confirm_bad_checks=1)
        self.policy = ExposurePolicy(self.config)

    def test_dark_blue_increases_and_bright_blue_decreases(self):
        self.assertEqual(self.policy.decide(60, metrics(), quality(), 0),
                         (69, 'dark_blue_increase'))
        self.assertEqual(self.policy.decide(60, metrics(blue=220), quality(), 0),
                         (51, 'bright_blue_decrease'))

    def test_stable_quality_holds_even_when_tape_is_dark(self):
        self.assertEqual(self.policy.decide(60, metrics(), quality(True), 0),
                         (None, 'stable_hold'))

    def test_no_line_does_not_blindly_increase(self):
        self.assertEqual(self.policy.decide(60, metrics(blue=None, count=0), quality(), 0),
                         (None, 'no_photometric_cause'))
        self.assertGreater(self.policy.decide(60, metrics(blue=None, count=0, scene=20), quality(), 0)[0], 60)
        self.assertLess(self.policy.decide(60, metrics(blue=None, count=0, clipped=0.3), quality(), 0)[0], 60)

    def test_bounds_cooldown_and_episode_limit(self):
        self.assertEqual(self.policy.decide(200, metrics(), quality(), 0), (None, 'exposure_bound'))
        self.policy.applied(0)
        self.assertEqual(self.policy.decide(60, metrics(), quality(), 0.5), (None, 'settling'))
        self.policy.adjustments = self.config.max_adjustments_per_episode
        self.assertEqual(self.policy.decide(60, metrics(), quality(), 2), (None, 'adjustment_limit'))
        self.policy.decide(60, metrics(), quality(True), 3)
        self.assertEqual(self.policy.adjustments, 0)

    def test_confirm_uncertainty_and_reject_processing_delay(self):
        p = ExposurePolicy(replace(self.config, confirm_bad_checks=3))
        for t in (0, 0.5):
            self.assertEqual(p.decide(60, metrics(), quality(), t)[1], 'confirming_uncertainty')
        self.assertEqual(p.decide(60, metrics(), quality(), 1)[0], 69)
        q = quality()
        q['processing_over_budget'] = True
        self.assertEqual(p.decide(60, metrics(), q, 2)[1], 'processing_over_budget')

    def test_roi_metrics_keep_dark_blue_and_reject_gray(self):
        image = np.full((480, 640, 3), 67, dtype=np.uint8)
        image[100:450, 305:335] = (5, 1, 0)
        m = image_metrics(image, self.config)
        self.assertGreater(m['blue_pixels'], 100)
        self.assertAlmostEqual(m['blue_value_median'], 5, delta=1)
        gray = np.full((480, 640, 3), 5, dtype=np.uint8)
        self.assertEqual(image_metrics(gray, self.config)['blue_pixels'], 0)

    def test_bad_quality_data_and_config(self):
        self.assertIsNone(quality_from_status({'lost': True, 'valid_row_fraction': float('nan')}, self.config))
        self.assertIsNone(quality_from_status({}, self.config))
        for values in ({'exposure_min': 300}, {'cooldown_sec': 0},
                       {'change_ratio': float('nan')}, {'roi_top': 1}, {'confirm_bad_checks': 0}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                replace(self.config, **values).validate()

    def test_adapter_uses_device_arguments_and_checks_readback(self):
        with patch('exposure_backend.subprocess.run') as run:
            run.return_value.stdout = 'exposure_time_absolute: 60\n'
            backend = V4L2Exposure('/dev/video0')
            self.assertEqual(backend.read(), 60)
            self.assertEqual(run.call_args.args[0],
                             ['v4l2-ctl', '-d', '/dev/video0', '--get-ctrl=exposure_time_absolute'])
            self.assertNotIn('shell', run.call_args.kwargs)
            backend.minimum, backend.maximum, backend.step = 1, 5000, 1
            with self.assertRaises(RuntimeError):
                backend.set(70)


if __name__ == '__main__':
    unittest.main()
