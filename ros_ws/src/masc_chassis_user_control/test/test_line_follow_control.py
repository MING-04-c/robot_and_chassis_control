#!/usr/bin/env python3
"""Unit checks for direction, lost-line stop, slowdown and parameter validation."""

import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from line_follow_control import ControlConfig, line_command, slew  # noqa: E402


def observation(**updates):
    value = {
        'lost': False, 'processing_over_budget': False, 'orientation': 'vertical',
        'bottom_connected': True, 'error_normalized': 0.0,
        'valid_row_fraction': 0.8, 'target_estimated': False,
        'target_fallback': False, 'occlusion_suspected': False,
    }
    value.update(updates)
    return value


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.config = ControlConfig()

    def test_straight_and_direction(self):
        self.assertEqual(line_command(observation(), self.config), (0.05, 0.0, 'tracking'))
        self.assertLess(line_command(observation(error_normalized=0.5), self.config)[1], 0)
        self.assertGreater(line_command(observation(error_normalized=-0.5), self.config)[1], 0)
        config = replace(self.config, steering_sign=1.0)
        self.assertGreater(line_command(observation(error_normalized=0.5), config)[1], 0)

    def test_unusable_observations_stop(self):
        for values in (
            {'lost': True}, {'processing_over_budget': True},
            {'orientation': 'horizontal'}, {'bottom_connected': False},
            {'valid_row_fraction': 0.01}, {'error_normalized': math.nan},
            {'error_normalized': True}, {'error_normalized': 1.1},
        ):
            with self.subTest(values=values):
                self.assertEqual(line_command(observation(**values), self.config)[:2], (0, 0))
        self.assertEqual(line_command(None, self.config)[:2], (0, 0))

    def test_slowdown_and_limits(self):
        straight = line_command(observation(), self.config)[0]
        curve = line_command(observation(error_normalized=1.0), self.config)
        estimate = line_command(observation(target_estimated=True), self.config)[0]
        self.assertLess(curve[0], straight)
        self.assertLess(estimate, straight)
        self.assertLessEqual(abs(curve[1]), self.config.max_angular_speed)
        self.assertEqual(line_command(observation(error_normalized=0.5),
                                     replace(self.config, forward_speed=0.0))[:2], (0, 0))

    def test_invalid_parameters(self):
        self.config.validate()
        for values in ({'forward_speed': -1}, {'forward_speed': 1},
                       {'forward_speed': math.nan}, {'steering_sign': 0},
                       {'estimated_speed_factor': 2}, {'linear_acceleration': 0}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                replace(self.config, **values).validate()

    def test_acceleration_limit(self):
        self.assertAlmostEqual(slew(0.0, 0.1, 0.15, 0.02), 0.003)
        self.assertAlmostEqual(slew(0.01, -0.1, 1.0, 0.02), -0.01)


if __name__ == '__main__':
    unittest.main()
