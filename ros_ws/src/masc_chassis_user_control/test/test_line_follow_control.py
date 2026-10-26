#!/usr/bin/env python3
"""Unit checks for direction, lost-line stop, slowdown and parameter validation."""

import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from line_follow_control import ControlConfig, line_command, motion_preview, tracking_geometry, slew, smooth_yaw  # noqa: E402


def observation(**updates):
    value = {
        'lost': False, 'processing_over_budget': False, 'orientation': 'vertical',
        'image_width': 640, 'image_height': 480,
        'bottom_connected': True, 'error_normalized': 0.0,
        'valid_row_fraction': 0.8, 'target_estimated': False,
        'target_fallback': False, 'occlusion_suspected': False,
    }
    value.update(updates)
    return value


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.config = ControlConfig(image_forward_direction='up', require_bottom_connected=True, steering_sign=-1.0, vehicle_reference_y_ratio=1.05)

    def test_small_noise_and_continuous_deadband(self):
        for error, heading in ((0.02, 0.02), (-0.02, -0.02)):
            self.assertEqual(line_command(observation(
                error_normalized=error, heading_error_rad=heading), self.config)[1], 0.0)
        # A just-visible error produces a tiny correction, not a step at the threshold.
        w = line_command(observation(error_normalized=self.config.error_deadband + 0.001), self.config)[1]
        self.assertAlmostEqual(abs(w), 0.0012)
        self.assertGreater(abs(line_command(observation(error_normalized=0.4), self.config)[1]), 0.1)

    def test_steering_filter_rejects_jitter_and_tracks_curve(self):
        steady = noisy = 0.0
        noise = []
        for index in range(100):
            noisy = smooth_yaw(noisy, 0.05 if index % 2 == 0 else -0.05, self.config, 0.02)
            noise.append(abs(noisy))
            steady = smooth_yaw(steady, 0.2, self.config, 0.02)
        self.assertLess(max(noise), 0.01)
        self.assertAlmostEqual(steady, 0.2, places=3)
        self.assertLessEqual(abs(smooth_yaw(0.0, 0.5, self.config, 0.02)), 0.01)
        unfiltered = replace(self.config, steering_filter_time_sec=0.0)
        self.assertAlmostEqual(smooth_yaw(0.0, 0.005, unfiltered, 0.02), 0.005)
        for changes in ({'heading_deadband_rad': -0.1}, {'steering_filter_time_sec': 2.0}):
            with self.assertRaises(ValueError):
                replace(self.config, **changes).validate()

    def test_straight_and_direction(self):
        self.assertEqual(line_command(observation(), self.config), (0.05, 0.0, 'tracking'))
        self.assertLess(line_command(observation(error_normalized=0.5), self.config)[1], 0)
        self.assertGreater(line_command(observation(error_normalized=-0.5), self.config)[1], 0)
        config = replace(self.config, steering_sign=1.0)
        self.assertGreater(line_command(observation(error_normalized=0.5), config)[1], 0)

    def test_image_down_direction_and_heading(self):
        config = replace(self.config, image_forward_direction='down')
        self.assertGreater(line_command(observation(error_normalized=0.5), config)[1], 0)
        self.assertLess(line_command(observation(error_normalized=-0.5), config)[1], 0)
        self.assertGreater(line_command(observation(heading_error_rad=0.3), config)[1], 0)
        self.assertEqual(line_command(observation(heading_error_rad=1.5), config)[:2], (0, 0))
        self.assertGreater(line_command(observation(), config)[0], 0)

    def test_body_reference_and_forward_goal(self):
        points = [(450, 100), (400, 200), (300, 300), (200, 420)]
        geometry, reason = tracking_geometry(points, 640, 480, self.config)
        self.assertIsNone(reason)
        self.assertEqual(geometry['near_x'], 200)
        self.assertLess(geometry['target_y'], geometry['reference_y'])
        self.assertLess(geometry['error_normalized'], 0)
        # Far detector error is positive; near measured line is left. Correct
        # the chassis back left rather than steering from that far-only error.
        data = observation(error_normalized=0.4)
        data.update({k: geometry[k] for k in ('error_normalized', 'heading_error_rad')})
        self.assertGreater(line_command(data, self.config)[1], 0)
        shifted, _ = tracking_geometry(points, 640, 480,
                                       replace(self.config, vehicle_reference_x_ratio=0.4))
        self.assertGreater(shifted['error_normalized'], geometry['error_normalized'])

    def test_visible_geometry_limits(self):
        points = [(320, 10), (320, 160)]
        self.assertEqual(tracking_geometry(points, 640, 480, self.config)[1], 'near_line_missing')
        self.assertEqual(tracking_geometry(points, 640, 480,
                         replace(self.config, image_forward_direction='down'))[1], 'no_forward_line')
        down = replace(self.config, image_forward_direction='down', vehicle_reference_y_ratio=-0.05)
        geometry, reason = tracking_geometry(points, 640, 480, down)
        self.assertIsNone(reason)
        self.assertGreater(geometry['target_y'], geometry['reference_y'])
        self.assertEqual(tracking_geometry(points, None, None, self.config)[1], 'invalid_image_size')
        self.assertEqual(tracking_geometry([(math.nan, 100), (320, 420)], 640, 480, self.config)[1],
                         'insufficient_heading_points')

    def test_steering_sign_reverses_both_terms(self):
        status = observation(error_normalized=0.2, heading_error_rad=0.15)
        before = line_command(status, self.config)
        after = line_command(status, replace(self.config, steering_sign=1.0))
        self.assertEqual(before[0], after[0])
        self.assertAlmostEqual(before[1], -after[1])

    def test_horizontal_curve_requires_valid_geometry(self):
        curve = [(160, 420), (230, 350), (320, 270), (410, 170)]
        geometry, reason = tracking_geometry(curve, 640, 480, self.config)
        self.assertIsNone(reason)
        status = observation(orientation='horizontal', curve_geometry_valid=True)
        status.update({key: geometry[key] for key in ('error_normalized', 'heading_error_rad')})
        self.assertEqual(line_command(status, self.config)[2], 'tracking')
        status['curve_geometry_valid'] = False
        self.assertEqual(line_command(status, self.config)[:2], (0, 0))
        status['curve_geometry_valid'] = True
        self.assertEqual(line_command(status, replace(self.config, allow_curved_candidates=False))[:2], (0, 0))
        horizontal = [(150, 400), (300, 400), (450, 400)]
        self.assertEqual(tracking_geometry(horizontal, 640, 480, self.config)[1], 'no_forward_line')
        shallow = [(150, 380), (300, 400), (450, 420)]
        self.assertEqual(tracking_geometry(shallow, 640, 480, self.config)[1], 'insufficient_forward_span')

    def test_motion_arrow_matches_forward_and_turn(self):
        down = ControlConfig()
        straight = motion_preview(0.03, 0.0, down)
        self.assertAlmostEqual(straight[-1][0], 0)
        self.assertGreater(straight[-1][1], 0)
        up = replace(down, image_forward_direction='up', vehicle_reference_y_ratio=1.05)
        self.assertLess(motion_preview(0.03, 0.0, up)[-1][1], 0)
        self.assertGreater(motion_preview(0.03, 0.3, down)[-1][0], 0)
        self.assertLess(motion_preview(0.03, -0.3, down)[-1][0], 0)
        self.assertEqual(motion_preview(0.0, 0.0, down)[-1], (0.0, 0.0))
        # Preserve the corrected effective left/right mapping after relocating
        # the body reference from bottom/up/+1 to top/down/-1.
        status = observation(error_normalized=0.2, heading_error_rad=0.1)
        old = replace(down, image_forward_direction='up', steering_sign=1.0)
        self.assertEqual(line_command(status, old), line_command(status, down))

    def test_top_wheel_margin_allows_current_near_line(self):
        config = ControlConfig()
        visible = [(450, 144), (380, 240), (280, 340), (160, 459)]
        geometry, reason = tracking_geometry(visible, 640, 480, config)
        self.assertIsNone(reason)
        self.assertEqual(geometry['near_y'], 144)
        distant = [(430, 310), (380, 350), (280, 400), (160, 459)]
        self.assertEqual(tracking_geometry(distant, 640, 480, config)[1], 'near_line_missing')

    def test_near_ground_start_excludes_wheel_region(self):
        config = ControlConfig()
        current = [(321, 177), (260, 240), (180, 320), (63, 389)]
        geometry, reason = tracking_geometry(current, 640, 480, config)
        self.assertIsNone(reason)
        self.assertAlmostEqual(geometry['ground_start_y'], 119.75)
        self.assertAlmostEqual(geometry['near_gap_pixels'], 57.25)
        self.assertEqual(tracking_geometry(current, 640, 480,
                         replace(config, near_edge_margin_ratio=0.0))[1], 'near_line_missing')

    def test_visible_line_without_bottom_anchor(self):
        config = replace(self.config, require_bottom_connected=False)
        v, _, reason = line_command(observation(bottom_connected=False), config)
        self.assertGreater(v, 0.0)
        self.assertEqual(reason, 'tracking')
        self.assertEqual(line_command(observation(bottom_connected=False, lost=True), config)[:2], (0, 0))
        self.assertEqual(line_command(observation(bottom_connected=False, valid_row_fraction=0.01), config)[:2], (0, 0))

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
                       {'estimated_speed_factor': 2}, {'linear_acceleration': 0}, {'image_forward_direction': 'left'}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                replace(self.config, **values).validate()

    def test_acceleration_limit(self):
        self.assertAlmostEqual(slew(0.0, 0.1, 0.15, 0.02), 0.003)
        self.assertAlmostEqual(slew(0.01, -0.1, 1.0, 0.02), -0.01)


if __name__ == '__main__':
    unittest.main()
