"""Recorded failure regressions using sampled inputs; no hardware or simulator."""
import math
from pathlib import Path
import sys
import unittest
from dataclasses import replace
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from forward_navigation import ForwardNavigationConfig, ForwardRouteNavigator, VisualCommand
from recorded_route import RouteConfig, StandardRouteFollower
from line_follow_control import ControlConfig, cruise_velocity


class ForwardNavigationTests(unittest.TestCase):
    def navigator(self, **overrides):
        document = dict(points=[[i*.01, 0., 0.] for i in range(81)])
        follower = StandardRouteFollower(document, (0., 0., 0.), RouteConfig())
        return ForwardRouteNavigator(follower, ForwardNavigationConfig(**overrides))

    def visual(self, **overrides):
        return replace(VisualCommand('tracking', .075, 0., 0., 0., True), **overrides)

    def approach(self, navigator):
        # The real tracker deliberately rejects teleporting directly to the end.
        for i in range(7):
            self.assertEqual(navigator.command(i*.1, (i*.1, 0., 0.), .075, self.visual())[2], 'tracking')

    def test_heading_alone_can_correct_and_large_offset_slows(self):
        navigator = self.navigator()
        v, w, state = navigator.command(0., (0., 0., 0.), .075,
            self.visual(heading=.08, w=.1))
        self.assertEqual(state, 'tracking')
        self.assertGreater(w, 0., 'centred but crooked must still correct heading')
        cruise = v
        v, w, _ = navigator.command(.1, (0., 0., 0.), .075,
            self.visual(lateral=.548, w=.2))
        self.assertLess(v, cruise/2., 'logged half-width error must not cruise at .075')
        self.assertLessEqual(abs(navigator.correction), .06)
        self.assertGreater(w, 0.)

    def test_bend_floor_does_not_override_low_cruise_or_large_visual_error(self):
        points = [[.25*math.sin(i*.01/.25), .25*(1-math.cos(i*.01/.25)), i*.01/.25] for i in range(101)]
        def command(speed, lateral=.25, route_config=RouteConfig()):
            follower = StandardRouteFollower(dict(points=points), (0., 0., 0.), route_config)
            navigator = ForwardRouteNavigator(follower)
            result = navigator.command(0., (0., 0., 0.), speed,
                self.visual(lateral=lateral, heading=.4))
            return result, navigator
        (v, w, state), navigator = command(.1)
        self.assertEqual(state, 'tracking')
        self.assertAlmostEqual(v, .05)
        self.assertTrue(navigator.curve_floor_active)
        self.assertGreater(abs(w), .01)
        self.assertLessEqual(command(.03)[0][0], .03)
        self.assertLess(command(.1, lateral=.548)[0][0], .05)
        (v, w, _), _ = command(.1, route_config=replace(RouteConfig(), max_curvature=5., max_lateral_accel=.005))
        self.assertLess(v, .05)
        self.assertLessEqual(abs(v*w), .005+1e-9, 'floor must yield to lateral acceleration limit')

    def test_terminal_braking_starts_before_handoff_and_ignores_curve_floor(self):
        navigator = self.navigator()
        self.approach(navigator)
        v, _, state = navigator.command(.7, (.63, 0., 0.), .1, self.visual())
        self.assertEqual(state, 'tracking')
        self.assertEqual(navigator.phase, 'route')
        self.assertAlmostEqual(v, .02)

    def test_cruise_changes_ramp_in_both_directions_and_stop_remains_immediate(self):
        config = ControlConfig()
        dt = .02
        for sign in (1., -1.):
            v, w = sign*.1, sign*.2
            for target in (.05, .1, .04):
                for _ in range(160):
                    old_v, old_w = v, w
                    v, w = cruise_velocity(v, w, sign*target, sign*target*2., config, dt)
                    rate = (config.cruise_linear_acceleration if target > abs(old_v)
                            else config.cruise_linear_deceleration)
                    self.assertLessEqual(abs(v-old_v), rate*dt+1e-9)
                    self.assertLessEqual(abs(w-old_w), config.angular_acceleration*dt+1e-9)
                    self.assertAlmostEqual(w/v, 2.)
                    self.assertGreater(abs(v), 0.)
                self.assertAlmostEqual(v, sign*target)
            self.assertEqual(cruise_velocity(v, w, 0., 0., config, dt), (0., 0.))
            v, w = cruise_velocity(sign*.1, sign*.2, sign*.05, sign*.1, config, dt, hard_speed_limit=.05)
            self.assertLessEqual(abs(v), .05)
            self.assertAlmostEqual(w/v, 2.)

    def test_terminal_handoff_never_rechecks_a_carrot_behind_robot(self):
        navigator = self.navigator()
        self.approach(navigator)
        # Within the terminal region a small lateral offset can put the final
        # carrot outside the follower's forward sector even with correct yaw.
        result = navigator.command(1., (.79, .025, 0.), .075, self.visual())
        self.assertEqual(navigator.phase, 'visual_finish')
        self.assertEqual(result, (.02, 0., 'tracking'))
        navigator.follower.command = Mock(side_effect=AssertionError('path phase must stay exited'))
        result = navigator.command(1.1, (.80, .02, .2), .075, self.visual(w=.01))
        self.assertEqual(result, (.02, .01, 'tracking'))
        navigator.follower.command.assert_not_called()

    def test_wrong_direction_in_middle_still_faults_and_latches(self):
        navigator = self.navigator()
        result = navigator.command(0., (.1, 0., math.pi), .075, self.visual())
        self.assertEqual(result, (0., 0., 'standard_target_not_ahead'))
        self.assertEqual(navigator.command(1., (.1, 0., 0.), .075, self.visual()), result)

    def test_terminal_requires_fresh_stable_visuals_and_does_not_claim_arrival(self):
        navigator = self.navigator()
        self.approach(navigator)
        result = navigator.command(1., (.66, 0., 0.), .075, VisualCommand('line_lost'))
        self.assertEqual(result, (0., 0., 'line_lost'))
        self.assertEqual(navigator.phase, 'visual_finish')
        self.assertEqual(navigator.command(2., (.66, 0., 0.), .075,
            self.visual(stable=False)), (0., 0., 'reacquiring'))
        self.assertEqual(navigator.command(3., (.80, 0., 0.), .075, self.visual())[2], 'tracking')

    def test_terminal_time_distance_endpoint_and_corridor_fences(self):
        cases = [(32., (.66, 0., 0.), 'standard_finish_timeout'),
                 (2., (.86, 0., 0.), 'standard_finish_search_limit'),
                 (2., (.66, .151, 0.), 'standard_finish_deviation'),
                 (2., (.66, 0., math.pi), 'standard_finish_heading_out_of_range')]
        for now, pose, fault in cases:
            with self.subTest(fault=fault):
                navigator = self.navigator()
                self.approach(navigator)
                navigator.command(1., (.66, 0., 0.), .075, self.visual())
                result = navigator.command(now, pose, .075, self.visual())
                self.assertEqual(result, (0., 0., fault))
        navigator = self.navigator()
        self.approach(navigator)
        navigator.command(1., (.66, 0., 0.), .075, self.visual())
        navigator.command(2., (.76, 0., 0.), .075, self.visual())
        self.assertEqual(navigator.command(3., (.66, 0., 0.), .075, self.visual())[2],
                         'standard_finish_search_limit', 'back-and-forth consumes total travel')

    def test_loss_continuation_uses_both_actual_distance_and_elapsed_time(self):
        for kind in ('time', 'distance'):
            with self.subTest(kind=kind):
                navigator = self.navigator()
                missing = VisualCommand('line_lost')
                v, _, state = navigator.command(0., (0., 0., 0.), .075, missing)
                self.assertEqual(state, 'tracking')
                self.assertTrue(0. < v <= .05)
                if kind == 'time':
                    result = navigator.command(5., (0., 0., 0.), .075, missing)
                    self.assertEqual(result[2], 'standard_visual_loss_timeout')
                else:
                    result = navigator.command(1., (.31, 0., 0.), .075, missing)
                    self.assertEqual(result[2], 'standard_visual_loss_distance_limit')
                self.assertEqual(result[:2], (0., 0.))
                self.assertEqual(navigator.command(6., (.31, 0., 0.), .075, self.visual()), result)

    def test_only_stable_recovery_ends_loss_budget(self):
        navigator = self.navigator()
        navigator.command(0., (0., 0., 0.), .075, VisualCommand('line_lost'))
        navigator.command(1., (.02, 0., 0.), .075, self.visual(stable=False))
        self.assertIsNotNone(navigator.visual_loss)
        navigator.command(2., (.04, 0., 0.), .075, self.visual())
        self.assertIsNone(navigator.visual_loss)

    def test_bad_streams_never_use_line_loss_allowance(self):
        for reason in ('camera_timeout', 'processing_over_budget', 'malformed_status'):
            navigator = self.navigator()
            self.assertEqual(navigator.command(0., (0., 0., 0.), .075,
                VisualCommand(reason)), (0., 0., reason))
            self.assertIsNone(navigator.visual_loss)

    def test_bad_numbers_and_large_visual_deviation_stop(self):
        for pose, speed, visual, reason in (
            ((math.nan, 0., 0.), .075, self.visual(), 'standard_invalid_pose_or_speed'),
            ((0., 0., 0.), -.1, self.visual(), 'standard_invalid_pose_or_speed'),
            ((0., 0., 0.), .075, self.visual(w=math.nan), 'standard_invalid_visual'),
            ((0., 0., 0.), .075, self.visual(lateral=.81), 'standard_visual_deviation')):
            self.assertEqual(self.navigator().command(0., pose, speed, visual), (0., 0., reason))
        for bad in ({'assist_gain': math.nan}, {'assist_max_yaw': .5}, {'loss_timeout': -1.}):
            with self.assertRaises(ValueError):
                ForwardNavigationConfig(**bad).validate()


if __name__ == '__main__':
    unittest.main()
