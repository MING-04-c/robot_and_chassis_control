"""ROS-independent forward route/visual handoff state machine.

Design references (ideas, not a Nav2 runtime dependency):
Nav2 regulated_pure_pursuit_controller: metric path tracking and speed regulation.
Nav2 rotation_shim_controller: align before handing over to the path controller.
Nav2 simple_goal_checker: stateful final approach, separate from path lookahead.
The real stop bar, not the path endpoint, remains the arrival authority.
"""
from dataclasses import dataclass
import bisect
import math


@dataclass(frozen=True)
class ForwardNavigationConfig:
    finish_distance: float = .15
    finish_extra_distance: float = .05
    finish_timeout: float = 30.
    finish_speed: float = .02
    loss_timeout: float = 5.
    loss_distance: float = .30
    loss_speed: float = .05
    assist_gain: float = .5
    assist_max_yaw: float = .06
    center_slowdown: float = .12
    heading_slowdown: float = .10
    min_visual_speed_factor: float = .4
    curve_min_speed: float = .05
    approach_deceleration: float = .04

    def validate(self):
        bounds = {'finish_distance': (.05, .3), 'finish_extra_distance': (0., .1),
                  'finish_timeout': (1., 60.), 'finish_speed': (.005, .03),
                  'loss_timeout': (0., 10.), 'loss_distance': (0., .5),
                  'loss_speed': (.01, .05), 'assist_gain': (0., 1.),
                  'assist_max_yaw': (0., .10), 'center_slowdown': (.05, .3),
                  'heading_slowdown': (.03, .3), 'min_visual_speed_factor': (.2, 1.),
                  'curve_min_speed': (0., .10), 'approach_deceleration': (.01, .10)}
        for key, (low, high) in bounds.items():
            value = getattr(self, key)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not low <= value <= high):
                raise ValueError(f'forward navigation {key} must be {low}..{high}')


@dataclass(frozen=True)
class VisualCommand:
    reason: str
    v: float = 0.
    w: float = 0.
    lateral: float = 0.  # image ratio, never metres without ground calibration
    heading: float = 0.  # near-line image tangent, not global odometry heading
    stable: bool = False

    @property
    def usable(self):
        return self.reason == 'tracking' and all(math.isfinite(x) for x in (
            self.v, self.w, self.lateral, self.heading))


class TerminalApproachGuard:
    """Independent odometry fence after the irreversible visual handoff."""
    def __init__(self, tracker, now, pose, remaining, config):
        self.config = config
        self.max_tracking_error = tracker.config.max_tracking_error
        first = max(0, bisect.bisect_right(tracker.arc,
                    max(0., tracker.arc[-1]-remaining-.10))-1)
        self.corridor = tracker.points[first:]
        self.goal = tracker.points[-1]
        before = tracker.at(max(0., tracker.arc[-1]-.10))
        dx, dy = self.goal[0]-before[0], self.goal[1]-before[1]
        length = math.hypot(dx, dy)
        if length < .01:
            raise ValueError('standard_finish_invalid_tangent')
        self.tangent = dx/length, dy/length
        self.snapshot = dict(started=now, previous=list(pose[:2]), travel=0.,
                             limit=remaining+config.finish_extra_distance,
                             remaining_at_handoff_m=remaining)

    def check(self, now, pose):
        data = self.snapshot
        data['travel'] += math.dist(pose[:2], data['previous'])
        data['previous'] = list(pose[:2])
        dx, dy = pose[0]-self.goal[0], pose[1]-self.goal[1]
        tx, ty = self.tangent
        beyond = dx*tx+dy*ty
        lateral = abs(-dx*ty+dy*tx)
        heading = math.atan2(math.sin(pose[2]-math.atan2(ty, tx)),
                             math.cos(pose[2]-math.atan2(ty, tx)))
        if beyond <= 0.:
            distances = []
            for a, b in zip(self.corridor, self.corridor[1:]):
                sx, sy = b[0]-a[0], b[1]-a[1]
                t = max(0., min(1., ((pose[0]-a[0])*sx+(pose[1]-a[1])*sy)/(sx*sx+sy*sy)))
                distances.append(math.hypot(pose[0]-a[0]-t*sx, pose[1]-a[1]-t*sy))
            lateral = min(distances, default=math.hypot(dx, dy))
        data.update(lateral_error_m=lateral, beyond_goal_m=max(0., beyond), heading_error_rad=heading)
        if now-data['started'] > self.config.finish_timeout:
            return 'standard_finish_timeout'
        if (data['travel'] > data['limit']
                or beyond > self.config.finish_extra_distance+1e-6):
            return 'standard_finish_search_limit'
        if lateral > self.max_tracking_error:
            return 'standard_finish_deviation'
        if abs(heading) > .9:
            return 'standard_finish_heading_out_of_range'
        return None


class ForwardRouteNavigator:
    """One owner per phase: route -> visual_finish; faults latch until reset.

    Call only with fresh odometry, camera stream and chassis permission. The ROS
    adapter owns those interlocks; this class owns path/vision arbitration and
    geometric/time fences. It never declares arrival or enables another task.
    """
    LOSS_REASONS = frozenset({'line_lost', 'empty_centerline', 'insufficient_heading_points',
        'no_forward_line', 'insufficient_forward_span', 'insufficient_rows',
        'near_line_missing', 'horizontal_or_unknown_line', 'tracking'})

    def __init__(self, follower, config=ForwardNavigationConfig()):
        config.validate()
        self.follower, self.config = follower, config
        self.phase = 'route'
        self.fault = None
        self.finish_guard = None
        self.visual_loss = None
        self.correction = 0.
        self.speed_factor = 1.
        self.curve_floor_active = False
        self.target_speed = 0.
        self.speed_limit_reason = 'route'

    @property
    def finish(self):
        return self.finish_guard.snapshot if self.finish_guard else None

    @property
    def diagnostics(self):
        return dict(phase=self.phase, visual_speed_factor=self.speed_factor,
                    visual_correction_radps=self.correction, visual_loss=self.visual_loss,
                    finish=self.finish, fault=self.fault, curve_min_speed_mps=self.config.curve_min_speed,
                    curve_floor_active=self.curve_floor_active, target_speed_mps=self.target_speed,
                    speed_limit_reason=self.speed_limit_reason)

    def fail(self, reason):
        self.phase, self.fault = 'fault', reason
        self.target_speed = 0.
        self.speed_limit_reason = reason
        return 0., 0., reason

    def _approach_speed(self, v, w, remaining):
        # Start braking before the visual handoff. The margin covers a camera
        # frame of travel; the slower planned deceleration leaves control reserve.
        distance = max(0., remaining-self.config.finish_distance-.02)
        limit = math.sqrt(self.config.finish_speed**2+2*self.config.approach_deceleration*distance)
        scale = min(1., limit/v) if v > 0. else 0.
        if scale < 1.:
            self.speed_limit_reason = 'terminal_braking'
            self.curve_floor_active = False
        self.target_speed = v*scale
        return v*scale, w*scale, 'tracking'

    def _finish_command(self, now, pose, visual):
        self.speed_limit_reason = 'visual_finish'
        fault = self.finish_guard.check(now, pose)
        if fault:
            return self.fail(fault)
        if not visual.usable or not visual.stable:
            return 0., 0., visual.reason if not visual.usable else 'reacquiring'
        # The gate in the adapter still enforces averaging and actual stop.
        self.target_speed = max(0., min(visual.v, self.config.finish_speed))
        return self.target_speed, visual.w, 'tracking'

    def command(self, now, pose, speed, visual):
        self.correction, self.speed_factor = 0., 1.
        self.curve_floor_active = False
        self.target_speed = 0.
        self.speed_limit_reason = 'route'
        if self.fault:
            return 0., 0., self.fault
        if len(pose) != 3 or not all(isinstance(x, (int, float)) and not isinstance(x, bool)
                                    and math.isfinite(x) for x in (*pose, now, speed)) or speed < 0.:
            return self.fail('standard_invalid_pose_or_speed')
        if visual.reason == 'tracking' and not visual.usable:
            return self.fail('standard_invalid_visual')
        if visual.usable and abs(visual.lateral) > .80:
            return self.fail('standard_visual_deviation')
        if self.phase == 'visual_finish':
            # No further path lookahead calls: a carrot may legitimately be
            # beside/behind the robot while vision completes the real stop.
            return self._finish_command(now, pose, visual)
        v, w, state = self.follower.command(pose, speed)
        data = self.follower.diagnostics
        remaining = data.get('remaining_m', math.inf)
        valid_states = ('standard_route_tracking', 'standard_route_complete', 'standard_target_not_ahead')
        if state not in valid_states:
            return self.fail(state)
        if remaining <= self.config.finish_distance:
            try:
                self.finish_guard = TerminalApproachGuard(
                    self.follower.tracker, now, pose, remaining, self.config)
            except ValueError as error:
                return self.fail(str(error))
            self.phase = 'visual_finish'
            self.visual_loss = None
            return self._finish_command(now, pose, visual)
        if state == 'standard_target_not_ahead':
            return self.fail(state)
        if visual.usable and not visual.stable and self.visual_loss is None:
            return 0., 0., 'reacquiring'
        if visual.usable and visual.stable:
            self.visual_loss = None
            # Correct heading even when the centre happens to be in tolerance.
            if abs(visual.lateral) > .02 or abs(visual.heading) > .035:
                self.correction = max(-self.config.assist_max_yaw,
                    min(self.config.assist_max_yaw, self.config.assist_gain*visual.w))
            w += self.correction
            # A reliable image showing a deviation must slow the metric tracker,
            # rather than maintaining cruise while its weak correction saturates.
            self.speed_factor = max(self.config.min_visual_speed_factor, min(1.,
                self.config.center_slowdown/max(abs(visual.lateral), 1e-9),
                self.config.heading_slowdown/max(abs(visual.heading), 1e-9)))
            target = v*self.speed_factor
            if self.speed_factor < 1.:
                self.speed_limit_reason = 'visual_slowdown'
            curvature = w/v if v > 1e-9 else 0.
            # The floor is a normal bend target, never a motion permission or
            # an override of physical limits / severe visual disagreement.
            if abs(curvature) >= .5 and abs(visual.lateral) <= .30 and abs(visual.heading) <= .60:
                floor = min(v, self.config.curve_min_speed)
                self.curve_floor_active = target < floor
                target = max(target, floor)
                if self.curve_floor_active:
                    self.speed_limit_reason = 'curve_floor'
            # Visual assistance may have increased curvature after the follower
            # applied its own lateral acceleration constraint.
            if abs(curvature) > 1e-9:
                lateral_limit = math.sqrt(self.follower.tracker.config.max_lateral_accel/abs(curvature))
                if target > lateral_limit:
                    target = lateral_limit
                    self.curve_floor_active = False
                    self.speed_limit_reason = 'lateral_acceleration'
            return self._approach_speed(target, curvature*target, remaining)
        if visual.reason not in self.LOSS_REASONS:
            return 0., 0., visual.reason
        if self.visual_loss is None:
            self.visual_loss = dict(started=now, previous=list(pose[:2]), travel=0., reason=visual.reason)
        loss = self.visual_loss
        loss['travel'] += math.dist(pose[:2], loss['previous'])
        loss['previous'] = list(pose[:2])
        if now-loss['started'] >= self.config.loss_timeout:
            return self.fail('standard_visual_loss_timeout')
        if loss['travel'] >= self.config.loss_distance:
            return self.fail('standard_visual_loss_distance_limit')
        if v > self.config.loss_speed:
            w *= self.config.loss_speed/v
            v = self.config.loss_speed
        self.speed_limit_reason = 'visual_loss'
        return self._approach_speed(v, w, remaining)
