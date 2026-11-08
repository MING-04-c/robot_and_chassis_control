"""Filtered visual alignment to a departure bar before route recording/replay."""

import math
from statistics import median

class StartLineAligner:
    """Stop, observe a stable bar, execute one odometry step, then observe again.

    Pixel error is intentionally kept dimensionless unless a measured GroundMapping
    is supplied.  This follows the image-based visual-servo pattern used in ROS
    robotics: track a stable feature over fresh frames, use bounded corrections,
    and only declare alignment after fresh in-tolerance observations within
    a bounded stationary window.
    """

    POSITION_STEP_TOLERANCE_M = .001
    ANGLE_STEP_TOLERANCE_RAD = .003
    OBSERVATION_WINDOW_SEC = 1.5

    def __init__(self, config, mapping=None, max_distance_m=0.30,
                 metric_tolerance_m=0.02, lateral_tolerance_m=0.02,
                 confirmation_frames=5, filter_frames=5, decision_frames=None):
        if not 0.05 <= max_distance_m <= 1.0:
            raise ValueError('start alignment max distance must be 0.05..1.0m')
        if not 0.005 <= metric_tolerance_m <= 0.1:
            raise ValueError('start alignment metric tolerance must be 0.005..0.1m')
        if not 0.005 <= lateral_tolerance_m <= 0.1:
            raise ValueError('start alignment lateral tolerance must be 0.005..0.1m')
        if not 3 <= confirmation_frames <= 15 or not 3 <= filter_frames <= 9:
            raise ValueError('start alignment frame counts out of range')
        decision_frames = filter_frames if decision_frames is None else decision_frames
        if not 3 <= decision_frames <= filter_frames:
            raise ValueError('decision frames must be 3..filter_frames')
        self.decision_frames = int(decision_frames)
        self.config = config
        self.mapping = mapping
        self.max_distance_m = float(max_distance_m)
        self.metric_tolerance_m = float(metric_tolerance_m)
        self.lateral_tolerance_m = float(lateral_tolerance_m)
        self.confirmation_frames = int(confirmation_frames)
        self.filter_frames = int(filter_frames)
        self.reset()

    def reset(self, now=None, pose=None):
        self.started_at = now
        self.start_pose = tuple(pose[:3]) if pose is not None else None
        self.previous_pose = tuple(pose[:3]) if pose is not None else None
        self.travelled_m = 0.0
        self.reverse_travelled_m = 0.0
        self.last_command_v = 0.0
        self.phase = 'settling'
        self.recheck_settle = False
        self.stopped_since = None
        self.accept_after = None
        self.step_pose = None
        self.step_started_at = None
        self.step_timeout_sec = 4.
        self.step_limit_m = self.config.start_alignment_step_distance
        self.step_distance = 0.0
        self.step_lateral_m = 0.0
        self.step_angle = 0.0
        self.step_axis = None
        self.step_remaining_m = 0.0
        self.step_remaining_rad = 0.0
        self.step_actual_m = self.step_actual_rad = 0.0
        self.step_v = self.step_vy = self.step_w = 0.0
        self.step_count = 0
        self.image_response_per_m = None
        self.lateral_response_per_m = None
        self.response_samples = {'longitudinal': [], 'lateral': []}
        self.response_reasons = {'longitudinal': 'not_observed', 'lateral': 'not_observed'}
        self.response_reason = 'not_observed'
        self.estimated_remaining_m = None
        self.estimated_correction_m = None
        self.distance_estimate_source = None
        self.response_reference = None
        self.inputs_ready = True
        self.marker_pending = False
        self.resume_observation_after = float('-inf')
        self.last_observation_valid = False
        self.last_tick_at = None
        self.input_block_reason = None
        self.is_stopped = False
        self.last_stamp = None
        self.last_seen_at = None
        self.marker = None
        self.width = self.height = None
        self.samples = []
        self.sample_times = []
        self.position_error = None
        self.position_error_m = None
        self.lateral_error = None
        self.heading_error = None
        self.stable_frames = 0
        self.visual_ready = False
        self.state = 'start_alignment_waiting_for_marker'
        self.fault = None

    @property
    def metric(self):
        return self.mapping is not None and self.position_error_m is not None

    def observe(self, candidates, width, height, stamp, now, source_time=None):
        """Use only frames captured after settling, never frames from a move.

        ``source_time`` is capture time converted to the monotonic clock by the
        ROS adapter. Tests without an asynchronous image pipeline may omit it.
        """
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return False
        self.last_stamp = stamp
        source_time = now if source_time is None else source_time
        if (self.phase != 'observing' or not (self.inputs_ready or self.marker_pending)
                or self.accept_after is None
                or source_time < max(self.accept_after, self.resume_observation_after)
                or source_time > now + .1 or now-source_time > .5):
            return False
        self.last_observation_valid = False
        self._prune_observations(now)
        if width <= 1 or height <= 1:
            return False

        target_y = self.config.stop_marker_target_ratio * (height - 1)
        valid = [m for m in candidates if self._valid_marker(m, width, height)]
        if not valid:
            # Missing detections add no evidence; retain only the bounded,
            # stationary window. Completion still requires a new valid frame.
            self.visual_ready = False
            return False
        if self.marker is None or self.last_seen_at is None or source_time-self.last_seen_at > .75:
            marker = min(valid, key=lambda m: abs(float(m['y']) - target_y))
            self._clear_observations()
        else:
            prev = self.marker
            nearby = [m for m in valid if abs(float(m['y']) - prev['y']) <= .12*height
                      and abs(float(m['x']) - prev['x']) <= .20*width]
            if not nearby:
                self.stable_frames = 0
                self.visual_ready = False
                return False
            marker = min(nearby, key=lambda m: math.hypot(
                (float(m['x'])-prev['x'])/width, (float(m['y'])-prev['y'])/height))

        x, y = float(marker['x']), float(marker['y'])
        forward_sign = 1.0 if self.config.image_forward_direction == 'down' else -1.0
        error_ratio = forward_sign * (y/(height-1) - self.config.stop_marker_target_ratio)
        error_m = None
        if self.mapping is not None:
            if self.mapping.size != (width, height):
                self.fault = 'start_alignment_failed_ground_mapping_resolution_mismatch'
                self.state = self.fault
                return False
            # Move the detected centerline/bar intersection to the same calibrated
            # ground point represented by the selected target row.
            slope = float(marker.get('main_slope', 0.0))
            target_x = x + slope*(target_y-y)
            try:
                tangent_half_span = max(8., .08*(height-1))
                tangent_y0 = max(0., y-tangent_half_span)
                tangent_y1 = min(float(height-1), y+tangent_half_span)
                tangent_points = [[x+slope*(tangent_y-y), tangent_y]
                                  for tangent_y in (tangent_y0, tangent_y1)]
                projected = self.mapping.project([[x, y], [target_x, target_y], *tangent_points])
            except (TypeError, ValueError, IndexError):
                self.fault = 'start_alignment_failed_ground_mapping_invalid'
                self.state = self.fault
                return False
            error_m = float(projected[0, 0] - projected[1, 0])
            if not math.isfinite(error_m):
                self.fault = 'start_alignment_failed_ground_mapping_invalid'
                self.state = self.fault
                return False
            tangent_dx = float(projected[3, 0]-projected[2, 0])
            tangent_dy = float(projected[3, 1]-projected[2, 1])
            if tangent_dx < 0.:
                tangent_dx, tangent_dy = -tangent_dx, -tangent_dy
            metric_heading = math.atan2(tangent_dy, tangent_dx)
            metric_lateral = float(projected[1, 1])
            if not all(math.isfinite(value) for value in (metric_heading, metric_lateral)):
                self.fault = 'start_alignment_failed_ground_mapping_invalid'
                self.state = self.fault
                return False

        target_x = x + float(marker.get('main_slope', 0.0))*(target_y-y)
        lateral = (metric_lateral if self.mapping is not None else
                   2.0*(target_x/(width-1)-self.config.vehicle_reference_x_ratio))
        heading = (metric_heading if self.mapping is not None else
                   math.atan(forward_sign*float(marker.get('main_slope', 0.0))))
        self.samples.append((error_ratio, error_m, lateral, heading))
        self.samples = self.samples[-self.filter_frames:]
        self.sample_times.append(source_time)
        self.sample_times = self.sample_times[-self.filter_frames:]
        self._filter_observations(self.decision_frames)
        self.marker = dict(marker)
        self.width, self.height = width, height
        self.last_seen_at = source_time
        self.last_observation_valid = True

        longitudinal_ok = (abs(self.position_error_m) <= self.metric_tolerance_m
                           if self.position_error_m is not None else
                           abs(self.position_error) <= self.config.start_alignment_tolerance_ratio)
        lateral_tolerance = (self.lateral_tolerance_m if self.mapping is not None else
                             self.config.start_alignment_lateral_tolerance_ratio)
        lateral_ok = abs(self.lateral_error) <= lateral_tolerance
        heading_ok = abs(self.heading_error) <= self.config.start_alignment_heading_tolerance_rad
        raw_longitudinal_ok = (abs(error_m) <= self.metric_tolerance_m if error_m is not None
                               else abs(error_ratio) <= self.config.start_alignment_tolerance_ratio)
        raw_ok = (raw_longitudinal_ok and abs(lateral) <= lateral_tolerance
                  and abs(heading) <= self.config.start_alignment_heading_tolerance_rad)
        self.stable_frames = self.stable_frames + 1 if (
            longitudinal_ok and lateral_ok and heading_ok and raw_ok) else 0
        self.visual_ready = (self.window_stable() and
                             self.stable_frames >= self.confirmation_frames)
        if self.visual_ready:
            self.state = 'start_alignment_waiting_for_stop'
        elif longitudinal_ok:
            self.state = 'start_alignment_visual_settling'
        else:
            self.state = 'start_alignment_visual_tracking'
        return True

    def _filter_observations(self, frames):
        samples = self.samples[-frames:]
        self.position_error = median(row[0] for row in samples)
        metric_values = [row[1] for row in samples if row[1] is not None]
        self.position_error_m = median(metric_values) if len(metric_values) == len(samples) else None
        self.lateral_error = median(row[2] for row in samples)
        self.heading_error = median(row[3] for row in samples)

    def _clear_observations(self):
        self.samples.clear()
        self.sample_times.clear()
        self.stable_frames = 0
        self.visual_ready = False

    def _prune_observations(self, now):
        removed = False
        while self.sample_times and now-self.sample_times[0] > self.OBSERVATION_WINDOW_SEC:
            self.sample_times.pop(0)
            self.samples.pop(0)
            removed = True
        if removed or not self.samples:
            self.stable_frames = min(self.stable_frames, len(self.samples))
            self.visual_ready = False

    def window_stable(self, frames=None):
        frames = self.filter_frames if frames is None else frames
        if len(self.samples) < frames:
            return False
        limits = (self.config.start_alignment_tolerance_ratio,
                  self.metric_tolerance_m,
                  self.lateral_tolerance_m if self.mapping is not None else
                  self.config.start_alignment_lateral_tolerance_ratio,
                  self.config.start_alignment_heading_tolerance_rad)
        for index, limit in enumerate(limits):
            values = [row[index] for row in self.samples[-frames:] if row[index] is not None]
            if values and max(values)-min(values) > limit:
                return False
        return True

    def observation_stability(self):
        """Explain the decision window without changing its acceptance rules."""
        limits = (self.config.start_alignment_tolerance_ratio, self.metric_tolerance_m,
                  self.lateral_tolerance_m if self.mapping is not None else
                  self.config.start_alignment_lateral_tolerance_ratio,
                  self.config.start_alignment_heading_tolerance_rad)
        axes = {}
        for index, (name, limit) in enumerate(zip(
                ('longitudinal_ratio', 'longitudinal_m', 'lateral', 'heading_rad'), limits)):
            values = [row[index] for row in self.samples[-self.decision_frames:]
                      if row[index] is not None]
            spread = max(values)-min(values) if values else None
            axes[name] = dict(spread=spread, limit=limit,
                              within_limit=spread is None or spread <= limit)
        return dict(sample_count=len(self.samples), required_samples=self.decision_frames,
                    stable=self.window_stable(self.decision_frames), axes=axes)

    def _settle(self, recheck=False):
        self.recheck_settle = recheck
        self.phase = 'settling'
        self.stopped_since = self.accept_after = None
        self.step_v = self.step_vy = self.step_w = 0.0
        self.last_observation_valid = False
        self._clear_observations()
        self.state = 'start_alignment_waiting_for_stop'

    def _update_image_response(self, pose):
        reference, self.response_reference = self.response_reference, None
        if self.metric or reference is None:
            return
        old_pose, old_error, axis, image_size, old_spread = reference
        dx, dy = pose[0]-old_pose[0], pose[1]-old_pose[1]
        distance = (dx*math.cos(old_pose[2])+dy*math.sin(old_pose[2]) if axis == 'longitudinal'
                    else -dx*math.sin(old_pose[2])+dy*math.cos(old_pose[2]))
        angle = math.atan2(math.sin(pose[2]-old_pose[2]), math.cos(pose[2]-old_pose[2]))
        cross_distance = (-dx*math.sin(old_pose[2])+dy*math.cos(old_pose[2])
                          if axis == 'longitudinal' else
                          dx*math.cos(old_pose[2])+dy*math.sin(old_pose[2]))
        error = self.position_error if axis == 'longitudinal' else self.image_body_sign*self.lateral_error
        attr = 'image_response_per_m' if axis == 'longitudinal' else 'lateral_response_per_m'
        history = self.response_samples[axis]
        # Compare two stopped windows, not a commanded distance or moving image.
        # Require a change above half a pixel and the observed window variation.
        pixel = 1./(self.height-1) if axis == 'longitudinal' else 2./(self.width-1)
        noise = max(.5*pixel, old_spread+self._error_spread(axis))
        reason = None
        if image_size != (self.width, self.height):
            reason = 'image_size_changed'
        elif abs(distance) < .002 or abs(angle) > .02 or abs(cross_distance) > max(.002, .25*abs(distance)):
            reason = 'motion_contaminated'
        elif abs(old_error-error) <= noise:
            reason = 'image_change_below_noise'
        else:
            response = (old_error-error)/distance
            if not .1 <= response <= 100.:
                reason = 'wrong_or_invalid_response'
            elif history and not .5*median(history) <= response <= 2.*median(history):
                reason = 'inconsistent_response'
        if reason:
            # Do not keep trusting a stale slope after contrary evidence. The
            # next bounded probe can acquire a new local scale independently.
            history.clear()
            setattr(self, attr, None)
            self.response_reason = reason
            self.response_reasons[axis] = reason
            return
        history.append(response)
        del history[:-3]
        setattr(self, attr, median(history))
        self.response_reason = 'accepted'
        self.response_reasons[axis] = 'accepted'

    def _error_spread(self, axis):
        values = [row[0 if axis == 'longitudinal' else 2] for row in self.samples[-self.decision_frames:]]
        return max(values)-min(values) if values else 0.

    @property
    def image_body_sign(self):
        return (self.config.steering_sign if self.config.image_forward_direction == 'up'
                else -self.config.steering_sign)

    def _translation_distance(self, error, response, tolerance):
        self.response_reason = self.response_reasons[self.step_axis]
        self.estimated_remaining_m = self.estimated_correction_m = None
        if self.metric or response is not None:
            # The largest of the recent consistent slopes shortens the step if
            # the camera response strengthens near the target (perspective).
            scale = 1. if self.metric else max(self.response_samples[self.step_axis] or [response])
            self.estimated_remaining_m = error/scale
            # Aim inside the tolerance band, rather than chasing exact zero or
            # requesting a nearly zero correction just outside the boundary.
            self.estimated_correction_m = math.copysign((abs(error)-.5*tolerance)/scale, error)
            self.distance_estimate_source = ('ground_mapping' if self.metric else
                'local_response_confirmed' if len(self.response_samples[self.step_axis]) >= 2
                else 'local_response_tentative')
            distance = .8*abs(self.estimated_correction_m)
        else:
            self.distance_estimate_source = 'probe_no_reliable_scale'
            distance = (.002 if self.step_axis == 'longitudinal' and abs(error) <= 2.*tolerance
                        else .005)
        self.step_limit_m = self.config.start_alignment_step_distance
        reliable = self.distance_estimate_source in ('ground_mapping', 'local_response_confirmed')
        if (reliable and abs(error) > 4.*tolerance and
                abs(self.estimated_remaining_m) > 2.*self.step_limit_m + 1e-6):
            self.step_limit_m = self.config.start_alignment_coarse_step_distance
        # Do not plan beyond the remaining cumulative motion budget.
        self.step_limit_m = min(self.step_limit_m, max(0., self.max_distance_m-self.travelled_m))
        if self.step_axis == 'longitudinal' and error < 0.:
            self.step_limit_m = min(self.step_limit_m, max(0.,
                self.config.start_alignment_max_reverse_distance-self.reverse_travelled_m))
        # A lateral correction must leave room for the nominal braking distance
        # at the minimum target speed, plus the odometry arrival tolerance.
        # This is a planning heuristic, not a measured stopping-distance claim.
        minimum = (.001 + self.config.start_alignment_min_lateral_speed**2 /
                   (2.*self.config.linear_acceleration) if self.step_axis == 'lateral' else .002)
        return math.copysign(min(self.step_limit_m, max(minimum, distance)), error)

    def _plan_step(self, now, pose):
        self._update_image_response(pose)
        error = self.position_error_m if self.metric else self.position_error
        tolerance = self.metric_tolerance_m if self.metric else self.config.start_alignment_tolerance_ratio
        lateral_tolerance = (self.lateral_tolerance_m if self.metric else
                             self.config.start_alignment_lateral_tolerance_ratio)
        if (abs(error) <= tolerance and abs(self.lateral_error) <= lateral_tolerance
                and abs(self.heading_error) <= self.config.start_alignment_heading_tolerance_rad):
            self.state = 'start_alignment_waiting_for_stable_image'
            return  # Still accumulating independent confirmation frames.
        # For this holonomic chassis, lateral and heading errors must never be
        # summed into one yaw command: they can cancel while the robot is off line.
        distance = lateral = angle = v = vy = w = 0.
        sign = 1. if self.metric else self.image_body_sign
        if abs(self.heading_error) > self.config.start_alignment_heading_tolerance_rad:
            self.step_axis = 'heading'
            self.estimated_remaining_m = self.estimated_correction_m = None
            self.distance_estimate_source = 'heading_correction'
            self.response_reason = 'heading_correction'
            self.image_response_per_m = self.lateral_response_per_m = None
            self.response_reasons = dict.fromkeys(self.response_reasons, 'heading_changed')
            for history in self.response_samples.values():
                history.clear()
            angle_limit = (self.config.start_alignment_coarse_max_angle
                           if abs(self.heading_error) > 4.*self.config.start_alignment_heading_tolerance_rad
                           else .04)
            angle = math.copysign(min(angle_limit, max(.01, .6*abs(self.heading_error))), sign*self.heading_error)
            speed = max(self.config.start_alignment_min_angular_speed, .6*abs(self.heading_error))
            w = math.copysign(min(self.config.start_alignment_max_angular_speed, speed), angle)
        elif abs(self.lateral_error) > lateral_tolerance:
            self.step_axis = 'lateral'
            lateral = self._translation_distance(sign*self.lateral_error, self.lateral_response_per_m,
                                                 lateral_tolerance)
            vy = math.copysign(self.config.start_alignment_lateral_speed, lateral)
        else:
            self.step_axis = 'longitudinal'
            if error < -tolerance and not self.config.start_alignment_reverse_correction:
                self.fault = self.state = 'start_alignment_failed_correction_unavailable'
                return
            distance = self._translation_distance(error, self.image_response_per_m, tolerance)
            speed = min(self.config.start_alignment_speed, self.config.forward_speed if distance > 0.
                        else self.config.reverse_max_speed)
            if speed < self.config.start_alignment_min_linear_speed:
                self.fault = self.state = 'start_alignment_failed_speed_below_motion_threshold'
                return
            v = math.copysign(speed, distance)
        if self.step_axis != 'heading' and abs(distance or lateral) <= self.POSITION_STEP_TOLERANCE_M:
            self.fault = self.state = ('start_alignment_failed_reverse_limit' if self.step_axis == 'longitudinal' and error < 0. else
                                       'start_alignment_failed_distance_limit')
            return
        self.step_pose, self.step_started_at = tuple(pose[:3]), now
        self.step_actual_m = self.step_actual_rad = 0.0
        self.step_distance, self.step_lateral_m, self.step_angle = distance, lateral, angle
        self.step_remaining_m, self.step_remaining_rad = abs(distance or lateral), abs(angle)
        self.step_v, self.step_vy, self.step_w = v, vy, w
        if self.step_axis != 'heading':
            response_error = self.position_error if distance else sign*self.lateral_error
            self.response_reference = (self.step_pose, response_error, self.step_axis,
                                       (self.width, self.height), self._error_spread(self.step_axis))
        duration = abs(distance/v) if v else abs(lateral/vy) if vy else abs(angle/w)
        self.step_timeout_sec = max(4., min(8., 2.+1.5*duration))
        self.step_count += 1
        self.phase = 'moving'
        self._clear_observations()
        self.state = ('start_alignment_moving_forward' if distance > 0. else
                      'start_alignment_moving_reverse' if distance < 0. else
                      'start_alignment_moving_lateral' if lateral else 'start_alignment_moving_yaw')

    def velocity(self):
        """Distance-based braking profile with per-axis minimum target speeds.

        Following Nav2 DriveOnHeading's ordering, arrival always wins over the
        minimum speed. The adapter applies acceleration limits before publishing.
        No integral term accumulates while the chassis is not making progress.
        """
        if self.phase != 'moving' or self.fault:
            return 0., 0., 0.
        def bounded(speed, remaining, floor, acceleration, tolerance):
            if not speed or remaining <= tolerance:
                return 0.
            braking_speed = math.sqrt(2.*acceleration*(remaining-tolerance))
            return math.copysign(min(abs(speed), max(floor, braking_speed)), speed)
        return (bounded(self.step_v, self.step_remaining_m, self.config.start_alignment_min_linear_speed,
                        self.config.linear_acceleration, self.POSITION_STEP_TOLERANCE_M),
                bounded(self.step_vy, self.step_remaining_m, self.config.start_alignment_min_lateral_speed,
                        self.config.linear_acceleration, self.POSITION_STEP_TOLERANCE_M),
                bounded(self.step_w, self.step_remaining_rad, self.config.start_alignment_min_angular_speed,
                        self.config.angular_acceleration, self.ANGLE_STEP_TOLERANCE_RAD))

    def _update_step_progress(self, pose):
        if self.step_pose is None:
            return
        dx, dy = pose[0]-self.step_pose[0], pose[1]-self.step_pose[1]
        self.step_actual_m = (dx*math.cos(self.step_pose[2])+dy*math.sin(self.step_pose[2])
                              if self.step_axis == 'longitudinal' else
                              -dx*math.sin(self.step_pose[2])+dy*math.cos(self.step_pose[2])
                              if self.step_axis == 'lateral' else 0.)
        self.step_actual_rad = math.atan2(math.sin(pose[2]-self.step_pose[2]),
                                          math.cos(pose[2]-self.step_pose[2]))
        target = self.step_distance or self.step_lateral_m
        self.step_remaining_m = max(0., abs(target)-math.copysign(1., target)*self.step_actual_m)
        self.step_remaining_rad = max(0., abs(self.step_angle)-
                                      math.copysign(1., self.step_angle)*self.step_actual_rad)

    @staticmethod
    def _valid_marker(marker, width, height):
        if not isinstance(marker, dict):
            return False
        try:
            x, y = float(marker['x']), float(marker['y'])
            slope = float(marker.get('main_slope', 0.0))
        except (KeyError, TypeError, ValueError):
            return False
        return (all(math.isfinite(value) for value in (x, y, slope))
                and 0 <= x < width and 0 <= y < height and abs(slope) <= 2.0)

    def command(self, now, pose, odom_fresh=True, line_ready=True, stopped=False,
                marker_pending=False):
        """Advance the discrete motion/settling/observation cycle."""
        self.last_tick_at = now
        self.inputs_ready = line_ready and odom_fresh and pose is not None
        self.is_stopped = stopped and odom_fresh
        # A delayed detector is not a new camera outage while fresh matching
        # camera data and odometry keep confirming the same stationary window.
        # This permits collecting evidence only; line_ready still gates motion.
        self.marker_pending = bool(marker_pending and self.is_stopped and pose is not None)
        self._prune_observations(now)
        if self.fault:
            return self.fault
        if self.started_at is not None and now-self.started_at > self.config.start_alignment_timeout_sec:
            self.fault = self.state = 'start_alignment_failed_timeout'
            return self.state
        if pose is not None:
            if self.previous_pose is not None:
                delta = math.dist(pose[:2], self.previous_pose[:2])
                self.travelled_m += delta
                projected = ((pose[0]-self.previous_pose[0])*math.cos(self.previous_pose[2])
                             + (pose[1]-self.previous_pose[1])*math.sin(self.previous_pose[2]))
                if projected < 0.0:
                    self.reverse_travelled_m += delta
            self.previous_pose = tuple(pose[:3])
        if self.travelled_m > self.max_distance_m:
            self.fault = self.state = 'start_alignment_failed_distance_limit'
            return self.state
        if self.reverse_travelled_m > self.config.start_alignment_max_reverse_distance:
            self.fault = self.state = 'start_alignment_failed_reverse_limit'
            return self.state
        if not odom_fresh or pose is None:
            self._settle()
            self.state = 'start_alignment_waiting_for_odometry'
            return self.state
        # Include measured run-on during the stopped observation phase as well.
        self._update_step_progress(pose)
        if self.config.forward_speed == 0.:
            self.inputs_ready = False
            self._settle()
            self.state = 'start_alignment_waiting_for_speed'
            return self.state
        if self.phase == 'moving':
            if not line_ready:
                self._settle()  # Interrupted motion always needs a new stationary observation.
                self.resume_observation_after = now
                self.state = 'start_alignment_waiting_for_line'
                return self.state
            reached = (self.step_remaining_rad <= self.ANGLE_STEP_TOLERANCE_RAD
                       if self.step_axis == 'heading' else
                       self.step_remaining_m <= self.POSITION_STEP_TOLERANCE_M)
            if reached:
                self._settle(recheck=True)
            elif now-self.step_started_at > self.step_timeout_sec:
                self.fault = self.state = 'start_alignment_failed_step_timeout'
            return self.state
        if not stopped:
            # Keep the completed-step dwell policy through measured run-on;
            # interrupted moves already clear it in their own branch above.
            self._settle(recheck=self.recheck_settle)
            return self.state
        if self.stopped_since is None:
            self.stopped_since = now
            settle = (self.config.start_alignment_recheck_settle_sec if self.recheck_settle
                      else self.config.start_alignment_settle_sec)
            self.accept_after = now+settle
        # Camera dropout must not erase an odometry-confirmed stop. Keep the
        # bounded stationary sample window, but require a new captured frame on
        # recovery before issuing any motion or declaring completion.
        if not line_ready:
            if not self.marker_pending:
                self.resume_observation_after = now
            self.visual_ready = False
            self.state = 'start_alignment_waiting_for_line'
            if now >= self.accept_after:
                self.phase = 'observing'
            return self.state
        if now < self.accept_after:
            self.state = 'start_alignment_settling'
            return self.state
        self.phase = 'observing'
        if (not self.last_observation_valid or self.last_seen_at is None
                or self.last_seen_at < self.resume_observation_after
                or now-self.last_seen_at > .5):
            self.visual_ready = False
            self.state = 'start_alignment_waiting_for_marker'
            return self.state
        if self.visual_ready and self.window_stable():
            self.state = 'start_alignment_complete'
        elif self.window_stable(self.decision_frames):
            self._filter_observations(self.decision_frames)
            self._plan_step(now, pose)
        else:
            self.state = 'start_alignment_waiting_for_stable_image'
        return self.state

    def record_command(self, linear_x):
        self.last_command_v = float(linear_x)

    def snapshot(self):
        return dict(state=self.state, fault=self.fault, samples=len(self.samples),
                    phase=self.phase, axis=self.step_axis, step_count=self.step_count,
                    step_distance_m=self.step_distance, step_angle_rad=self.step_angle,
                    step_lateral_m=self.step_lateral_m,
                    step_remaining_m=self.step_remaining_m,
                    step_remaining_rad=self.step_remaining_rad,
                    step_actual_m=self.step_actual_m, step_actual_rad=self.step_actual_rad,
                    image_response_per_m=self.image_response_per_m,
                    lateral_response_per_m=self.lateral_response_per_m,
                    estimated_remaining_m=self.estimated_remaining_m,
                    estimated_correction_m=self.estimated_correction_m,
                    distance_estimate_source=self.distance_estimate_source,
                    response_reason=self.response_reason,
                    response_sample_counts={axis: len(values) for axis, values in self.response_samples.items()},
                    input_block_reason=self.input_block_reason, stopped=self.is_stopped,
                    observation_age_sec=(self.last_tick_at-self.last_seen_at
                                         if self.last_tick_at is not None and self.last_seen_at is not None else None),
                    filter_frames=self.filter_frames, decision_frames=self.decision_frames,
                    step_limit_m=self.step_limit_m, step_timeout_sec=self.step_timeout_sec,
                    inputs_ready=self.inputs_ready,
                    marker_pending=self.marker_pending,
                    observation_stability=self.observation_stability(),
                    stable_frames=self.stable_frames, confirmation_frames=self.confirmation_frames,
                    marker=self.marker, target_ratio=self.config.stop_marker_target_ratio,
                    position_error_ratio=self.position_error,
                    position_error_m=self.position_error_m,
                    lateral_error_ratio=self.lateral_error, heading_error_rad=self.heading_error,
                    lateral_error_unit=('m' if self.mapping is not None else 'image_ratio'),
                    metric=self.metric, geometry_mode='ground_meters' if self.metric else 'image_ratio',
                    travelled_m=self.travelled_m, max_distance_m=self.max_distance_m,
                    reverse_travelled_m=self.reverse_travelled_m,
                    started_at_monotonic=self.started_at)
