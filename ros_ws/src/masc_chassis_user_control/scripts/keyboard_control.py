"""Keyboard command state, independent of ROS and terminal access."""

import math


class KeyboardControl:
    MOVES = {'w': (1, 0, 0), 's': (-1, 0, 0),
             'a': (0, 1, 0), 'd': (0, -1, 0),
             'q': (0, 0, 1), 'e': (0, 0, -1),
             'z': (-1, 0, 1), 'c': (-1, 0, -1)}

    def __init__(self, linear_speed=0.05, angular_speed=0.2, key_timeout=0.35):
        if not all(math.isfinite(v) for v in (linear_speed, angular_speed, key_timeout)):
            raise ValueError('keyboard settings must be finite')
        if not 0.01 <= linear_speed <= 0.3 or not 0.05 <= angular_speed <= 0.8:
            raise ValueError('linear_speed must be 0.01..0.3 m/s; angular_speed 0.05..0.8 rad/s')
        if not 0.1 <= key_timeout <= 1.0:
            raise ValueError('key_timeout must be 0.1..1.0 seconds')
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.timeout = key_timeout
        self.direction = (0, 0, 0)
        self.last_motion_key = -math.inf

    def key(self, key, now):
        key = key.lower()
        if key in self.MOVES:
            self.direction = self.MOVES[key]
            self.last_motion_key = now
        elif key in ('+', '=', '-', '_'):
            factor = 1.1 if key in ('+', '=') else 1 / 1.1
            self.linear_speed = max(0.01, min(0.3, self.linear_speed * factor))
            self.angular_speed = max(0.05, min(0.8, self.angular_speed * factor))
            self.direction = (0, 0, 0)  # Speed adjustment itself must not start or prolong movement.
        else:
            self.direction = (0, 0, 0)  # Space, X and unknown keys all stop.

    def command(self, now):
        if now - self.last_motion_key >= self.timeout:
            self.direction = (0, 0, 0)
        return (self.direction[0] * self.linear_speed, self.direction[1] * self.linear_speed,
                self.direction[2] * self.angular_speed)
