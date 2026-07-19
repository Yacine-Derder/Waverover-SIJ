"""Deterministic fixed-timestep synthetic fixed-wing motion primitives."""

from dataclasses import asdict
import math
import secrets

import numpy as np


ACTIONS = ('straight', 'bank_left', 'bank_right')


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def integrate_motion(x, y, yaw, speed, yaw_rate, timestep):
    """Exactly integrate one constant-speed, constant-turn-rate interval."""
    x = float(x)
    y = float(y)
    yaw = float(yaw)
    speed = float(speed)
    yaw_rate = float(yaw_rate)
    timestep = float(timestep)
    if speed <= 0.0:
        raise ValueError('Synthetic forward speed must be positive.')
    if timestep <= 0.0:
        raise ValueError('Synthetic timestep must be positive.')
    if abs(yaw_rate) <= 1e-12:
        return (
            x + speed * timestep * math.cos(yaw),
            y + speed * timestep * math.sin(yaw),
            wrap_angle(yaw),
        )
    next_yaw_unwrapped = yaw + yaw_rate * timestep
    radius = speed / yaw_rate
    return (
        x + radius * (math.sin(next_yaw_unwrapped) - math.sin(yaw)),
        y - radius * (math.cos(next_yaw_unwrapped) - math.cos(yaw)),
        wrap_angle(next_yaw_unwrapped),
    )


def action_primitive(vehicle, action):
    if action == 'straight':
        return vehicle.straight_speed_mps, 0.0
    if action == 'bank_left':
        return vehicle.turning_path_speed_mps, vehicle.bank_yaw_rate_rad_s
    if action == 'bank_right':
        return vehicle.turning_path_speed_mps, -vehicle.bank_yaw_rate_rad_s
    raise ValueError('Unknown synthetic action %s.' % action)


def preset_script(vehicle, preset):
    half_turn = math.pi / vehicle.bank_yaw_rate_rad_s
    full_turn = 2.0 * half_turn
    if preset == 'circle':
        return (('bank_left', full_turn),)
    if preset == 'racetrack':
        return (
            ('straight', 1.0),
            ('bank_left', half_turn),
            ('straight', 1.0),
            ('bank_left', half_turn),
        )
    if preset == 'figure_eight':
        return (
            ('bank_left', full_turn),
            ('bank_right', full_turn),
        )
    raise ValueError('Unknown synthetic preset %s.' % preset)


def yaw_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class SyntheticTrajectory:
    """Generate a rigid, safe-to-validate multi-rover trajectory."""

    def __init__(self, config, positions, rate_hz, initial_yaw=0.0, seed=None):
        self.config = config
        self.synthetic = config.synthetic_mcs
        self.timestep = 1.0 / float(rate_hz)
        self.actual_seed = (
            secrets.randbits(63) if seed is None and self.synthetic.seed is None
            else int(self.synthetic.seed if seed is None else seed)
        )
        self.rng = np.random.default_rng(self.actual_seed)
        self.positions = {
            robot_id: (float(point[0]), float(point[1]))
            for robot_id, point in sorted(positions.items())
        }
        self.yaw = wrap_angle(initial_yaw)
        self.elapsed = 0.0
        self.segment_index = 0
        self.segment_elapsed = 0.0
        self.generated_segments = []
        self._segments = self._initial_segments()
        self.last_action = 'static' if self.synthetic.mode == 'static' else None
        self.last_speed = 0.0
        self.last_yaw_rate = 0.0

    def _initial_segments(self):
        if self.synthetic.mode == 'static':
            return ()
        if self.synthetic.mode == 'scripted':
            values = tuple(
                (segment.action, segment.duration_sec)
                for segment in self.synthetic.script
            )
        elif self.synthetic.mode in ('preset', 'noisy_path'):
            values = preset_script(self.config.vehicle, self.synthetic.preset)
        else:
            values = ()
        if self.synthetic.mode == 'noisy_path':
            values = tuple(
                (action, duration * float(self.rng.uniform(0.85, 1.15)))
                for action, duration in values
            )
        self.generated_segments.extend(
            {'action': action, 'duration_sec': float(duration)}
            for action, duration in values
        )
        return values

    def _random_segment(self):
        action = str(self.rng.choice(ACTIONS))
        duration = float(self.rng.uniform(
            self.synthetic.segment_duration_min_sec,
            self.synthetic.segment_duration_max_sec,
        ))
        self.generated_segments.append({
            'action': action,
            'duration_sec': duration,
        })
        return action, duration

    def _current_segment(self):
        if self.synthetic.mode == 'random_walk':
            if self.segment_index >= len(self._segments):
                self._segments = self._segments + (self._random_segment(),)
            return self._segments[self.segment_index]
        return self._segments[self.segment_index % len(self._segments)]

    def _sample_primitive(self, action):
        base_speed, base_yaw_rate = action_primitive(
            self.config.vehicle, action
        )
        speed_noise = float(np.clip(
            self.rng.normal(0.0, self.synthetic.process_speed_std_mps),
            -3.0 * self.synthetic.process_speed_std_mps,
            3.0 * self.synthetic.process_speed_std_mps,
        ))
        yaw_noise = float(np.clip(
            self.rng.normal(0.0, self.synthetic.process_yaw_rate_std_rad_s),
            -3.0 * self.synthetic.process_yaw_rate_std_rad_s,
            3.0 * self.synthetic.process_yaw_rate_std_rad_s,
        ))
        speed = max(0.05 * base_speed, base_speed + speed_noise)
        yaw_rate = base_yaw_rate + yaw_noise
        if base_yaw_rate > 0.0:
            yaw_rate = max(0.05 * base_yaw_rate, yaw_rate)
        elif base_yaw_rate < 0.0:
            yaw_rate = min(0.05 * base_yaw_rate, yaw_rate)
        return speed, yaw_rate

    def step(self):
        """Advance exactly one configured timestep and return true motion."""
        if self.synthetic.mode == 'static':
            self.elapsed += self.timestep
            return self.positions, self.yaw, 'static', 0.0, 0.0
        action, duration = self._current_segment()
        speed, yaw_rate = self._sample_primitive(action)
        next_x, next_y, next_yaw = integrate_motion(
            0.0, 0.0, self.yaw, speed, yaw_rate, self.timestep
        )
        self.positions = {
            robot_id: (point[0] + next_x, point[1] + next_y)
            for robot_id, point in self.positions.items()
        }
        self.yaw = next_yaw
        self.elapsed += self.timestep
        self.segment_elapsed += self.timestep
        if self.segment_elapsed + 1e-12 >= duration:
            self.segment_elapsed = 0.0
            self.segment_index += 1
        self.last_action = action
        self.last_speed = speed
        self.last_yaw_rate = yaw_rate
        return self.positions, self.yaw, action, speed, yaw_rate

    def observed_formation(self, validator, maximum_attempts=50):
        """Sample bounded measurement noise and reject unsafe observations."""
        position_std = self.synthetic.measurement_position_std_m
        heading_std = self.synthetic.measurement_heading_std_rad
        if position_std == 0.0 and heading_std == 0.0:
            validator(self.positions)
            return self.positions, {
                robot_id: self.yaw for robot_id in self.positions
            }
        for _attempt in range(maximum_attempts):
            observed = {}
            headings = {}
            for robot_id, point in self.positions.items():
                noise = np.clip(
                    self.rng.normal(0.0, position_std, size=2),
                    -3.0 * position_std,
                    3.0 * position_std,
                )
                heading_noise = float(np.clip(
                    self.rng.normal(0.0, heading_std),
                    -3.0 * heading_std,
                    3.0 * heading_std,
                ))
                observed[robot_id] = (
                    float(point[0] + noise[0]),
                    float(point[1] + noise[1]),
                )
                headings[robot_id] = wrap_angle(self.yaw + heading_noise)
            try:
                validator(observed)
            except ValueError:
                continue
            return observed, headings
        raise ValueError(
            'Could not sample a safe synthetic MCS observation after %d attempts.'
            % maximum_attempts
        )

    def metadata(self):
        return {
            'schema_version': 1,
            'actual_seed': self.actual_seed,
            'mode': self.synthetic.mode,
            'preset': self.synthetic.preset,
            'formation_coupling': self.synthetic.formation_coupling,
            'timestep_sec': self.timestep,
            'duration_sec': self.synthetic.duration_sec,
            'noise': {
                'process_speed_std_mps': self.synthetic.process_speed_std_mps,
                'process_yaw_rate_std_rad_s': (
                    self.synthetic.process_yaw_rate_std_rad_s
                ),
                'measurement_position_std_m': (
                    self.synthetic.measurement_position_std_m
                ),
                'measurement_heading_std_rad': (
                    self.synthetic.measurement_heading_std_rad
                ),
            },
            'vehicle': asdict(self.config.vehicle),
            'generated_segments': list(self.generated_segments),
        }
