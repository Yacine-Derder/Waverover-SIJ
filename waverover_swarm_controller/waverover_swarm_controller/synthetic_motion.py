"""Deterministic fixed-timestep synthetic fixed-wing motion primitives."""

from dataclasses import asdict, dataclass, replace
import hashlib
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


def derive_rover_seed(master_seed, robot_id):
    """Derive an order-independent RNG seed from a master seed and full ID."""
    payload = ('%d\0%s' % (int(master_seed), str(robot_id))).encode('utf-8')
    return int.from_bytes(hashlib.sha256(payload).digest()[:8], 'big')


@dataclass(frozen=True)
class RoverMotionState:
    robot_id: str
    derived_seed: int
    x: float
    y: float
    yaw: float
    current_action: str
    segment_index: int
    segment_elapsed_sec: float
    current_segment_duration_sec: float
    last_speed_mps: float
    last_yaw_rate_rad_s: float
    generated_segments: tuple


@dataclass(frozen=True)
class TrajectoryStep:
    positions: dict
    headings: dict
    actions: dict
    speeds: dict
    yaw_rates: dict
    corrective: bool = False

    def __iter__(self):
        """Keep the legacy rigid five-tuple unpacking API usable."""
        first = min(self.positions)
        yield self.positions
        yield self.headings[first]
        yield self.actions[first]
        yield self.speeds[first]
        yield self.yaw_rates[first]


class SyntheticTrajectory:
    """Generate rigid or independent atomic multi-rover trajectories."""

    def __init__(
        self, config, positions, rate_hz, initial_yaw=0.0, seed=None,
        initial_radius_m=None,
    ):
        self.config = config
        self.synthetic = config.synthetic_mcs
        self.timestep = 1.0 / float(rate_hz)
        self.actual_seed = (
            secrets.randbits(63) if seed is None and self.synthetic.seed is None
            else int(self.synthetic.seed if seed is None else seed)
        )
        if initial_radius_m is None:
            center = config.station.position
            initial_radius_m = sum(
                math.dist(point, center) for point in positions.values()
            ) / len(positions)
        self.initial_radius_m = float(initial_radius_m)
        self.derived_seeds = {
            str(robot_id): derive_rover_seed(self.actual_seed, robot_id)
            for robot_id in sorted(positions)
        }
        self._rng_by_id = {
            robot_id: np.random.default_rng(rover_seed)
            for robot_id, rover_seed in self.derived_seeds.items()
        }
        self._rigid_rng = np.random.default_rng(self.actual_seed)
        self.states = {}
        for robot_id, point in sorted(positions.items()):
            action, duration = self._initial_segment(robot_id)
            self.states[robot_id] = RoverMotionState(
                robot_id=robot_id,
                derived_seed=self.derived_seeds[robot_id],
                x=float(point[0]),
                y=float(point[1]),
                yaw=wrap_angle(initial_yaw),
                current_action=action,
                segment_index=0,
                segment_elapsed_sec=0.0,
                current_segment_duration_sec=duration,
                last_speed_mps=0.0,
                last_yaw_rate_rad_s=0.0,
                generated_segments=(self._segment_record(action, duration),)
                if action != 'static' else (),
            )
        # Rigid mode historically used one master stream and one segment phase.
        if self.synthetic.formation_coupling == 'rigid' and self.states:
            first_id = min(self.states)
            common_action, common_duration = self._initial_segment(
                first_id, rng=self._rigid_rng
            )
            common_history = (
                (self._segment_record(common_action, common_duration),)
                if common_action != 'static' else ()
            )
            self.states = {
                robot_id: replace(
                    state,
                    current_action=common_action,
                    current_segment_duration_sec=common_duration,
                    generated_segments=common_history,
                )
                for robot_id, state in self.states.items()
            }
        self.elapsed = 0.0
        self.candidate_rejections = 0
        self.candidate_rejection_causes = {}
        self.corrective_interventions = 0
        self.corrective_intervention_causes = {}
        self.observation_rejections = 0
        self.observation_rejection_causes = {}
        self.last_true_validation = None
        self.last_observed_validation = None

    @staticmethod
    def _segment_record(action, duration):
        return {'action': action, 'duration_sec': float(duration)}

    @property
    def positions(self):
        return {
            robot_id: (state.x, state.y)
            for robot_id, state in self.states.items()
        }

    @property
    def generated_segments(self):
        if self.synthetic.formation_coupling == 'rigid':
            return list(next(iter(self.states.values())).generated_segments)
        return {
            robot_id: list(state.generated_segments)
            for robot_id, state in self.states.items()
        }

    def _initial_segment(self, robot_id, rng=None):
        if self.synthetic.mode == 'static':
            return 'static', 0.0
        return self._segment_for_index(robot_id, 0, rng=rng)

    def _segment_for_index(self, robot_id, index, rng=None):
        rng = rng if rng is not None else self._rng_by_id[robot_id]
        if self.synthetic.mode == 'scripted':
            segment = self.synthetic.script[index % len(self.synthetic.script)]
            return segment.action, float(segment.duration_sec)
        if self.synthetic.mode in ('preset', 'noisy_path'):
            action, duration = preset_script(
                self.config.vehicle, self.synthetic.preset
            )[index % len(preset_script(self.config.vehicle, self.synthetic.preset))]
            if self.synthetic.mode == 'noisy_path':
                duration *= float(rng.uniform(0.85, 1.15))
            return action, float(duration)
        return (
            str(rng.choice(ACTIONS)),
            float(rng.uniform(
                self.synthetic.segment_duration_min_sec,
                self.synthetic.segment_duration_max_sec,
            )),
        )

    def _sample_primitive(self, action, rng):
        base_speed, base_yaw_rate = action_primitive(self.config.vehicle, action)
        speed_std = self.synthetic.process_speed_std_mps
        yaw_std = self.synthetic.process_yaw_rate_std_rad_s
        speed_noise = float(np.clip(
            rng.normal(0.0, speed_std), -3.0 * speed_std, 3.0 * speed_std
        ))
        yaw_noise = float(np.clip(
            rng.normal(0.0, yaw_std), -3.0 * yaw_std, 3.0 * yaw_std
        ))
        speed = max(0.05 * base_speed, base_speed + speed_noise)
        yaw_rate = base_yaw_rate + yaw_noise
        if base_yaw_rate > 0.0:
            yaw_rate = max(0.05 * base_yaw_rate, yaw_rate)
        elif base_yaw_rate < 0.0:
            yaw_rate = min(0.05 * base_yaw_rate, yaw_rate)
        else:
            # Straight remains approximately straight under bounded process noise.
            yaw_rate = yaw_noise
        return float(speed), float(yaw_rate)

    def _candidate(self, actions, samples):
        candidate = {}
        for robot_id, state in self.states.items():
            action = actions[robot_id]
            speed, yaw_rate = samples[robot_id][action]
            x, y, yaw = integrate_motion(
                state.x, state.y, state.yaw, speed, yaw_rate, self.timestep
            )
            candidate[robot_id] = replace(
                state,
                x=x,
                y=y,
                yaw=yaw,
                current_action=action,
                last_speed_mps=speed,
                last_yaw_rate_rad_s=yaw_rate,
            )
        return candidate

    def _correction_score(self, robot_id, candidate):
        state = candidate[robot_id]
        fence = self.config.safety.geofence
        boundary = min(
            state.x - fence.x_min, fence.x_max - state.x,
            state.y - fence.y_min, fence.y_max - state.y,
        )
        separation = min((
            math.hypot(state.x - other.x, state.y - other.y)
            for other_id, other in candidate.items() if other_id != robot_id
        ), default=math.inf)
        return boundary + (2.0 * separation if math.isfinite(separation) else 0.0)

    def _avoidance_score(self, candidate):
        """Rank bounded corrections by a short constant-bank projection."""
        current_values = list(candidate.values())
        current_separation = min(
            math.hypot(first.x - second.x, first.y - second.y)
            for index, first in enumerate(current_values)
            for second in current_values[index + 1:]
        )
        projected = {
            robot_id: replace(
                state,
                **dict(zip(('x', 'y', 'yaw'), integrate_motion(
                    state.x,
                    state.y,
                    state.yaw,
                    state.last_speed_mps,
                    state.last_yaw_rate_rad_s,
                    0.75,
                )))
            )
            for robot_id, state in candidate.items()
        }
        values = list(projected.values())
        final_separation = min(
            math.hypot(first.x - second.x, first.y - second.y)
            for index, first in enumerate(values)
            for second in values[index + 1:]
        )
        fence = self.config.safety.geofence
        boundary = min(
            min(
                state.x - fence.x_min, fence.x_max - state.x,
                state.y - fence.y_min, fence.y_max - state.y,
            )
            for state in values
        )
        return final_separation + 0.1 * current_separation + 0.25 * boundary

    def _transition_guard(self, candidate):
        """Reserve one tick of closing distance for nonholonomic correction."""
        values = list(candidate.values())
        if len(values) < 2:
            return True
        separation = min(
            math.hypot(first.x - second.x, first.y - second.y)
            for index, first in enumerate(values)
            for second in values[index + 1:]
        )
        # Three turn radii give bank-only corrections time to alter heading;
        # a one-tick reserve is insufficient for fixed-wing kinematics when
        # two rovers are already closing head-on.
        reserve = 3.0 * self.config.vehicle.turn_radius_m
        fence = self.config.safety.geofence
        boundary = min(
            min(
                state.x - fence.x_min, fence.x_max - state.x,
                state.y - fence.y_min, fence.y_max - state.y,
            )
            for state in values
        )
        return (
            separation >= self.config.safety.minimum_separation_m + reserve
            and boundary >= reserve
        )

    def _commit_candidate(self, candidate, validation, corrective):
        self.states = self._advance_segments(candidate)
        self.elapsed += self.timestep
        self.last_true_validation = validation
        return TrajectoryStep(
            positions=self.positions,
            headings={key: value.yaw for key, value in self.states.items()},
            actions={key: value.current_action for key, value in candidate.items()},
            speeds={key: value.last_speed_mps for key, value in candidate.items()},
            yaw_rates={key: value.last_yaw_rate_rad_s for key, value in candidate.items()},
            corrective=corrective,
        )

    @staticmethod
    def _record_cause(target, error):
        cause = str(error)
        target[cause] = target.get(cause, 0) + 1

    def _advance_segments(self, candidate):
        output = {}
        for robot_id, state in candidate.items():
            elapsed = self.states[robot_id].segment_elapsed_sec + self.timestep
            index = self.states[robot_id].segment_index
            duration = self.states[robot_id].current_segment_duration_sec
            history = self.states[robot_id].generated_segments
            action = self.states[robot_id].current_action
            if elapsed + 1e-12 >= duration:
                elapsed = max(0.0, elapsed - duration)
                index += 1
                rng = (
                    self._rigid_rng if self.synthetic.formation_coupling == 'rigid'
                    else self._rng_by_id[robot_id]
                )
                action, duration = self._segment_for_index(robot_id, index, rng=rng)
                history = history + (self._segment_record(action, duration),)
            output[robot_id] = replace(
                state,
                current_action=action,
                segment_index=index,
                segment_elapsed_sec=elapsed,
                current_segment_duration_sec=duration,
                generated_segments=history,
            )
        if self.synthetic.formation_coupling == 'rigid' and output:
            first = output[min(output)]
            output = {
                robot_id: replace(
                    state,
                    segment_index=first.segment_index,
                    segment_elapsed_sec=first.segment_elapsed_sec,
                    current_segment_duration_sec=first.current_segment_duration_sec,
                    generated_segments=first.generated_segments,
                )
                for robot_id, state in output.items()
            }
        return output

    def step(self, validator=None):
        """Atomically advance one fixed timestep, with bounded corrections."""
        if self.synthetic.mode == 'static':
            if validator is not None:
                self.last_true_validation = validator(self.positions)
            self.elapsed += self.timestep
            return TrajectoryStep(
                self.positions,
                {robot_id: state.yaw for robot_id, state in self.states.items()},
                {robot_id: 'static' for robot_id in self.states},
                {robot_id: 0.0 for robot_id in self.states},
                {robot_id: 0.0 for robot_id in self.states},
            )

        scheduled = {
            robot_id: state.current_action
            for robot_id, state in self.states.items()
        }
        samples = {}
        if self.synthetic.formation_coupling == 'rigid':
            shared = {
                action: self._sample_primitive(action, self._rigid_rng)
                for action in ACTIONS
            }
            samples = {robot_id: shared for robot_id in self.states}
        else:
            samples = {
                robot_id: {
                    action: self._sample_primitive(action, self._rng_by_id[robot_id])
                    for action in ACTIONS
                }
                for robot_id in self.states
            }

        proposals = [scheduled]
        deterministic_proposal_count = 1
        if self.synthetic.formation_coupling == 'independent':
            # Deterministic bounded greedy search: single-rover alternatives,
            # followed by cumulative locally best corrections.
            for robot_id in sorted(self.states):
                for action in ACTIONS:
                    if action != scheduled[robot_id]:
                        proposal = dict(scheduled)
                        proposal[robot_id] = action
                        proposals.append(proposal)
            cumulative = dict(scheduled)
            for robot_id in sorted(self.states):
                ranked = []
                for action in ACTIONS:
                    trial = dict(cumulative)
                    trial[robot_id] = action
                    candidate = self._candidate(trial, samples)
                    ranked.append((
                        -self._correction_score(robot_id, candidate), action
                    ))
                cumulative[robot_id] = min(ranked)[1]
                proposals.append(dict(cumulative))
            deterministic_proposal_count = len(proposals)

            # Fill the remaining bounded budget with deterministic joint
            # proposals. This is linear in the configured attempt limit, not
            # an exponential enumeration of all 3^N action combinations.
            tick_index = int(round(self.elapsed / self.timestep))
            search_rng = np.random.default_rng(derive_rover_seed(
                self.actual_seed, '__transition_%d' % tick_index
            ))
            while len(proposals) < self.synthetic.maximum_transition_attempts:
                proposals.append({
                    robot_id: str(search_rng.choice(ACTIONS))
                    for robot_id in sorted(self.states)
                })

        attempts = min(self.synthetic.maximum_transition_attempts, len(proposals))
        last_error = None
        best_valid = None
        for attempt, actions in enumerate(proposals[:attempts]):
            candidate = self._candidate(actions, samples)
            positions = {
                robot_id: (state.x, state.y)
                for robot_id, state in candidate.items()
            }
            try:
                validation = validator(positions) if validator is not None else None
            except ValueError as error:
                last_error = error
                self.candidate_rejections += 1
                self._record_cause(self.candidate_rejection_causes, error)
                continue
            if not self._transition_guard(candidate):
                score = self._avoidance_score(candidate)
                if best_valid is None or score > best_valid[0]:
                    best_valid = (score, candidate, validation)
                self.candidate_rejections += 1
                reserve_error = (
                    'candidate entered fixed-wing collision-correction reserve'
                )
                self._record_cause(
                    self.candidate_rejection_causes, reserve_error
                )
                last_error = ValueError(reserve_error)
                if (
                    best_valid is not None
                    and attempt + 1 >= deterministic_proposal_count
                ):
                    break
                continue
            corrective = attempt > 0
            if corrective:
                self.corrective_interventions += 1
                self._record_cause(
                    self.corrective_intervention_causes,
                    last_error or 'scheduled candidate rejected',
                )
            return self._commit_candidate(candidate, validation, corrective)
        if best_valid is not None:
            _score, candidate, validation = best_valid
            self.corrective_interventions += 1
            self._record_cause(
                self.corrective_intervention_causes,
                last_error or 'collision-correction reserve',
            )
            return self._commit_candidate(candidate, validation, True)
        raise ValueError(
            'Could not find a safe atomic synthetic transition after %d attempts: %s'
            % (attempts, last_error or 'no candidate was valid')
        )

    def observed_formation(self, validator, maximum_attempts=None):
        """Sample bounded measurement noise without changing true state."""
        attempts = maximum_attempts or self.synthetic.maximum_transition_attempts
        position_std = self.synthetic.measurement_position_std_m
        heading_std = self.synthetic.measurement_heading_std_rad
        for _attempt in range(attempts):
            observed = {}
            headings = {}
            for robot_id, state in self.states.items():
                rng = self._rng_by_id[robot_id]
                noise = np.clip(
                    rng.normal(0.0, position_std, size=2),
                    -3.0 * position_std,
                    3.0 * position_std,
                )
                heading_noise = float(np.clip(
                    rng.normal(0.0, heading_std),
                    -3.0 * heading_std,
                    3.0 * heading_std,
                ))
                observed[robot_id] = (
                    float(state.x + noise[0]), float(state.y + noise[1])
                )
                headings[robot_id] = wrap_angle(state.yaw + heading_noise)
            try:
                self.last_observed_validation = validator(observed)
            except ValueError as error:
                self.observation_rejections += 1
                self._record_cause(self.observation_rejection_causes, error)
                continue
            return observed, headings
        raise ValueError(
            'Could not sample a safe synthetic MCS observation after %d attempts.'
            % attempts
        )

    @staticmethod
    def _validation_metadata(validation):
        if validation is None:
            return None
        if isinstance(validation, dict):
            return dict(validation)
        if not hasattr(validation, '__dataclass_fields__'):
            return validation
        return dict(asdict(validation))

    def metadata(self):
        true = self._validation_metadata(self.last_true_validation)
        observed = self._validation_metadata(self.last_observed_validation)
        return {
            'schema_version': 2,
            'actual_master_seed': self.actual_seed,
            # Retain the v1 key for older consumers.
            'actual_seed': self.actual_seed,
            'derived_rover_seeds': dict(self.derived_seeds),
            'mode': self.synthetic.mode,
            'preset': self.synthetic.preset,
            'formation_coupling': self.synthetic.formation_coupling,
            'connectivity_policy': self.synthetic.connectivity_policy,
            'initial_radius_m': self.initial_radius_m,
            'timestep_sec': self.timestep,
            'duration_sec': self.synthetic.duration_sec,
            'noise': {
                'process_speed_std_mps': self.synthetic.process_speed_std_mps,
                'process_yaw_rate_std_rad_s': self.synthetic.process_yaw_rate_std_rad_s,
                'measurement_position_std_m': self.synthetic.measurement_position_std_m,
                'measurement_heading_std_rad': self.synthetic.measurement_heading_std_rad,
            },
            'vehicle': asdict(self.config.vehicle),
            'rovers': {
                robot_id: asdict(state) for robot_id, state in self.states.items()
            },
            'generated_segments': self.generated_segments,
            'candidate_rejections': {
                'count': self.candidate_rejections,
                'causes': dict(self.candidate_rejection_causes),
                'observation_count': self.observation_rejections,
                'observation_causes': dict(self.observation_rejection_causes),
            },
            'corrective_interventions': {
                'count': self.corrective_interventions,
                'causes': dict(self.corrective_intervention_causes),
            },
            'true_formation': true,
            'observed_formation': observed,
            'current_true_minimum_separation_m': (
                true.get('minimum_separation_m') if isinstance(true, dict) else None
            ),
            'current_observed_minimum_separation_m': (
                observed.get('minimum_separation_m')
                if isinstance(observed, dict) else None
            ),
            'current_true_binary_lambda_2': (
                true.get('binary_lambda_2') if isinstance(true, dict) else None
            ),
            'current_true_weighted_lambda_2': (
                true.get('weighted_lambda_2') if isinstance(true, dict) else None
            ),
            'current_observed_binary_lambda_2': (
                observed.get('binary_lambda_2')
                if isinstance(observed, dict) else None
            ),
            'current_observed_weighted_lambda_2': (
                observed.get('weighted_lambda_2')
                if isinstance(observed, dict) else None
            ),
            'disconnected': {
                'true': true.get('disconnected') if isinstance(true, dict) else None,
                'observed': (
                    observed.get('disconnected')
                    if isinstance(observed, dict) else None
                ),
            },
        }
