"""Event-triggered dispatcher that respects the onboard FIFO."""

from dataclasses import dataclass, field
import math


SUPERSEDED_TOKEN_CACHE_SIZE = 32


def token_value(token):
    """Return an immutable machine-readable stamp token."""
    return None if token is None else tuple(token)


@dataclass(frozen=True)
class DispatchAction:
    kind: str
    robot_id: str
    point: tuple = None
    reason: str = ''
    token: tuple = None
    target_epoch: int = 0
    objective_revision: int = 0


@dataclass
class RoverDispatchState:
    active_waypoint: tuple = None
    active_requested_waypoint: tuple = None
    active_token: tuple = None
    active_target_epoch: int = 0
    pending_waypoint: tuple = None
    pending_requested_waypoint: tuple = None
    pending_target_epoch: int = 0
    active_objective_revision: int = 0
    pending_objective_revision: int = 0
    active_command_revision: int = 0
    pending_command_revision: int = 0
    pending_supersede: bool = False
    active_published_at: float = None
    last_published_at: float = None
    refresh_count: int = 0
    ever_commanded: bool = False
    last_acknowledged_token: tuple = None
    last_acknowledged_waypoint: tuple = None
    last_acknowledged_at: float = None
    last_acknowledged_target_epoch: int = 0
    acknowledgement_count: int = 0
    unmatched_acknowledgement_count: int = 0
    handoff_cause: str = 'none'
    suppression_reason: str = ''
    suppression_count: int = 0
    last_failed_token: tuple = None
    last_failed_waypoint: tuple = None
    last_failed_target_epoch: int = 0
    failure_count: int = 0
    unmatched_failure_count: int = 0
    retained_due_to_deadband: bool = False
    deadband_retention_count: int = 0
    superseded_tokens: list = field(default_factory=list)
    superseded_token_count: int = 0
    superseded_acknowledgement_count: int = 0
    last_superseded_token: tuple = None
    last_superseded_by_token: tuple = None


class WaypointDispatcher:
    def __init__(self, robot_ids, config, safety_config=None):
        self.config = config
        self.safety_config = safety_config
        self.states = {
            str(robot_id): RoverDispatchState()
            for robot_id in sorted(robot_ids)
        }
        self.faulted = False
        self.fault_reason = ''
        self._last_token_ns = 0
        self.last_activation_failure = ''
        self.last_activation_report = None
        self._failed_activation_signature = None

    @property
    def commanded_robot_ids(self):
        return tuple(sorted(
            robot_id
            for robot_id, state in self.states.items()
            if state.ever_commanded
        ))

    def update_pending(
        self, setpoints, target_epoch=0, objective_revision=None,
        command_revision=None, allow_supersession=False,
        waypoint_deadband_m=0.0,
    ):
        revision = (
            int(target_epoch) if objective_revision is None
            else int(objective_revision)
        )
        generation = (
            revision if command_revision is None else int(command_revision)
        )
        deadband = max(0.0, float(waypoint_deadband_m))
        updated = False
        for robot_id, point in setpoints.items():
            if robot_id not in self.states:
                raise ValueError('Unknown dispatcher robot %s.' % robot_id)
            state = self.states[robot_id]
            incoming = (revision, generation)
            newest = max(
                (state.pending_objective_revision,
                 state.pending_command_revision),
                (state.active_objective_revision,
                 state.active_command_revision),
            )
            if incoming < newest:
                continue
            updated = True
            candidate = (
                float(point[0]),
                float(point[1]),
            )
            state.pending_target_epoch = int(target_epoch)
            state.pending_objective_revision = revision
            state.pending_command_revision = generation
            state.retained_due_to_deadband = False
            active_reference = (
                state.active_requested_waypoint
                if state.active_requested_waypoint is not None
                else state.active_waypoint
            )
            if (
                allow_supersession
                and active_reference is not None
                and math.dist(candidate, active_reference) <= deadband
            ):
                # This newest reactive generation intentionally retains the
                # active token and invalidates any older pending generation.
                state.pending_waypoint = None
                state.pending_requested_waypoint = None
                state.pending_supersede = False
                state.retained_due_to_deadband = True
                state.deadband_retention_count += 1
                state.handoff_cause = 'reactive_deadband_retained'
                continue
            state.pending_waypoint = candidate
            state.pending_requested_waypoint = candidate
            state.pending_supersede = bool(
                allow_supersession and state.active_waypoint is not None
            )
        if updated:
            self._failed_activation_signature = None
            self.last_activation_failure = ''

    def _publish_pending(self, robot_id, state, now):
        point = state.pending_waypoint
        superseded_token = (
            state.active_token
            if state.pending_supersede and state.active_token is not None
            else None
        )
        if superseded_token is not None:
            state.superseded_tokens.append(tuple(superseded_token))
            del state.superseded_tokens[:-SUPERSEDED_TOKEN_CACHE_SIZE]
            state.superseded_token_count += 1
            state.last_superseded_token = tuple(superseded_token)
        state.active_waypoint = point
        state.active_requested_waypoint = state.pending_requested_waypoint
        # A stamp-shaped, process-local correlation token. The ROS boundary
        # may replace it, but refreshes always reuse this exact value.
        token_ns = max(
            self._last_token_ns + 1,
            max(1, int(float(now) * 1000000000)),
        )
        self._last_token_ns = token_ns
        candidate = (token_ns // 1000000000, token_ns % 1000000000)
        state.active_token = candidate
        state.active_target_epoch = state.pending_target_epoch
        state.active_objective_revision = state.pending_objective_revision
        state.active_command_revision = state.pending_command_revision
        state.pending_waypoint = None
        state.pending_requested_waypoint = None
        state.pending_supersede = False
        state.active_published_at = now
        state.last_published_at = now
        state.refresh_count = 0
        state.handoff_cause = (
            'reactive_supersession' if superseded_token is not None
            else 'initial' if not state.ever_commanded
            else 'acknowledgement'
        )
        state.suppression_reason = ''
        state.ever_commanded = True
        if superseded_token is not None:
            state.last_superseded_by_token = state.active_token
        return DispatchAction(
            'waypoint', robot_id, point=point, token=state.active_token,
            reason=state.handoff_cause,
            target_epoch=state.active_target_epoch,
            objective_revision=state.active_objective_revision,
        )

    def _activation_signature(self, robot_ids):
        return (
            tuple(sorted(
                (robot_id, self.states[robot_id].pending_waypoint,
                 self.states[robot_id].pending_objective_revision,
                 self.states[robot_id].pending_command_revision,
                 self.states[robot_id].pending_supersede)
                for robot_id in robot_ids
            )),
            tuple(sorted(
                (robot_id, state.active_waypoint)
                for robot_id, state in self.states.items()
                if state.active_waypoint is not None
                and robot_id not in robot_ids
            )),
        )

    def _activate_pending(self, robot_ids, now):
        robot_ids = tuple(sorted(
            robot_id for robot_id in robot_ids
            if self.states[robot_id].pending_waypoint is not None
            and (
                self.states[robot_id].active_waypoint is None
                or self.states[robot_id].pending_supersede
            )
        ))
        if not robot_ids:
            return []
        signature = self._activation_signature(robot_ids)
        if signature == self._failed_activation_signature:
            return []
        if self.safety_config is not None:
            from .waypoint_repair import repair_outgoing_endpoints
            proposed = {
                robot_id: self.states[robot_id].pending_waypoint
                for robot_id in robot_ids
            }
            fixed = {
                robot_id: state.active_waypoint
                for robot_id, state in self.states.items()
                if state.active_waypoint is not None
                and robot_id not in robot_ids
            }
            revision = max(
                self.states[robot_id].pending_objective_revision
                for robot_id in robot_ids
            )
            corrected, report = repair_outgoing_endpoints(
                proposed,
                fixed,
                self.safety_config.geofence,
                self.safety_config.minimum_separation_m,
                self.safety_config.collision_repair_max_iterations,
                revision,
            )
            self.last_activation_report = report
            if report.residual_violation_m > 1e-9:
                self.last_activation_failure = (
                    'outgoing_waypoint_separation_failed: residual %.6f m'
                    % report.residual_violation_m
                )
                self._failed_activation_signature = signature
                return []
            for robot_id, point in corrected.items():
                self.states[robot_id].pending_waypoint = point
        self.last_activation_failure = ''
        self._failed_activation_signature = None
        return [
            self._publish_pending(robot_id, self.states[robot_id], now)
            for robot_id in robot_ids
        ]

    def _suppress_repeated(self, state, measured_position):
        point = state.pending_waypoint
        epoch = state.pending_target_epoch
        epsilon = self.config.repeated_destination_epsilon_m
        if (
            state.last_failed_waypoint is not None and
            epoch == state.last_failed_target_epoch and
            math.dist(point, state.last_failed_waypoint) <= epsilon
        ):
            return 'same_destination_as_failed_token'
        if (
            state.last_acknowledged_waypoint is None or
            epoch != state.last_acknowledged_target_epoch or
            math.dist(point, state.last_acknowledged_waypoint) > epsilon
        ):
            return ''
        if measured_position is None:
            return 'completed_destination_position_unknown'
        if math.dist(measured_position, state.last_acknowledged_waypoint) <= (
            self.config.completed_destination_reissue_distance_m
        ):
            return 'completed_destination_hold_continuation'
        return ''

    def _drop_suppressed_pending(self, state, measured_position):
        reason = self._suppress_repeated(state, measured_position)
        if not reason:
            return False
        state.pending_waypoint = None
        state.pending_requested_waypoint = None
        state.suppression_reason = reason
        state.suppression_count += 1
        state.handoff_cause = 'suppressed'
        return True

    def acknowledge(self, robot_id, frame_id, token, point, now, expected_frame,
                    measured_position=None):
        state = self.states.get(str(robot_id))
        try:
            point = (float(point[0]), float(point[1]))
        except (TypeError, ValueError, IndexError):
            point = (float('nan'), float('nan'))
        valid = (
            state is not None and frame_id == expected_frame and
            token is not None and tuple(token) == state.active_token and
            all(__import__('math').isfinite(value) for value in point) and
            state.active_waypoint is not None and
            all(abs(a - b) <= 1e-6 for a, b in zip(point, state.active_waypoint))
        )
        if not valid:
            if state is not None:
                if token is not None and tuple(token) in state.superseded_tokens:
                    state.superseded_acknowledgement_count += 1
                else:
                    state.unmatched_acknowledgement_count += 1
            return []
        state.last_acknowledged_token = state.active_token
        state.last_acknowledged_waypoint = state.active_waypoint
        state.last_acknowledged_at = float(now)
        state.last_acknowledged_target_epoch = state.active_target_epoch
        state.acknowledgement_count += 1
        state.active_waypoint = None
        state.active_requested_waypoint = None
        state.active_token = None
        state.active_published_at = None
        state.last_published_at = None
        state.refresh_count = 0
        state.handoff_cause = 'acknowledgement'
        if state.pending_waypoint is None:
            return []
        if self._drop_suppressed_pending(state, measured_position):
            return []
        return self._activate_pending((str(robot_id),), now)

    def fail(self, robot_id, frame_id, token, point, now, expected_frame):
        """Exact-match an onboard navigation_stalled notification."""
        state = self.states.get(str(robot_id))
        try:
            point = (float(point[0]), float(point[1]))
        except (TypeError, ValueError, IndexError):
            point = (float('nan'), float('nan'))
        valid = (
            state is not None and frame_id == expected_frame and
            token is not None and tuple(token) == state.active_token and
            all(math.isfinite(value) for value in point) and
            state.active_waypoint is not None and
            all(abs(a - b) <= 1e-6 for a, b in zip(point, state.active_waypoint))
        )
        if not valid:
            if state is not None:
                state.unmatched_failure_count += 1
            return False
        state.last_failed_token = state.active_token
        state.last_failed_waypoint = state.active_waypoint
        state.last_failed_target_epoch = state.active_target_epoch
        state.failure_count += 1
        state.active_waypoint = None
        state.active_requested_waypoint = None
        state.active_token = None
        state.active_published_at = None
        state.last_published_at = None
        state.refresh_count = 0
        state.handoff_cause = 'navigation_stalled'
        if state.pending_waypoint is not None:
            self._drop_suppressed_pending(state, None)
        return True

    def tick(self, snapshot, now, commands_enabled):
        if self.faulted or not commands_enabled:
            return []
        actions = []
        eligible = []
        for robot_id in sorted(self.states):
            state = self.states[robot_id]
            if (
                state.pending_waypoint is not None
                and state.pending_supersede
            ):
                eligible.append(robot_id)
                continue
            if state.active_waypoint is None:
                if state.pending_waypoint is not None:
                    robot = snapshot.robots.get(robot_id) if snapshot else None
                    position = None if robot is None else robot.position
                    if self._drop_suppressed_pending(state, position):
                        continue
                    eligible.append(robot_id)
                continue
            if now - state.last_published_at >= self.config.refresh_period_sec:
                state.last_published_at = now
                state.refresh_count += 1
                actions.append(DispatchAction(
                    'refresh', robot_id, point=state.active_waypoint,
                    token=state.active_token, target_epoch=state.active_target_epoch,
                    objective_revision=state.active_objective_revision,
                ))
        return self._activate_pending(eligible, now) + actions

    def observability(self, now):
        output = {}
        for robot_id, state in sorted(self.states.items()):
            active_age = (
                None if state.active_published_at is None else
                max(0.0, now - state.active_published_at)
            )
            publication_age = (
                None if state.last_published_at is None else
                max(0.0, now - state.last_published_at)
            )
            output[robot_id] = {
                'active_waypoint': state.active_waypoint,
                'active_requested_waypoint': state.active_requested_waypoint,
                'pending_waypoint': state.pending_waypoint,
                'pending_requested_waypoint': state.pending_requested_waypoint,
                'active_token': token_value(state.active_token),
                'last_acknowledged_token': token_value(
                    state.last_acknowledged_token
                ),
                'last_acknowledged_waypoint': state.last_acknowledged_waypoint,
                'last_acknowledged_target_epoch': state.last_acknowledged_target_epoch,
                'acknowledgement_count': state.acknowledgement_count,
                'last_acknowledgement_monotonic_sec': state.last_acknowledged_at,
                'last_acknowledgement_age_sec': (
                    None if state.last_acknowledged_at is None else
                    max(0.0, now - state.last_acknowledged_at)
                ),
                'unmatched_acknowledgement_count': state.unmatched_acknowledgement_count,
                'handoff_cause': state.handoff_cause,
                'suppression_reason': state.suppression_reason,
                'suppression_count': state.suppression_count,
                'last_failed_token': token_value(state.last_failed_token),
                'last_failed_waypoint': state.last_failed_waypoint,
                'failure_count': state.failure_count,
                'unmatched_failure_count': state.unmatched_failure_count,
                'active_target_epoch': state.active_target_epoch,
                'pending_target_epoch': state.pending_target_epoch,
                'active_objective_revision': state.active_objective_revision,
                'pending_objective_revision': state.pending_objective_revision,
                'active_command_revision': state.active_command_revision,
                'pending_command_revision': state.pending_command_revision,
                'pending_supersede': state.pending_supersede,
                'retained_due_to_deadband': state.retained_due_to_deadband,
                'deadband_retention_count': state.deadband_retention_count,
                'superseded_token_count': state.superseded_token_count,
                'superseded_acknowledgement_count': (
                    state.superseded_acknowledgement_count
                ),
                'last_superseded_token': token_value(
                    state.last_superseded_token
                ),
                'last_superseded_by_token': token_value(
                    state.last_superseded_by_token
                ),
                'active_waypoint_age_sec': active_age,
                'last_publication_monotonic_sec': state.last_published_at,
                'last_publication_age_sec': publication_age,
                'refresh_count': state.refresh_count,
                'active_waypoint_overdue': (
                    active_age is not None and
                    active_age > self.config.active_waypoint_warning_sec
                ),
            }
        return output

    def mark_fault(self, reason):
        self.faulted = True
        self.fault_reason = str(reason)

    def stop(self):
        for state in self.states.values():
            state.active_waypoint = None
            state.active_requested_waypoint = None
            state.active_token = None
            state.pending_waypoint = None
            state.pending_requested_waypoint = None
            state.pending_supersede = False
            state.active_published_at = None
            state.last_published_at = None
            state.refresh_count = 0
            state.superseded_tokens.clear()
