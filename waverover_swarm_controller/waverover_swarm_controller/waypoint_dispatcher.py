"""Event-triggered dispatcher that respects the onboard FIFO."""

from dataclasses import dataclass


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


@dataclass
class RoverDispatchState:
    active_waypoint: tuple = None
    active_token: tuple = None
    active_target_epoch: int = 0
    pending_waypoint: tuple = None
    pending_target_epoch: int = 0
    active_published_at: float = None
    last_published_at: float = None
    refresh_count: int = 0
    ever_commanded: bool = False
    last_acknowledged_token: tuple = None
    last_acknowledged_waypoint: tuple = None
    last_acknowledged_at: float = None
    acknowledgement_count: int = 0
    unmatched_acknowledgement_count: int = 0
    handoff_cause: str = 'none'


class WaypointDispatcher:
    def __init__(self, robot_ids, config):
        self.config = config
        self.states = {
            str(robot_id): RoverDispatchState()
            for robot_id in sorted(robot_ids)
        }
        self.faulted = False
        self.fault_reason = ''
        self._last_token_ns = 0

    @property
    def commanded_robot_ids(self):
        return tuple(sorted(
            robot_id
            for robot_id, state in self.states.items()
            if state.ever_commanded
        ))

    def update_pending(self, setpoints, target_epoch=0):
        for robot_id, point in setpoints.items():
            if robot_id not in self.states:
                raise ValueError('Unknown dispatcher robot %s.' % robot_id)
            state = self.states[robot_id]
            if int(target_epoch) < state.pending_target_epoch:
                continue
            state.pending_waypoint = (
                float(point[0]),
                float(point[1]),
            )
            state.pending_target_epoch = int(target_epoch)

    def _publish_pending(self, robot_id, state, now):
        point = state.pending_waypoint
        state.active_waypoint = point
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
        state.pending_waypoint = None
        state.active_published_at = now
        state.last_published_at = now
        state.refresh_count = 0
        state.handoff_cause = 'initial' if not state.ever_commanded else 'acknowledgement'
        state.ever_commanded = True
        return DispatchAction(
            'waypoint', robot_id, point=point, token=state.active_token,
            target_epoch=state.active_target_epoch,
        )

    def acknowledge(self, robot_id, frame_id, token, point, now, expected_frame):
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
                state.unmatched_acknowledgement_count += 1
            return []
        state.last_acknowledged_token = state.active_token
        state.last_acknowledged_waypoint = state.active_waypoint
        state.last_acknowledged_at = float(now)
        state.acknowledgement_count += 1
        state.active_waypoint = None
        state.active_token = None
        state.active_published_at = None
        state.last_published_at = None
        state.refresh_count = 0
        state.handoff_cause = 'acknowledgement'
        if state.pending_waypoint is None:
            return []
        return [self._publish_pending(str(robot_id), state, now)]

    def tick(self, snapshot, now, commands_enabled):
        if self.faulted or not commands_enabled:
            return []
        actions = []
        for robot_id in sorted(self.states):
            state = self.states[robot_id]
            if state.active_waypoint is None:
                if state.pending_waypoint is not None:
                    actions.append(self._publish_pending(robot_id, state, now))
                continue
            if now - state.last_published_at >= self.config.refresh_period_sec:
                state.last_published_at = now
                state.refresh_count += 1
                actions.append(DispatchAction(
                    'refresh', robot_id, point=state.active_waypoint,
                    token=state.active_token, target_epoch=state.active_target_epoch,
                ))
        return actions

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
                'pending_waypoint': state.pending_waypoint,
                'active_token': token_value(state.active_token),
                'last_acknowledged_token': token_value(
                    state.last_acknowledged_token
                ),
                'last_acknowledged_waypoint': state.last_acknowledged_waypoint,
                'acknowledgement_count': state.acknowledgement_count,
                'last_acknowledgement_monotonic_sec': state.last_acknowledged_at,
                'last_acknowledgement_age_sec': (
                    None if state.last_acknowledged_at is None else
                    max(0.0, now - state.last_acknowledged_at)
                ),
                'unmatched_acknowledgement_count': state.unmatched_acknowledgement_count,
                'handoff_cause': state.handoff_cause,
                'active_target_epoch': state.active_target_epoch,
                'pending_target_epoch': state.pending_target_epoch,
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
            state.active_token = None
            state.pending_waypoint = None
            state.active_published_at = None
            state.last_published_at = None
            state.refresh_count = 0
