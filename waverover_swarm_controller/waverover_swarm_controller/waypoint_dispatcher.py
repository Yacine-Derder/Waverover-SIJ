"""Event-triggered dispatcher that respects the onboard FIFO."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DispatchAction:
    kind: str
    robot_id: str
    point: tuple = None
    reason: str = ''


@dataclass
class RoverDispatchState:
    active_waypoint: tuple = None
    pending_waypoint: tuple = None
    active_published_at: float = None
    last_published_at: float = None
    refresh_count: int = 0
    reached_since: float = None
    ever_commanded: bool = False


class WaypointDispatcher:
    def __init__(self, robot_ids, config):
        self.config = config
        self.states = {
            str(robot_id): RoverDispatchState()
            for robot_id in sorted(robot_ids)
        }
        self.faulted = False
        self.fault_reason = ''

    @property
    def commanded_robot_ids(self):
        return tuple(sorted(
            robot_id
            for robot_id, state in self.states.items()
            if state.ever_commanded
        ))

    def update_pending(self, setpoints):
        for robot_id, point in setpoints.items():
            if robot_id not in self.states:
                raise ValueError('Unknown dispatcher robot %s.' % robot_id)
            self.states[robot_id].pending_waypoint = (
                float(point[0]),
                float(point[1]),
            )

    def _publish_pending(self, robot_id, state, now):
        point = state.pending_waypoint
        state.active_waypoint = point
        state.pending_waypoint = None
        state.active_published_at = now
        state.last_published_at = now
        state.refresh_count = 0
        state.reached_since = None
        state.ever_commanded = True
        return DispatchAction('waypoint', robot_id, point=point)

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
            robot = snapshot.robots[robot_id]
            distance = math.dist(robot.position, state.active_waypoint)
            if distance <= self.config.reached_distance_m:
                if state.reached_since is None:
                    state.reached_since = now
                if (
                    state.pending_waypoint is not None
                    and now - state.reached_since >= self.config.handoff_delay_sec
                ):
                    actions.append(self._publish_pending(robot_id, state, now))
                    continue
            else:
                state.reached_since = None
            if now - state.last_published_at >= self.config.refresh_period_sec:
                state.last_published_at = now
                state.refresh_count += 1
                actions.append(DispatchAction(
                    'refresh', robot_id, point=state.active_waypoint
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
            state.pending_waypoint = None
            state.active_published_at = None
            state.last_published_at = None
            state.refresh_count = 0
            state.reached_since = None
