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
            active_age = now - state.active_published_at
            if active_age > self.config.maximum_active_time_sec:
                self.faulted = True
                self.fault_reason = (
                    'Active waypoint for %s timed out after %.3f s.'
                    % (robot_id, active_age)
                )
                actions.append(DispatchAction(
                    'fault', robot_id, reason=self.fault_reason
                ))
                break
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
            else:
                state.reached_since = None
        return actions

    def mark_fault(self, reason):
        self.faulted = True
        self.fault_reason = str(reason)

    def rearm(self):
        self.faulted = False
        self.fault_reason = ''
        for state in self.states.values():
            state.active_waypoint = None
            state.pending_waypoint = None
            state.active_published_at = None
            state.reached_since = None

    def stop(self):
        for state in self.states.values():
            state.active_waypoint = None
            state.pending_waypoint = None
            state.active_published_at = None
            state.reached_since = None
