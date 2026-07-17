"""Validation and synchronized aggregation of MCS poses."""

import math
import time

from .models import RobotState, SwarmSnapshot


class SnapshotUnavailableError(RuntimeError):
    """Raised when a complete, fresh, synchronized snapshot is unavailable."""


class PoseAggregator:
    def __init__(
        self,
        robot_ids,
        frame_id,
        timeout_sec,
        maximum_snapshot_skew_sec,
        logger=None,
        monotonic_clock=time.monotonic,
    ):
        self.robot_ids = tuple(sorted(str(value) for value in robot_ids))
        self.frame_id = frame_id
        self.timeout_sec = float(timeout_sec)
        self.maximum_snapshot_skew_sec = float(maximum_snapshot_skew_sec)
        self.logger = logger
        self.monotonic_clock = monotonic_clock
        self.latest = {}
        self._last_warning_at = {}

    def _warn(self, key, message, period_sec=2.0):
        now = self.monotonic_clock()
        previous = self._last_warning_at.get(key)
        if previous is not None and now - previous < period_sec:
            return
        self._last_warning_at[key] = now
        if self.logger is not None:
            self.logger.warn(message)

    def receive(self, robot_id, message, receipt_time=None):
        robot_id = str(robot_id)
        if robot_id not in self.robot_ids:
            self._warn('unknown:' + robot_id, 'Ignored pose for unknown robot %s.' % robot_id)
            return False
        frame_id = str(message.header.frame_id).strip()
        if frame_id != self.frame_id:
            self._warn(
                'frame:' + robot_id,
                'Rejected %s pose frame_id="%s"; expected "%s".'
                % (robot_id, frame_id, self.frame_id),
            )
            return False

        position = message.pose.position
        orientation = message.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        try:
            values = tuple(float(value) for value in values)
        except (TypeError, ValueError):
            values = ()
        if len(values) != 7 or not all(math.isfinite(value) for value in values):
            self._warn(
                'finite:' + robot_id,
                'Rejected %s pose containing non-finite values.' % robot_id,
            )
            return False
        quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
        if quaternion_norm <= 1e-12:
            self._warn(
                'quaternion:' + robot_id,
                'Rejected %s pose with a zero-length quaternion.' % robot_id,
            )
            return False
        qx, qy, qz, qw = (
            value / quaternion_norm for value in values[3:]
        )
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        stamp = message.header.stamp
        message_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if not math.isfinite(message_time) or message_time <= 0.0:
            message_time = None
        received_at = (
            self.monotonic_clock() if receipt_time is None else float(receipt_time)
        )
        self.latest[robot_id] = RobotState(
            robot_id=robot_id,
            x=values[0],
            y=values[1],
            yaw=yaw,
            receipt_time=received_at,
            message_time=message_time,
        )
        return True

    def snapshot(self, targets, station, now=None):
        current_time = self.monotonic_clock() if now is None else float(now)
        missing = [value for value in self.robot_ids if value not in self.latest]
        if missing:
            raise SnapshotUnavailableError(
                'Missing poses for: %s.' % ', '.join(missing)
            )
        states = {robot_id: self.latest[robot_id] for robot_id in self.robot_ids}
        stale = [
            robot_id
            for robot_id, state in states.items()
            if current_time - state.receipt_time > self.timeout_sec
            or current_time < state.receipt_time
        ]
        if stale:
            raise SnapshotUnavailableError(
                'Stale poses for: %s.' % ', '.join(stale)
            )
        receipt_times = [state.receipt_time for state in states.values()]
        skew = max(receipt_times) - min(receipt_times)
        if skew > self.maximum_snapshot_skew_sec:
            raise SnapshotUnavailableError(
                'Pose snapshot skew %.3f s exceeds %.3f s.'
                % (skew, self.maximum_snapshot_skew_sec)
            )
        return SwarmSnapshot(
            frame_id=self.frame_id,
            robots=states,
            targets={target.target_id: target for target in targets},
            station=station,
            created_at=current_time,
        )

    def pose_ages(self, now=None):
        current_time = self.monotonic_clock() if now is None else float(now)
        return {
            robot_id: (
                None
                if robot_id not in self.latest
                else current_time - self.latest[robot_id].receipt_time
            )
            for robot_id in self.robot_ids
        }
