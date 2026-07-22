"""Conservative pre-dispatch safety validation."""

from itertools import combinations
import math


class SafetyViolation(RuntimeError):
    """Raised when a result must stop rather than command the swarm."""


def _minimum_pairwise_separation(points_by_id):
    """Return distance and deterministically ordered IDs for the closest pair."""
    candidates = (
        (
            math.dist(points_by_id[first_id], points_by_id[second_id]),
            first_id,
            second_id,
        )
        for first_id, second_id in combinations(sorted(points_by_id), 2)
    )
    return min(candidates, default=(math.inf, None, None))


def validate_controller_result(config, snapshot, result, now):
    from .controllers.base import controller_schedule, ControllerSchedule
    if snapshot.frame_id != 'robotics_lab':
        raise SafetyViolation('Snapshot frame is not robotics_lab.')
    if now - result.created_at > config.safety.controller_result_timeout_sec:
        raise SafetyViolation('Controller result is stale.')
    if int(result.target_epoch) != int(snapshot.target_epoch):
        raise SafetyViolation(
            'Controller result target epoch %d does not match snapshot epoch %d.'
            % (result.target_epoch, snapshot.target_epoch)
        )
    expected = set(snapshot.robots)
    received = set(result.setpoints)
    if received != expected:
        missing = sorted(expected - received)
        extra = sorted(received - expected)
        raise SafetyViolation(
            'Controller outputs mismatch; missing=%s extra=%s.'
            % (missing, extra)
        )
    for robot_id, point in result.setpoints.items():
        if len(point) != 2 or not all(math.isfinite(float(value)) for value in point):
            raise SafetyViolation('Non-finite setpoint for %s.' % robot_id)
        if not config.safety.geofence.contains(point):
            raise SafetyViolation('Setpoint for %s violates geofence.' % robot_id)

    paths = result.predicted_paths
    if paths:
        receding = controller_schedule(
            config.controller.algorithm
        ) is ControllerSchedule.RECEDING_HORIZON
        for robot_id in sorted(snapshot.robots):
            path = paths.get(robot_id)
            if path is None or not path:
                raise SafetyViolation('Missing predicted path for %s.' % robot_id)
            if receding and math.dist(
                path[0], snapshot.robots[robot_id].position
            ) > 1e-9:
                raise SafetyViolation(
                    'Predicted path for %s does not start at its measured position.'
                    % robot_id
                )
            if receding and len(path) >= 2 and math.dist(
                path[1], result.setpoints[robot_id]
            ) > 1e-9:
                raise SafetyViolation(
                    'Predicted path step 1 for %s does not match its setpoint.'
                    % robot_id
                )
        maximum_length = max((len(path) for path in paths.values()), default=0)
        for step in range(maximum_length):
            positions = {}
            for robot_id in sorted(snapshot.robots):
                path = paths.get(robot_id)
                point = path[min(step, len(path) - 1)]
                if not config.safety.geofence.contains(point):
                    raise SafetyViolation(
                        'Predicted path for %s leaves geofence.' % robot_id
                    )
                positions[robot_id] = point
    valid_nodes = expected | {snapshot.station.station_id}
    for edge in result.selected_edges:
        if edge[0] not in valid_nodes or edge[1] not in valid_nodes:
            raise SafetyViolation('Selected edge references an unknown node.')
    return True
