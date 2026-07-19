"""Conservative pre-dispatch safety validation."""

import math
from itertools import combinations


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
    if snapshot.frame_id != 'robotics_lab':
        raise SafetyViolation('Snapshot frame is not robotics_lab.')
    if now - result.created_at > config.safety.controller_result_timeout_sec:
        raise SafetyViolation('Controller result is stale.')
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

    current_distance, current_first, current_second = _minimum_pairwise_separation(
        {
            robot_id: snapshot.robots[robot_id].position
            for robot_id in snapshot.robots
        }
    )
    if current_distance < config.safety.minimum_separation_m:
        raise SafetyViolation(
            'Emergency current separation between %s and %s is %.3f m, '
            'below %.3f m.'
            % (
                current_first,
                current_second,
                current_distance,
                config.safety.minimum_separation_m,
            )
        )
    proposed_distance, proposed_first, proposed_second = (
        _minimum_pairwise_separation(result.setpoints)
    )
    if proposed_distance < config.safety.minimum_separation_m:
        raise SafetyViolation(
            'Immediate predicted separation between %s and %s is %.3f m, '
            'below %.3f m.'
            % (
                proposed_first,
                proposed_second,
                proposed_distance,
                config.safety.minimum_separation_m,
            )
        )

    paths = result.predicted_paths
    if paths:
        for robot_id in sorted(snapshot.robots):
            path = paths.get(robot_id)
            if path is None or not path:
                raise SafetyViolation('Missing predicted path for %s.' % robot_id)
            if math.dist(
                path[0], snapshot.robots[robot_id].position
            ) > 1e-9:
                raise SafetyViolation(
                    'Predicted path for %s does not start at its measured position.'
                    % robot_id
                )
            if len(path) >= 2 and math.dist(
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
            separation, first_id, second_id = _minimum_pairwise_separation(
                positions
            )
            if separation < config.safety.minimum_separation_m:
                raise SafetyViolation(
                    'Predicted separation between %s and %s at path step %d '
                    'is %.3f m, below %.3f m.'
                    % (
                        first_id,
                        second_id,
                        step,
                        separation,
                        config.safety.minimum_separation_m,
                    )
                )
    valid_nodes = expected | {snapshot.station.station_id}
    for edge in result.selected_edges:
        if edge[0] not in valid_nodes or edge[1] not in valid_nodes:
            raise SafetyViolation('Selected edge references an unknown node.')
    return True
