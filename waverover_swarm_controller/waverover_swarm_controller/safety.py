"""Conservative pre-dispatch safety validation."""

import math

from .metrics import minimum_pairwise_distance


class SafetyViolation(RuntimeError):
    """Raised when a result must stop rather than command the swarm."""


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

    current_distance = minimum_pairwise_distance(
        state.position for state in snapshot.robots.values()
    )
    if current_distance < config.safety.minimum_separation_m:
        raise SafetyViolation(
            'Emergency current separation %.3f m is below %.3f m.'
            % (current_distance, config.safety.minimum_separation_m)
        )
    proposed_distance = minimum_pairwise_distance(result.setpoints.values())
    if proposed_distance < config.safety.minimum_separation_m:
        raise SafetyViolation(
            'Immediate predicted separation %.3f m is below %.3f m.'
            % (proposed_distance, config.safety.minimum_separation_m)
        )

    paths = result.predicted_paths
    if paths:
        maximum_length = max((len(path) for path in paths.values()), default=0)
        for step in range(maximum_length):
            positions = []
            for robot_id in sorted(snapshot.robots):
                path = paths.get(robot_id)
                if path is None or not path:
                    raise SafetyViolation(
                        'Missing predicted path for %s.' % robot_id
                    )
                point = path[min(step, len(path) - 1)]
                if not config.safety.geofence.contains(point):
                    raise SafetyViolation(
                        'Predicted path for %s leaves geofence.' % robot_id
                    )
                positions.append(point)
            separation = minimum_pairwise_distance(positions)
            if separation < config.safety.minimum_separation_m:
                raise SafetyViolation(
                    'Predicted separation violation at path step %d.' % step
                )
    valid_nodes = expected | {snapshot.station.station_id}
    for edge in result.selected_edges:
        if edge[0] not in valid_nodes or edge[1] not in valid_nodes:
            raise SafetyViolation('Selected edge references an unknown node.')
    return True
