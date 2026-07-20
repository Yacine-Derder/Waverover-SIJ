"""Canonical machine-readable controller telemetry construction."""

import json
import math

from .metrics import (
    algebraic_connectivity,
    minimum_pairwise_with_ids,
    weighted_algebraic_connectivity,
)


CONTROLLER_TELEMETRY_SCHEMA_VERSION = 3


def _point(point):
    return [float(point[0]), float(point[1])]


def _finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


def predicted_minimum(result):
    if result is None or not result.predicted_paths:
        return None, None, None, None
    maximum_length = max(
        len(path) for path in result.predicted_paths.values()
    )
    candidates = []
    for step in range(maximum_length):
        points = {
            robot_id: path[min(step, len(path) - 1)]
            for robot_id, path in result.predicted_paths.items()
            if path
        }
        distance, first, second = minimum_pairwise_with_ids(points)
        if first is not None:
            candidates.append((distance, first, second, step))
    return min(candidates, default=(None, None, None, None))


def build_controller_telemetry(
    config,
    snapshot,
    result,
    result_state,
    ros_timestamp,
    commands_enabled,
    stop_reason,
    pose_ages,
    active_waypoints,
    pending_waypoints,
    dispatch_observability,
    latest_handoff,
):
    robots = {}
    current_points = {}
    if snapshot is not None:
        for robot_id, state in snapshot.robots.items():
            robots[robot_id] = {
                'position': _point(state.position),
                'heading_rad': float(state.yaw),
            }
            current_points[robot_id] = state.position
    current_distance, current_first, current_second = (
        minimum_pairwise_with_ids(current_points)
    )
    predicted_distance, predicted_first, predicted_second, predicted_step = (
        predicted_minimum(result)
    )
    node_positions = {}
    if snapshot is not None and snapshot.station is not None:
        node_positions[snapshot.station.station_id] = snapshot.station.position
        node_positions.update(current_points)
    receipt_times = (
        [state.receipt_time for state in snapshot.robots.values()]
        if snapshot is not None else []
    )
    snapshot_skew = (
        max(receipt_times) - min(receipt_times) if receipt_times else None
    )
    targets = {
        target_id: {
            'position': _point(target.position),
            'weight': float(target.weight),
            'is_main': bool(target.is_main),
        }
        for target_id, target in (
            snapshot.targets.items() if snapshot is not None else []
        )
    }
    payload = {
        'schema_version': CONTROLLER_TELEMETRY_SCHEMA_VERSION,
        'ros_timestamp': {
            'sec': int(ros_timestamp.sec),
            'nanosec': int(ros_timestamp.nanosec),
        },
        'algorithm': config.controller.algorithm,
        'result_state': str(result_state),
        'commands_enabled': bool(commands_enabled),
        'dry_run': bool(config.safety.dry_run),
        'robots': robots,
        'station': (
            {
                'id': snapshot.station.station_id,
                'position': _point(snapshot.station.position),
            }
            if snapshot is not None and snapshot.station is not None else None
        ),
        'targets': targets,
        'setpoints': (
            {
                robot_id: _point(point)
                for robot_id, point in result.setpoints.items()
            } if result is not None else {}
        ),
        'active_waypoints': {
            robot_id: None if point is None else _point(point)
            for robot_id, point in sorted(active_waypoints.items())
        },
        'pending_waypoints': {
            robot_id: None if point is None else _point(point)
            for robot_id, point in sorted(pending_waypoints.items())
        },
        'waypoint_dispatch': {
            robot_id: {
                'active_waypoint': (
                    None if values['active_waypoint'] is None else
                    _point(values['active_waypoint'])
                ),
                'pending_waypoint': (
                    None if values['pending_waypoint'] is None else
                    _point(values['pending_waypoint'])
                ),
                'active_waypoint_age_sec': (
                    None if values['active_waypoint_age_sec'] is None else
                    _finite_or_none(values['active_waypoint_age_sec'])
                ),
                'last_publication_monotonic_sec': (
                    None if values['last_publication_monotonic_sec'] is None else
                    _finite_or_none(values['last_publication_monotonic_sec'])
                ),
                'last_publication_age_sec': (
                    None if values['last_publication_age_sec'] is None else
                    _finite_or_none(values['last_publication_age_sec'])
                ),
                'refresh_count': int(values['refresh_count']),
                'active_waypoint_overdue': bool(
                    values['active_waypoint_overdue']
                ),
            }
            for robot_id, values in sorted(dispatch_observability.items())
        },
        'predicted_paths': (
            {
                robot_id: [_point(point) for point in path]
                for robot_id, path in result.predicted_paths.items()
            } if result is not None else {}
        ),
        'selected_edges': (
            [list(edge) for edge in result.selected_edges]
            if result is not None else []
        ),
        'target_assignments': (
            dict(result.target_assignments)
            if result is not None and result.target_assignments else None
        ),
        'solver_status': result.solver_status if result is not None else None,
        'solve_duration_sec': (
            _finite_or_none(result.solve_duration_sec)
            if result is not None else None
        ),
        'stop_reason': str(stop_reason),
        'current_minimum_separation': {
            'distance_m': _finite_or_none(current_distance),
            'pair': (
                [current_first, current_second]
                if current_first is not None else None
            ),
        },
        'predicted_minimum_separation': {
            'distance_m': (
                _finite_or_none(predicted_distance)
                if predicted_distance is not None else None
            ),
            'pair': (
                [predicted_first, predicted_second]
                if predicted_first is not None else None
            ),
            'step': predicted_step,
        },
        'connectivity': {
            'binary_lambda_2': (
                algebraic_connectivity(
                    node_positions, config.communication.maximum_range_m
                ) if node_positions else None
            ),
            'weighted_lambda_2': (
                weighted_algebraic_connectivity(
                    node_positions,
                    config.communication.ideal_range_m,
                    config.communication.maximum_range_m,
                    config.analysis.connectivity_alpha,
                ) if node_positions else None
            ),
        },
        'pose_ages_sec': {
            robot_id: (
                None if age is None else _finite_or_none(age)
            ) for robot_id, age in sorted(pose_ages.items())
        },
        'snapshot_skew_sec': snapshot_skew,
        'latest_waypoint_handoff': str(latest_handoff),
    }
    return payload


def canonical_json(payload):
    return json.dumps(payload, sort_keys=True, separators=(',', ':'))
