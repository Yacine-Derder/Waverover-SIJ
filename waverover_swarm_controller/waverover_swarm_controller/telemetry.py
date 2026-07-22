"""Canonical machine-readable controller telemetry construction."""

import json
import math

from .metrics import (
    algebraic_connectivity,
    minimum_pairwise_with_ids,
    weighted_algebraic_connectivity,
)


CONTROLLER_TELEMETRY_SCHEMA_VERSION = 7


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
    execution_outcome=None,
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
            'is_priority': bool(target.is_priority),
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
        'controller_mode': (
            execution_outcome.controller_mode
            if execution_outcome is not None else (
                result.optimization_mode if result is not None else ''
            )
        ),
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
        'priority_target_id': (
            snapshot.priority_target_id if snapshot is not None else None
        ),
        'target_epoch': snapshot.target_epoch if snapshot is not None else None,
        'target_epoch_start_monotonic_sec': (
            snapshot.target_epoch_started_at if snapshot is not None else None
        ),
        'target_epoch_elapsed_sec': (
            max(0.0, snapshot.created_at - snapshot.target_epoch_started_at)
            if snapshot is not None else None
        ),
        'next_target_switch_monotonic_sec': (
            snapshot.next_target_switch_at
            if snapshot is not None else None
        ),
        'target_switch_remaining_sec': (
            max(0.0, snapshot.next_target_switch_at - snapshot.created_at)
            if snapshot is not None
            and snapshot.next_target_switch_at is not None else None
        ),
        'target_selection_seed': (
            snapshot.target_selection_seed if snapshot is not None
            else int(config.target_dynamics.seed)
        ),
        'target_switch_reason': (
            snapshot.target_switch_reason if snapshot is not None else None
        ),
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
                'active_token': values.get('active_token'),
                'last_acknowledged_token': values.get(
                    'last_acknowledged_token'
                ),
                'last_acknowledged_waypoint': (
                    None if values.get('last_acknowledged_waypoint') is None
                    else _point(values['last_acknowledged_waypoint'])
                ),
                'acknowledgement_count': int(values.get(
                    'acknowledgement_count', 0
                )),
                'last_acknowledgement_monotonic_sec': values.get(
                    'last_acknowledgement_monotonic_sec'
                ),
                'last_acknowledgement_age_sec': values.get(
                    'last_acknowledgement_age_sec'
                ),
                'unmatched_acknowledgement_count': int(values.get(
                    'unmatched_acknowledgement_count', 0
                )),
                'handoff_cause': values.get('handoff_cause', 'none'),
                'suppression_reason': values.get('suppression_reason', ''),
                'suppression_count': int(values.get('suppression_count', 0)),
                'last_failed_token': values.get('last_failed_token'),
                'last_failed_waypoint': (
                    None if values.get('last_failed_waypoint') is None else
                    _point(values['last_failed_waypoint'])
                ),
                'failure_count': int(values.get('failure_count', 0)),
                'unmatched_failure_count': int(values.get(
                    'unmatched_failure_count', 0
                )),
                'active_target_epoch': int(values.get(
                    'active_target_epoch', 0
                )),
                'pending_target_epoch': int(values.get(
                    'pending_target_epoch', 0
                )),
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
        'optimization_mode': (
            result.optimization_mode if result is not None else ''
        ),
        'normal_solver_status': (
            execution_outcome.failure_metadata.get('normal_solver_status')
            if execution_outcome is not None else None
        ),
        'recovery_solver_status': (
            execution_outcome.failure_metadata.get('recovery_solver_status')
            if execution_outcome is not None else None
        ),
        'normal_failure_reason': (
            execution_outcome.failure_metadata.get(
                'normal_failure_reason', ''
            ) if execution_outcome is not None else ''
        ),
        'recovery_failure_reason': (
            execution_outcome.failure_metadata.get(
                'recovery_failure_reason', ''
            ) if execution_outcome is not None else ''
        ),
        'maximum_connectivity_slack_m': (
            execution_outcome.failure_metadata.get(
                'maximum_connectivity_slack_m', 0.0
            ) if execution_outcome is not None else 0.0
        ),
        'total_connectivity_slack_m': (
            execution_outcome.failure_metadata.get(
                'total_connectivity_slack_m', 0.0
            ) if execution_outcome is not None else 0.0
        ),
        'consecutive_recovery_cycles': (
            execution_outcome.consecutive_recovery_cycles
            if execution_outcome is not None else 0
        ),
        'fallback_counters': (
            dict(execution_outcome.fallback_counters)
            if execution_outcome is not None else {}
        ),
        'complete_command_set_generated': (
            execution_outcome.complete_command_set_generated
            if execution_outcome is not None else result is not None
        ),
        'final_command_set_passed_validation': (
            execution_outcome.final_command_set_passed_validation
            if execution_outcome is not None else result_state == 'valid'
        ),
        'dispatch_allowed': (
            execution_outcome.dispatch_allowed
            if execution_outcome is not None else bool(commands_enabled)
        ),
        'controller_exception': (
            execution_outcome.failure_metadata.get('controller_exception')
            if execution_outcome is not None else None
        ),
        'distributed_local_solver_statuses': (
            execution_outcome.failure_metadata.get(
                'distributed_local_solver_statuses', {}
            ) if execution_outcome is not None else {}
        ),
        'controller_diagnostics': (
            dict(result.controller_diagnostics) if result is not None else {}
        ),
        'collision_policy': config.safety.collision_policy,
        'preferred_separation_m': config.safety.preferred_separation_m,
        'waypoint_separation_repair': (
            dict(result.collision_repair) if result is not None else {}
        ),
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
