"""Generate machine-readable and plotted metrics for one recorded run."""

import argparse
import csv
from itertools import combinations
import json
import math
from pathlib import Path

import numpy as np
import yaml

from .analysis_metrics import (
    descriptive_statistics,
    graph_metrics,
    integrate_series,
    mission_cost,
    priority_target_distances,
    separation_metrics,
)
from .offline_data import load_run_data


def _relative_time(data, timestamp):
    return max(0.0, float(timestamp) - data['start_time'])


def _pose_series(data, topic):
    output = []
    for sample in data['samples'].get(topic, []):
        pose = sample['message'].pose
        quaternion = pose.orientation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
        )
        output.append({
            'time_sec': _relative_time(data, sample['timestamp_sec']),
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'yaw': yaw,
        })
    return output


def compute_analysis(data):
    config = data['config']
    rows = []
    exact_cost_flags = []
    previous_edges = None
    edge_changes = 0
    previous_graph_edges = None
    graph_edge_changes = 0
    for sample in data['telemetry']:
        elapsed = _relative_time(data, sample['_timestamp_sec'])
        has_controller_result = (
            sample.get('result_state') in ('valid', 'rejected')
            and sample.get('solver_status') is not None
        )
        if has_controller_result:
            communication, target, total, exact = mission_cost(
                sample, config.communication.ideal_range_m
            )
            exact_cost_flags.append(exact)
        else:
            communication = target = total = None
            exact = None
        main_exact, main_proxy = priority_target_distances(sample)
        graph = graph_metrics(sample, config) or {}
        separation, pair = separation_metrics(sample)
        predicted = sample.get('predicted_minimum_separation', {})
        tracking_errors = [
            math.dist(
                robot_value['position'], sample['setpoints'][robot_id]
            )
            for robot_id, robot_value in sample.get('robots', {}).items()
            if robot_id in sample.get('setpoints', {})
        ]
        edges = tuple(tuple(edge) for edge in sample.get('selected_edges', []))
        if previous_edges is not None and edges != previous_edges:
            edge_changes += 1
        previous_edges = edges
        graph_nodes = {
            robot_id: value['position']
            for robot_id, value in sample.get('robots', {}).items()
        }
        station = sample.get('station')
        if station:
            graph_nodes[station['id']] = station['position']
        graph_edges = tuple(
            (first, second)
            for first, second in combinations(sorted(graph_nodes), 2)
            if math.dist(graph_nodes[first], graph_nodes[second])
            <= config.communication.maximum_range_m
        )
        if previous_graph_edges is not None and graph_edges != previous_graph_edges:
            graph_edge_changes += 1
        previous_graph_edges = graph_edges
        rows.append({
            'elapsed_sec': elapsed,
            'priority_target_id': sample.get('priority_target_id'),
            'target_epoch': sample.get('target_epoch', 0),
            'result_state': sample.get('result_state'),
            'solver_status': sample.get('solver_status'),
            'solve_duration_sec': sample.get('solve_duration_sec'),
            'communication_cost': communication,
            'target_distance_cost': target,
            'mission_cost_total': total,
            'mission_cost_exact': exact,
            'priority_target_distance_exact_m': main_exact,
            'priority_target_distance_proxy_m': main_proxy,
            'binary_lambda_2': graph.get('binary_lambda_2'),
            'weighted_lambda_2': graph.get('weighted_lambda_2'),
            'connected_components': graph.get('connected_components'),
            'station_reachable_rovers': graph.get('station_reachable_rovers'),
            'current_minimum_separation_m': separation,
            'current_minimum_pair': pair,
            'predicted_minimum_separation_m': predicted.get('distance_m'),
            'predicted_minimum_pair': predicted.get('pair'),
            'predicted_minimum_step': predicted.get('step'),
            'snapshot_skew_sec': sample.get('snapshot_skew_sec'),
            'setpoint_tracking_error_m': (
                float(np.mean(tracking_errors)) if tracking_errors else None
            ),
            'stop_reason': sample.get('stop_reason', ''),
            'target_assignments': sample.get('target_assignments'),
            'separation_repair_residual_m': sample.get(
                'waypoint_separation_repair', {}
            ).get('residual_violation_m'),
            'separation_repair_fallback': bool(sample.get(
                'waypoint_separation_repair', {}
            ).get('least_violating_fallback', False)),
        })
    times = [row['elapsed_sec'] for row in rows]
    cost_values = [row['mission_cost_total'] for row in rows]
    lambda_values = [row['binary_lambda_2'] for row in rows]
    separation_values = [row['current_minimum_separation_m'] for row in rows]
    solve_values = [row['solve_duration_sec'] for row in rows]
    deadline_misses = sum(
        value is not None
        and value > config.controller.control_period_sec
        for value in solve_values
    )
    violation_count = sum(
        value is not None and value < config.safety.minimum_separation_m
        for value in separation_values
    )
    violation_duration = sum(
        max(0.0, second['elapsed_sec'] - first['elapsed_sec'])
        for first, second in zip(rows, rows[1:])
        if first['current_minimum_separation_m'] is not None
        and first['current_minimum_separation_m']
        < config.safety.minimum_separation_m
    )
    outage_count = 0
    in_outage = False
    outage_durations = []
    outage_started = None
    for row in rows:
        disconnected = (
            row['binary_lambda_2'] is not None
            and row['binary_lambda_2'] <= 1e-9
        )
        if disconnected and not in_outage:
            outage_count += 1
            outage_started = row['elapsed_sec']
        if not disconnected and in_outage and outage_started is not None:
            outage_durations.append(row['elapsed_sec'] - outage_started)
        in_outage = disconnected
    if in_outage and outage_started is not None and rows:
        outage_durations.append(rows[-1]['elapsed_sec'] - outage_started)

    pose_paths = {}
    ground_truth_paths = {}
    motion_samples = {}
    path_lengths = {}
    pose_rates = {}
    forward_fractions = {}
    measured_speeds = []
    measured_yaw_rates = []
    observation_position_errors = []
    observation_heading_errors = []
    realized_speeds = []
    realized_yaw_rates = []
    speed_statistics_by_rover = {}
    yaw_rate_statistics_by_rover = {}
    measured_speed_statistics_by_rover = {}
    measured_yaw_rate_statistics_by_rover = {}
    for robot_id in config.robot_ids:
        pose_topic = '/macortex_bridge/waverover_%s/pose' % robot_id
        truth_topic = (
            '/waverover_swarm/synthetic/ground_truth/waverover_%s' % robot_id
        )
        motion_topic = '/waverover_swarm/synthetic/motion/waverover_%s' % robot_id
        pose_paths[robot_id] = _pose_series(data, pose_topic)
        ground_truth_paths[robot_id] = _pose_series(data, truth_topic)
        motion_samples[robot_id] = data['samples'].get(motion_topic, [])
        path_lengths[robot_id] = sum(
            math.hypot(second['x'] - first['x'], second['y'] - first['y'])
            for first, second in zip(
                pose_paths[robot_id], pose_paths[robot_id][1:]
            )
        )
        duration = (
            pose_paths[robot_id][-1]['time_sec'] - pose_paths[robot_id][0]['time_sec']
            if len(pose_paths[robot_id]) >= 2 else 0.0
        )
        pose_rates[robot_id] = (
            (len(pose_paths[robot_id]) - 1) / duration if duration > 0.0 else None
        )
        speeds = [sample['message'].twist.linear.x for sample in motion_samples[robot_id]]
        yaw_rates = [
            sample['message'].twist.angular.z
            for sample in motion_samples[robot_id]
        ]
        realized_speeds.extend(speeds)
        realized_yaw_rates.extend(yaw_rates)
        speed_statistics_by_rover[robot_id] = descriptive_statistics(speeds)
        yaw_rate_statistics_by_rover[robot_id] = descriptive_statistics(yaw_rates)
        forward_fractions[robot_id] = (
            sum(speed > 0.0 for speed in speeds) / len(speeds) if speeds else None
        )
        rover_measured_speeds = []
        rover_measured_yaw_rates = []
        for first, second in zip(
            pose_paths[robot_id], pose_paths[robot_id][1:]
        ):
            delta_time = second['time_sec'] - first['time_sec']
            if delta_time > 0.0:
                rover_measured_speeds.append(math.hypot(
                    second['x'] - first['x'], second['y'] - first['y']
                ) / delta_time)
                yaw_delta = (
                    second['yaw'] - first['yaw'] + math.pi
                ) % (2.0 * math.pi) - math.pi
                rover_measured_yaw_rates.append(yaw_delta / delta_time)
        measured_speeds.extend(rover_measured_speeds)
        measured_yaw_rates.extend(rover_measured_yaw_rates)
        measured_speed_statistics_by_rover[robot_id] = descriptive_statistics(
            rover_measured_speeds
        )
        measured_yaw_rate_statistics_by_rover[robot_id] = descriptive_statistics(
            rover_measured_yaw_rates
        )
        for observed, truth in zip(
            pose_paths[robot_id], ground_truth_paths[robot_id]
        ):
            if abs(observed['time_sec'] - truth['time_sec']) <= (
                config.analysis.maximum_interpolation_gap_sec
            ):
                observation_position_errors.append(math.hypot(
                    observed['x'] - truth['x'], observed['y'] - truth['y']
                ))
                observation_heading_errors.append(abs((
                    observed['yaw'] - truth['yaw'] + math.pi
                ) % (2.0 * math.pi) - math.pi))

    pairwise_distances = []
    selected_link_margins = []
    geofence_violations = 0
    target_occupancy = {}
    for sample in data['telemetry']:
        robot_points = {
            robot_id: value['position']
            for robot_id, value in sample.get('robots', {}).items()
        }
        for first, second in combinations(sorted(robot_points), 2):
            pairwise_distances.append(math.dist(
                robot_points[first], robot_points[second]
            ))
        geofence_violations += sum(
            not config.safety.geofence.contains(point)
            for point in robot_points.values()
        )
        nodes = dict(robot_points)
        station = sample.get('station')
        if station:
            nodes[station['id']] = station['position']
        for first, second in sample.get('selected_edges', []):
            if first in nodes and second in nodes:
                selected_link_margins.append(
                    config.communication.maximum_range_m
                    - math.dist(nodes[first], nodes[second])
                )
        for target_id in (sample.get('target_assignments') or {}).values():
            target_occupancy[target_id] = target_occupancy.get(target_id, 0) + 1

    coverage_threshold = config.controller.minimum_mpc_lookahead_m
    convergence_time = next((
        row['elapsed_sec'] for row in rows
        if row['priority_target_distance_proxy_m'] is not None
        and row['priority_target_distance_proxy_m'] <= coverage_threshold
    ), None)
    coverage_duration = sum(
        max(0.0, second['elapsed_sec'] - first['elapsed_sec'])
        for first, second in zip(rows, rows[1:])
        if first['priority_target_distance_proxy_m'] is not None
        and first['priority_target_distance_proxy_m'] <= coverage_threshold
    )
    speed_references = (
        config.vehicle.straight_speed_mps,
        config.vehicle.turning_path_speed_mps,
    )
    yaw_references = (
        0.0,
        config.vehicle.bank_yaw_rate_rad_s,
        -config.vehicle.bank_yaw_rate_rad_s,
    )
    speed_adherence_errors = [
        min(abs(speed - reference) for reference in speed_references)
        for speed in realized_speeds if speed > 0.0
    ]
    yaw_adherence_errors = [
        min(abs(rate - reference) for reference in yaw_references)
        for rate in realized_yaw_rates
    ]

    epoch_groups = {}
    for row in rows:
        epoch_groups.setdefault(row['target_epoch'], []).append(row)
    target_epochs = []
    for epoch, epoch_rows in sorted(epoch_groups.items()):
        start = epoch_rows[0]['elapsed_sec']
        end = epoch_rows[-1]['elapsed_sec']
        response = next((
            row['elapsed_sec'] - start for row in epoch_rows
            if row['priority_target_distance_proxy_m'] is not None
            and row['priority_target_distance_proxy_m'] <= coverage_threshold
        ), None)
        occupancy = {}
        for row in epoch_rows:
            for target_id in (row.get('target_assignments') or {}).values():
                occupancy[target_id] = occupancy.get(target_id, 0) + 1
        target_epochs.append({
            'target_epoch': epoch,
            'priority_target_id': epoch_rows[0]['priority_target_id'],
            'start_elapsed_sec': start,
            'duration_sec': max(0.0, end - start),
            'response_convergence_time_sec': response,
            'priority_target_distance': descriptive_statistics([
                row['priority_target_distance_proxy_m'] for row in epoch_rows
            ]),
            'weighted_mission_cost': descriptive_statistics([
                row['mission_cost_total'] for row in epoch_rows
            ]),
            'target_assignment_occupancy': occupancy,
        })

    waypoint_count = sum(
        len(data['samples'].get('/waverover_%s/waypoints' % robot_id, []))
        for robot_id in config.robot_ids
    )
    acknowledgement_counts = {}
    for event in data.get('acknowledgement_events', []):
        robot_id = event['robot_id']
        acknowledgement_counts[robot_id] = (
            acknowledgement_counts.get(robot_id, 0) + 1
        )
    unmatched_acknowledgements = {}
    for sample in data['telemetry']:
        for robot_id, state in sample.get('waypoint_dispatch', {}).items():
            unmatched_acknowledgements[robot_id] = max(
                unmatched_acknowledgements.get(robot_id, 0),
                int(state.get('unmatched_acknowledgement_count', 0)),
            )
    cmd_topics_present = all(
        '/waverover_%s/cmd_vel' % robot_id in data['topic_types']
        for robot_id in config.robot_ids
    )
    cmd_messages = [
        sample['message']
        for robot_id in config.robot_ids
        for sample in data['samples'].get('/waverover_%s/cmd_vel' % robot_id, [])
    ]

    warnings = []
    required_topics = {
        '/waverover_swarm/controller_telemetry',
        '/waverover_swarm/run_event',
    }
    missing_required = sorted(required_topics - set(data['topic_types']))
    if missing_required:
        warnings.append('Missing topics: ' + ', '.join(missing_required))
    if not rows:
        warnings.append('No controller telemetry; controller metrics unavailable.')
    if not data['used_run_events']:
        warnings.append('Analysis interval uses bag timestamps; BEGIN/END unavailable.')
    missing_recorded = sorted(
        set(data['manifest'].get(
            'requested_topics', data['manifest'].get('recorded_topics', [])
        )) - set(data['topic_types'])
    )
    if missing_recorded:
        warnings.append(
            'Configured topics absent from bag metadata: '
            + ', '.join(missing_recorded)
        )
    synthetic_true_lambda = [
        value.get('current_true_binary_lambda_2')
        for value in data['synthetic_metadata']
    ]
    synthetic_observed_lambda = [
        value.get('current_observed_binary_lambda_2')
        for value in data['synthetic_metadata']
    ]
    synthetic_true_weighted = [
        value.get('current_true_weighted_lambda_2')
        for value in data['synthetic_metadata']
    ]
    synthetic_observed_weighted = [
        value.get('current_observed_weighted_lambda_2')
        for value in data['synthetic_metadata']
    ]
    summary = {
        'schema_version': 2,
        'run_id': data['manifest'].get('run_id'),
        'run_state': data['manifest'].get('state'),
        'algorithm': data['manifest'].get('algorithm'),
        'analysis_interval': {
            'duration_sec': max(0.0, data['end_time'] - data['start_time']),
            'source': 'run_events' if data['used_run_events'] else 'bag_timestamps',
        },
        'mission_cost': {
            'convention': 'Each stored undirected selected edge is counted once.',
            'exact_assignments_available': (
                all(exact_cost_flags) if exact_cost_flags else None
            ),
            'label': (
                'exact' if exact_cost_flags and all(exact_cost_flags)
                else 'nearest-target proxy'
            ),
            'communication': descriptive_statistics(
                [row['communication_cost'] for row in rows]
            ),
            'target_distance': descriptive_statistics(
                [row['target_distance_cost'] for row in rows]
            ),
            'total': descriptive_statistics(cost_values),
            'cumulative_integral': integrate_series(times, cost_values),
        },
        'priority_target_distance': {
            'assigned': descriptive_statistics([
                row['priority_target_distance_exact_m'] for row in rows
            ]),
            'minimum_any_rover_proxy': descriptive_statistics([
                row['priority_target_distance_proxy_m'] for row in rows
            ]),
            'coverage_threshold_m': coverage_threshold,
            'convergence_time_sec': convergence_time,
            'coverage_duration_sec': coverage_duration,
            'target_switch_response': None,
        },
        'target_epochs': target_epochs,
        'connectivity': {
            'binary_lambda_2': descriptive_statistics(lambda_values),
            'weighted_lambda_2': descriptive_statistics([
                row['weighted_lambda_2'] for row in rows
            ]),
            'outage_count': outage_count,
            'total_disconnected_duration_sec': sum(outage_durations),
            'longest_outage_sec': max(outage_durations, default=0.0),
            'selected_edge_changes': edge_changes,
            'communication_graph_changes': graph_edge_changes,
            'connected_components': descriptive_statistics([
                row['connected_components'] for row in rows
            ]),
            'station_reachable_rovers': descriptive_statistics([
                row['station_reachable_rovers'] for row in rows
            ]),
            'synthetic_true_binary_lambda_2': descriptive_statistics(
                synthetic_true_lambda
            ),
            'synthetic_observed_binary_lambda_2': descriptive_statistics(
                synthetic_observed_lambda
            ),
            'synthetic_true_weighted_lambda_2': descriptive_statistics(
                synthetic_true_weighted
            ),
            'synthetic_observed_weighted_lambda_2': descriptive_statistics(
                synthetic_observed_weighted
            ),
        },
        'computation': {
            'duration_sec': descriptive_statistics(solve_values),
            'deadline_sec': config.controller.control_period_sec,
            'deadline_misses': deadline_misses,
            'solver_status_counts': {
                status: sum(row['solver_status'] == status for row in rows)
                for status in sorted({
                    row['solver_status'] for row in rows
                    if row['solver_status'] is not None
                })
            },
        },
        'separation': {
            'current_m': descriptive_statistics(separation_values),
            'predicted_m': descriptive_statistics([
                row['predicted_minimum_separation_m'] for row in rows
            ]),
            'violation_count': violation_count,
            'violation_duration_sec': violation_duration,
            # Kept alongside legacy violation names for report compatibility;
            # these are best-effort preference warnings, not fatal events.
            'preferred_separation_warning_count': violation_count,
            'repair_residual_m': descriptive_statistics([
                row['separation_repair_residual_m'] for row in rows
            ]),
            'least_violating_fallback_count': sum(
                row['separation_repair_fallback'] for row in rows
            ),
            'pairwise_distance_m': descriptive_statistics(pairwise_distances),
            'offending_pairs': sorted({
                tuple(row['current_minimum_pair'])
                for row in rows
                if row['current_minimum_pair'] is not None
                and row['current_minimum_separation_m']
                < config.safety.minimum_separation_m
            }),
        },
        'motion': {
            'path_length_m_by_rover': path_lengths,
            'mean_path_length_m': (
                float(np.mean(list(path_lengths.values())))
                if path_lengths else None
            ),
            'pose_rate_hz_by_rover': pose_rates,
            'forward_motion_fraction_by_rover': forward_fractions,
            'setpoint_tracking_error_m': descriptive_statistics([
                row['setpoint_tracking_error_m'] for row in rows
            ]),
            'predicted_path_realized_error_m': None,
            'measured_speed_mps': descriptive_statistics(measured_speeds),
            'measured_yaw_rate_rad_s': descriptive_statistics(measured_yaw_rates),
            'synthetic_observation_position_error_m': descriptive_statistics(
                observation_position_errors
            ),
            'synthetic_observation_heading_error_rad': descriptive_statistics(
                observation_heading_errors
            ),
            'realized_forward_speed_mps': descriptive_statistics(realized_speeds),
            'realized_yaw_rate_rad_s': descriptive_statistics(realized_yaw_rates),
            'forward_speed_mps_by_rover': speed_statistics_by_rover,
            'yaw_rate_rad_s_by_rover': yaw_rate_statistics_by_rover,
            'measured_speed_mps_by_rover': measured_speed_statistics_by_rover,
            'measured_yaw_rate_rad_s_by_rover': (
                measured_yaw_rate_statistics_by_rover
            ),
            'primitive_speed_adherence_error_mps': descriptive_statistics(
                speed_adherence_errors
            ),
            'primitive_yaw_rate_adherence_error_rad_s': descriptive_statistics(
                yaw_adherence_errors
            ),
        },
        'engineering': {
            'geofence_violation_count': geofence_violations,
            'selected_link_margin_to_maximum_m': descriptive_statistics(
                selected_link_margins
            ),
            'snapshot_skew_sec': descriptive_statistics([
                row['snapshot_skew_sec'] for row in rows
            ]),
            'waypoint_count': waypoint_count,
            'waypoint_acknowledgement_count_by_rover': acknowledgement_counts,
            'unmatched_acknowledgement_count_by_rover': (
                unmatched_acknowledgements
            ),
            'waypoint_publication_rate_hz': (
                waypoint_count / max(1e-9, data['end_time'] - data['start_time'])
            ),
            'cmd_vel_topic_available': cmd_topics_present,
            'cmd_vel_message_count': len(cmd_messages) if cmd_topics_present else None,
            'cmd_vel_linear_effort': descriptive_statistics([
                abs(message.linear.x) for message in cmd_messages
            ]),
            'cmd_vel_angular_effort': descriptive_statistics([
                abs(message.angular.z) for message in cmd_messages
            ]),
            'waypoint_handoff_latency_sec': None,
            'command_to_motion_delay_sec': None,
            'target_assignment_sample_counts': target_occupancy,
        },
        'result_state_counts': {
            state: sum(row['result_state'] == state for row in rows)
            for state in sorted({
                row['result_state'] for row in rows
                if row['result_state'] is not None
            })
        },
        'warnings': warnings,
    }
    return summary, rows, pose_paths, ground_truth_paths


def _write_csv(path, rows):
    if not rows:
        Path(path).write_text('', encoding='utf-8')
        return
    fields = list(rows[0])
    with Path(path).open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({
                key: json.dumps(value, sort_keys=True)
                if isinstance(value, (dict, list, tuple)) else value
                for key, value in row.items()
            })


def _write_plots(analysis_directory, rows, pose_paths, config):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    times = [row['elapsed_sec'] for row in rows]
    figure, axes = plt.subplots(2, 2, figsize=(12, 8))
    series = (
        ('mission_cost_total', 'Mission cost J'),
        ('priority_target_distance_proxy_m', 'Priority-target distance proxy [m]'),
        ('binary_lambda_2', 'Binary lambda_2'),
        ('current_minimum_separation_m', 'Minimum separation [m]'),
    )
    for axis, (key, title) in zip(axes.flat, series):
        axis.plot(times, [row[key] for row in rows])
        axis.set_title(title)
        axis.set_xlabel('Elapsed time [s]')
        axis.grid(True, alpha=0.3)
    figure.tight_layout()
    figure.savefig(analysis_directory / 'metrics_over_time.png', dpi=150)
    plt.close(figure)

    figure, axes = plt.subplots(2, 2, figsize=(12, 8))
    for axis, (key, title) in zip(axes.flat, series):
        values = [
            row[key] for row in rows
            if row[key] is not None and math.isfinite(float(row[key]))
        ]
        if values:
            axis.hist(values, bins=min(30, max(5, len(values))))
        axis.set_title(title)
        axis.grid(True, alpha=0.3)
    figure.tight_layout()
    figure.savefig(analysis_directory / 'metric_distributions.png', dpi=150)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(8, 8))
    for robot_id, path in pose_paths.items():
        if path:
            axis.plot([row['x'] for row in path], [row['y'] for row in path],
                      label=robot_id)
    axis.scatter([config.station.x], [config.station.y], marker='s', label='station')
    for target in config.targets:
        axis.scatter([target.x], [target.y], marker='x')
        axis.text(target.x, target.y, target.target_id)
    fence = config.safety.geofence
    axis.set_xlim(fence.x_min, fence.x_max)
    axis.set_ylim(fence.y_min, fence.y_max)
    axis.set_aspect('equal')
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(analysis_directory / 'trajectories.png', dpi=150)
    plt.close(figure)


def analyze(run_directory):
    data = load_run_data(run_directory)
    analysis_directory = data['run_directory'] / 'analysis'
    analysis_directory.mkdir(exist_ok=True)
    summary, rows, pose_paths, ground_truth_paths = compute_analysis(data)
    (analysis_directory / 'summary.json').write_text(
        json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8'
    )
    with (analysis_directory / 'summary.yaml').open('w', encoding='utf-8') as stream:
        yaml.safe_dump(summary, stream, sort_keys=False)
    _write_csv(analysis_directory / 'timeseries.csv', rows)
    _write_csv(analysis_directory / 'events.csv', [
        {
            'elapsed_sec': _relative_time(data, event['_timestamp_sec']),
            'event': event.get('event'),
            'detail': event.get('detail', ''),
            'run_id': event.get('run_id'),
        }
        for event in data['events']
    ])
    _write_plots(analysis_directory, rows, pose_paths, data['config'])
    report = [
        '# WaveRover experiment report',
        '',
        '- Run ID: `%s`' % summary['run_id'],
        '- State: `%s`' % summary['run_state'],
        '- Algorithm: `%s`' % summary['algorithm'],
        '- Analysis interval: %.3f s (%s)'
        % (
            summary['analysis_interval']['duration_sec'],
            summary['analysis_interval']['source'],
        ),
        '- Mission-cost label: **%s**' % summary['mission_cost']['label'],
        '',
        'Each selected undirected communication edge is counted once. '
        'Target distance uses stored assignments when available; otherwise it '
        'is explicitly labeled a nearest-target proxy.',
        '',
        'Unavailable quantities are encoded as null rather than zero.',
        '',
        '## Warnings',
        '',
    ]
    report.extend('- ' + warning for warning in summary['warnings'])
    if not summary['warnings']:
        report.append('- None')
    (analysis_directory / 'report.md').write_text(
        '\n'.join(report) + '\n', encoding='utf-8'
    )
    return analysis_directory


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('run_directory')
    arguments = parser.parse_args(args)
    print(analyze(arguments.run_directory))


if __name__ == '__main__':
    main()
