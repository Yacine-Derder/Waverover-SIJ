"""Pure metric calculations shared by per-run and comparison tooling."""

import math

import numpy as np

from .metrics import (
    algebraic_connectivity,
    connected_components,
    minimum_pairwise_with_ids,
    weighted_algebraic_connectivity,
)


def interpolate_angle(first, second, fraction):
    delta = (second - first + math.pi) % (2.0 * math.pi) - math.pi
    return (first + fraction * delta + math.pi) % (2.0 * math.pi) - math.pi


def descriptive_statistics(values):
    finite = np.asarray([
        float(value) for value in values
        if value is not None and math.isfinite(float(value))
    ])
    if not len(finite):
        return None
    return {
        'count': int(len(finite)),
        'mean': float(np.mean(finite)),
        'median': float(np.median(finite)),
        'stddev': float(np.std(finite)),
        'minimum': float(np.min(finite)),
        'maximum': float(np.max(finite)),
        'quantiles': {
            'p05': float(np.quantile(finite, 0.05)),
            'p25': float(np.quantile(finite, 0.25)),
            'p75': float(np.quantile(finite, 0.75)),
            'p95': float(np.quantile(finite, 0.95)),
            'p99': float(np.quantile(finite, 0.99)),
        },
    }


def mission_cost(sample, ideal_range):
    robots = {
        robot_id: value['position']
        for robot_id, value in sample.get('robots', {}).items()
    }
    station = sample.get('station')
    nodes = dict(robots)
    if station:
        nodes[station['id']] = station['position']
    communication = 0.0
    for first, second in sample.get('selected_edges', []):
        if first in nodes and second in nodes:
            communication += max(
                ideal_range, math.dist(nodes[first], nodes[second])
            )
    targets = sample.get('targets', {})
    assignments = sample.get('target_assignments')
    exact = assignments is not None
    selected_assignments = {}
    if assignments is not None:
        selected_assignments = assignments
    elif targets:
        selected_assignments = {
            robot_id: min(
                targets,
                key=lambda target_id: math.dist(
                    point, targets[target_id]['position']
                ),
            )
            for robot_id, point in robots.items()
        }
    target_cost = 0.0
    for robot_id, target_id in selected_assignments.items():
        if robot_id in robots and target_id in targets:
            target = targets[target_id]
            target_cost += float(target['weight']) * math.dist(
                robots[robot_id], target['position']
            )
    return communication, target_cost, communication + target_cost, exact


def main_target_distances(sample):
    targets = sample.get('targets', {})
    main = next((
        (target_id, value) for target_id, value in targets.items()
        if value.get('is_main')
    ), None)
    if main is None:
        return None, None
    main_id, main_value = main
    robots = sample.get('robots', {})
    proxy = min((
        math.dist(value['position'], main_value['position'])
        for value in robots.values()
    ), default=None)
    assignments = sample.get('target_assignments')
    if assignments is None:
        return None, proxy
    exact = min((
        math.dist(robots[robot_id]['position'], main_value['position'])
        for robot_id, target_id in assignments.items()
        if target_id == main_id and robot_id in robots
    ), default=None)
    return exact, proxy


def graph_metrics(sample, config):
    points = {
        robot_id: value['position']
        for robot_id, value in sample.get('robots', {}).items()
    }
    station = sample.get('station')
    if station:
        points[station['id']] = station['position']
    if not points:
        return None
    components = connected_components(
        points, config.communication.maximum_range_m
    )
    station_component = next((
        component for component in components
        if station and station['id'] in component
    ), ())
    return {
        'binary_lambda_2': algebraic_connectivity(
            points, config.communication.maximum_range_m
        ),
        'weighted_lambda_2': weighted_algebraic_connectivity(
            points,
            config.communication.ideal_range_m,
            config.communication.maximum_range_m,
            config.analysis.connectivity_alpha,
        ),
        'connected_components': len(components),
        'station_reachable_rovers': max(0, len(station_component) - 1),
    }


def separation_metrics(sample):
    points = {
        robot_id: value['position']
        for robot_id, value in sample.get('robots', {}).items()
    }
    distance, first, second = minimum_pairwise_with_ids(points)
    return (
        distance if first is not None else None,
        [first, second] if first is not None else None,
    )


def integrate_series(times, values):
    total = 0.0
    intervals = 0
    for first_time, second_time, first_value, second_value in zip(
        times, times[1:], values, values[1:]
    ):
        if first_value is None or second_value is None:
            continue
        if not all(math.isfinite(float(value)) for value in (
            first_value, second_value
        )):
            continue
        total += 0.5 * (float(first_value) + float(second_value)) * (
            float(second_time) - float(first_time)
        )
        intervals += 1
    if intervals:
        return total
    return 0.0 if any(value is not None for value in values) else None
