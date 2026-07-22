import math

import pytest

from waverover_swarm_controller.config import GeofenceConfig
from waverover_swarm_controller.waypoint_repair import repair_waypoints


FENCE = GeofenceConfig(-1.0, 1.0, -1.0, 1.0)


def repair(points, active=None, epoch=0, fence=FENCE, iterations=50):
    return repair_waypoints(points, active or {}, fence, 0.35, iterations, epoch)


def test_symmetric_and_coincident_repair_is_deterministic():
    points = {'1': (0.0, 0.0), '2': (0.0, 0.0)}
    first, report = repair(points, epoch=4)
    second, _ = repair(points, epoch=4)
    assert first == second
    assert math.dist(first['1'], first['2']) == pytest.approx(0.35, abs=1e-6)
    assert report.entries['1']['displacement_m'] == pytest.approx(0.175)
    assert report.entries['2']['displacement_m'] == pytest.approx(0.175)


def test_three_way_and_active_destination_conflicts_iterate():
    points, report = repair(
        {'1': (0.0, 0.0), '2': (0.01, 0.0), '3': (0.02, 0.0)},
        active={'9': (0.0, 0.25)},
    )
    assert report.iterations > 1
    assert all(FENCE.contains(point) for point in points.values())
    assert report.preferred_separation_after_m >= 0.35 - 1e-6


def test_boundary_impossibility_returns_least_violating_finite_result():
    fence = GeofenceConfig(0.0, 0.1, 0.0, 0.1)
    points, report = repair(
        {'1': (0.05, 0.05), '2': (0.05, 0.05)}, fence=fence
    )
    assert all(fence.contains(point) for point in points.values())
    assert all(math.isfinite(value) for point in points.values() for value in point)
    assert report.least_violating_fallback
    assert report.residual_violation_m > 0.0


def test_crossing_first_step_is_detected_and_replaced_by_safe_holds():
    points, report = repair_waypoints(
        {'1': (0.5, 0.0), '2': (-0.5, 0.0)},
        {}, FENCE, 0.35, 50,
        current_positions={'1': (-0.5, 0.0), '2': (0.5, 0.0)},
    )

    assert points == {'1': (-0.5, 0.0), '2': (0.5, 0.0)}
    assert report.segment_conflicts == (('1', '2'),)
    assert report.minimum_segment_separation_m == pytest.approx(1.0)
    assert report.segment_residual_violation_m == 0.0
