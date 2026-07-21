"""Deterministic best-effort separation repair shared by all controllers."""

from dataclasses import dataclass
import hashlib
from itertools import combinations
import math


@dataclass(frozen=True)
class RepairReport:
    entries: dict
    iterations: int
    preferred_separation_before_m: float
    preferred_separation_after_m: float
    residual_violation_m: float
    least_violating_fallback: bool


def _minimum(points):
    distances = []
    for first, second in combinations(sorted(points), 2):
        if (
            first.startswith('active:') and first.removeprefix('active:') == second
        ) or (
            second.startswith('active:') and second.removeprefix('active:') == first
        ):
            continue
        distances.append(math.dist(points[first], points[second]))
    return min(distances, default=math.inf)


def _direction(first, second, epoch):
    digest = hashlib.sha256(
        ('%s|%s|%d' % (first, second, int(epoch))).encode('utf-8')
    ).digest()
    angle = int.from_bytes(digest[:8], 'big') / 2**64 * 2.0 * math.pi
    return math.cos(angle), math.sin(angle)


def repair_waypoints(proposed, active, geofence, preferred_separation,
                     max_iterations, target_epoch=0):
    """
    Repair movable proposals against each other and other active goals.

    Active goals are fixed obstacles for other robots. Stable IDs and epoch
    choose coincident directions; epoch-rotating pair priority avoids a
    permanent loser while identical inputs remain identical.
    """
    original = {
        str(key): (float(value[0]), float(value[1]))
        for key, value in proposed.items()
    }
    points = dict(original)
    fixed = {
        'active:' + str(key): (float(value[0]), float(value[1]))
        for key, value in active.items() if value is not None
    }
    before = _minimum({**points, **fixed})
    conflicts = {key: set() for key in points}
    best = dict(points)
    best_residual = math.inf
    iterations = 0

    for iteration in range(max_iterations):
        iterations = iteration + 1
        changed = False
        combined = {**points, **fixed}
        pairs = list(combinations(sorted(combined), 2))
        if pairs:
            offset = int(target_epoch) % len(pairs)
            pairs = pairs[offset:] + pairs[:offset]
        for first, second in pairs:
            first_robot = first.removeprefix('active:')
            second_robot = second.removeprefix('active:')
            if first.startswith('active:') and first_robot == second:
                continue
            if second.startswith('active:') and second_robot == first:
                continue
            distance = math.dist(combined[first], combined[second])
            deficit = preferred_separation - distance
            if deficit <= 1e-9:
                continue
            movable_first = first in points
            movable_second = second in points
            if not movable_first and not movable_second:
                continue
            if distance <= 1e-12:
                nx, ny = _direction(first, second, target_epoch)
            else:
                nx = (combined[first][0] - combined[second][0]) / distance
                ny = (combined[first][1] - combined[second][1]) / distance
            share_first = 0.5 if movable_first and movable_second else 1.0
            share_second = 0.5 if movable_first and movable_second else 1.0
            if movable_first:
                candidate = (
                    points[first][0] + nx * deficit * share_first,
                    points[first][1] + ny * deficit * share_first,
                )
                points[first] = (
                    min(max(candidate[0], geofence.x_min), geofence.x_max),
                    min(max(candidate[1], geofence.y_min), geofence.y_max),
                )
                conflicts[first].add(second_robot)
            if movable_second:
                candidate = (
                    points[second][0] - nx * deficit * share_second,
                    points[second][1] - ny * deficit * share_second,
                )
                points[second] = (
                    min(max(candidate[0], geofence.x_min), geofence.x_max),
                    min(max(candidate[1], geofence.y_min), geofence.y_max),
                )
                conflicts[second].add(first_robot)
            combined = {**points, **fixed}
            changed = True
        after_iteration = _minimum({**points, **fixed})
        residual = max(0.0, preferred_separation - after_iteration)
        if residual < best_residual:
            best_residual = residual
            best = dict(points)
        if not changed or residual <= 1e-6:
            break

    points = best
    after = _minimum({**points, **fixed})
    residual = max(0.0, preferred_separation - after)
    entries = {
        robot_id: {
            'original_waypoint': original[robot_id],
            'repaired_waypoint': points[robot_id],
            'displacement_m': math.dist(original[robot_id], points[robot_id]),
            'conflicting_robot_ids': tuple(sorted(conflicts[robot_id])),
        }
        for robot_id in sorted(points)
    }
    return points, RepairReport(
        entries=entries,
        iterations=iterations,
        preferred_separation_before_m=before,
        preferred_separation_after_m=after,
        residual_violation_m=residual,
        least_violating_fallback=residual > 1e-6,
    )
