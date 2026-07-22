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
    connectivity_edges: tuple = ()
    maximum_link_violation_m: float = 0.0
    segment_conflicts: tuple = ()
    minimum_segment_separation_m: float = math.inf
    segment_residual_violation_m: float = 0.0


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


def _project_ball(point, center, radius):
    distance = math.dist(point, center)
    if distance <= radius or distance <= 1e-12:
        return point
    scale = radius / distance
    return (
        center[0] + scale * (point[0] - center[0]),
        center[1] + scale * (point[1] - center[1]),
    )


def _simultaneous_segment_distance(first_start, first_end,
                                   second_start, second_end):
    relative_start = (
        first_start[0] - second_start[0],
        first_start[1] - second_start[1],
    )
    relative_delta = (
        (first_end[0] - first_start[0]) - (second_end[0] - second_start[0]),
        (first_end[1] - first_start[1]) - (second_end[1] - second_start[1]),
    )
    denominator = relative_delta[0] ** 2 + relative_delta[1] ** 2
    fraction = 0.0 if denominator <= 1e-15 else min(1.0, max(
        0.0,
        -(relative_start[0] * relative_delta[0]
          + relative_start[1] * relative_delta[1]) / denominator,
    ))
    delta = (
        relative_start[0] + fraction * relative_delta[0],
        relative_start[1] + fraction * relative_delta[1],
    )
    return math.hypot(*delta), fraction


def repair_waypoints(proposed, active, geofence, preferred_separation,
                     max_iterations, target_epoch=0, *, current_positions=None,
                     connectivity_constraints=None, maximum_step_m=None):
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
    current_positions = {
        str(key): tuple(map(float, value))
        for key, value in (current_positions or {}).items()
    }
    connectivity_constraints = {
        str(key): tuple(sorted(value, key=lambda item: str(item[0])))
        for key, value in (connectivity_constraints or {}).items()
    }
    # First expose the simulator-style connectivity snap independently from
    # later collision displacement. Cyclic projections converge
    # deterministically when the ball intersection is nonempty.
    snapped = dict(points)
    for _iteration in range(max_iterations):
        previous = dict(snapped)
        for robot_id in sorted(snapped):
            point = (
                min(max(snapped[robot_id][0], geofence.x_min), geofence.x_max),
                min(max(snapped[robot_id][1], geofence.y_min), geofence.y_max),
            )
            for _neighbor_id, center, radius in connectivity_constraints.get(
                robot_id, ()
            ):
                point = _project_ball(point, center, radius)
            if maximum_step_m is not None and robot_id in current_positions:
                point = _project_ball(
                    point, current_positions[robot_id], maximum_step_m
                )
            snapped[robot_id] = point
        if all(
            math.dist(previous[key], snapped[key]) <= 1e-12
            for key in snapped
        ):
            break
    points = dict(snapped)
    fixed = {
        'active:' + str(key): (float(value[0]), float(value[1]))
        for key, value in active.items() if value is not None
    }
    before = _minimum({**points, **fixed})
    conflicts = {key: set() for key in points}
    best = dict(points)
    best_residual = math.inf
    iterations = 0
    segment_conflicts = set()

    # Crossing endpoints can be well separated while the simultaneous first
    # step collides. Holding both participants is the safest deterministic
    # correction available to this waypoint-only architecture.
    for first, second in combinations(sorted(points), 2):
        if first not in current_positions or second not in current_positions:
            continue
        distance, _fraction = _simultaneous_segment_distance(
            current_positions[first], points[first],
            current_positions[second], points[second],
        )
        if distance < preferred_separation - 1e-9:
            segment_conflicts.add((first, second))
            points[first] = current_positions[first]
            points[second] = current_positions[second]

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
        # Later operations are part of the same bounded fixed-point loop so
        # geofence/movement/connectivity projection cannot silently be the
        # final operation that undoes separation repair.
        for robot_id in sorted(points):
            point = points[robot_id]
            clipped = (
                min(max(point[0], geofence.x_min), geofence.x_max),
                min(max(point[1], geofence.y_min), geofence.y_max),
            )
            for _neighbor_id, center, radius in connectivity_constraints.get(
                robot_id, ()
            ):
                clipped = _project_ball(clipped, center, radius)
            if maximum_step_m is not None and robot_id in current_positions:
                clipped = _project_ball(
                    clipped, current_positions[robot_id], maximum_step_m
                )
            if math.dist(clipped, points[robot_id]) > 1e-12:
                changed = True
            points[robot_id] = clipped
        after_iteration = _minimum({**points, **fixed})
        separation_residual = max(0.0, preferred_separation - after_iteration)
        link_residual = max((
            max(0.0, math.dist(points[robot_id], center) - radius)
            for robot_id in sorted(points)
            for _neighbor_id, center, radius in connectivity_constraints.get(
                robot_id, ()
            )
        ), default=0.0)
        residual = separation_residual + link_residual
        if residual < best_residual:
            best_residual = residual
            best = dict(points)
        if not changed or residual <= 1e-6:
            break

    points = best
    after = _minimum({**points, **fixed})
    residual = max(0.0, preferred_separation - after)
    maximum_link_violation = max((
        max(0.0, math.dist(points[robot_id], center) - radius)
        for robot_id in sorted(points)
        for _neighbor_id, center, radius in connectivity_constraints.get(
            robot_id, ()
        )
    ), default=0.0)
    final_segment_minimum = math.inf
    for first, second in combinations(sorted(points), 2):
        if first not in current_positions or second not in current_positions:
            continue
        distance, _fraction = _simultaneous_segment_distance(
            current_positions[first], points[first],
            current_positions[second], points[second],
        )
        final_segment_minimum = min(final_segment_minimum, distance)
        if distance < preferred_separation - 1e-9:
            segment_conflicts.add((first, second))
    segment_residual = max(
        0.0, preferred_separation - final_segment_minimum
    )
    entries = {
        robot_id: {
            'original_waypoint': original[robot_id],
            'snapped_waypoint': snapped[robot_id],
            'repaired_waypoint': points[robot_id],
            'displacement_m': math.dist(original[robot_id], points[robot_id]),
            'connectivity_displacement_m': math.dist(
                original[robot_id], snapped[robot_id]
            ) if connectivity_constraints.get(robot_id) else 0.0,
            'collision_repair_displacement_m': math.dist(
                snapped[robot_id], points[robot_id]
            ),
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
        least_violating_fallback=(
            residual > 1e-6 or maximum_link_violation > 1e-6
            or segment_residual > 1e-6
        ),
        connectivity_edges=tuple(sorted({
            tuple(sorted((robot_id, str(neighbor_id))))
            for robot_id, constraints in connectivity_constraints.items()
            for neighbor_id, _center, _radius in constraints
        })),
        maximum_link_violation_m=maximum_link_violation,
        segment_conflicts=tuple(sorted(segment_conflicts)),
        minimum_segment_separation_m=final_segment_minimum,
        segment_residual_violation_m=segment_residual,
    )
