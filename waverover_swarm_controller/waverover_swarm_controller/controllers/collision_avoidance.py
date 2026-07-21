"""Pure pairwise geometry for conservative convex collision constraints."""

from dataclasses import dataclass
import hashlib
from itertools import combinations
import math


SEPARATION_NUMERIC_MARGIN_M = 1e-3
_COINCIDENT_TOLERANCE_M = 1e-12
SEPARATION_SLACK_PENALTY = 1000.0


class CollisionGeometryError(ValueError):
    """Raised when no safe fixed separating direction can be constructed."""


@dataclass(frozen=True)
class PairwiseGeometry:
    first_id: str
    second_id: str
    distance: float
    normal: tuple


def stable_separation_normal(first_id, second_id, positions, target_epoch=0):
    """Return a finite deterministic normal, including coincident points."""
    first = positions[first_id]
    second = positions[second_id]
    dx = float(first[0]) - float(second[0])
    dy = float(first[1]) - float(second[1])
    distance = math.hypot(dx, dy)
    if distance > _COINCIDENT_TOLERANCE_M:
        return dx / distance, dy / distance
    digest = hashlib.sha256(
        ('%s|%s|%d' % (first_id, second_id, int(target_epoch))).encode()
    ).digest()
    angle = int.from_bytes(digest[:8], 'big') / 2**64 * 2.0 * math.pi
    return math.cos(angle), math.sin(angle)


def centralized_soft_separation(
    positions, robot_ids, current_positions, preferred_separation,
    target_epoch=0,
):
    """Return affine preferred-separation constraints and penalized slack."""
    import cvxpy as cp

    pairs = tuple(combinations(sorted(robot_ids), 2))
    if not pairs or int(positions.shape[0]) <= 1:
        return (), 0.0
    slack = cp.Variable((int(positions.shape[0]) - 1, len(pairs)), nonneg=True)
    index = {robot_id: value for value, robot_id in enumerate(robot_ids)}
    constraints = []
    for pair_index, (first, second) in enumerate(pairs):
        normal = stable_separation_normal(
            first, second, current_positions, target_epoch
        )
        for step in range(1, int(positions.shape[0])):
            difference = positions[step, index[first]] - positions[step, index[second]]
            constraints.append(
                normal[0] * difference[0] + normal[1] * difference[1]
                + slack[step - 1, pair_index] >= preferred_separation
            )
    return tuple(constraints), SEPARATION_SLACK_PENALTY * cp.sum(slack)


def pairwise_geometries(positions, minimum_separation):
    """Return deterministic current-position separating directions."""
    geometries = []
    for first_id, second_id in combinations(sorted(positions), 2):
        first = positions[first_id]
        second = positions[second_id]
        delta_x = float(first[0]) - float(second[0])
        delta_y = float(first[1]) - float(second[1])
        distance = math.hypot(delta_x, delta_y)
        if not math.isfinite(distance):
            raise CollisionGeometryError(
                'Non-finite separation between rover IDs %s and %s.'
                % (first_id, second_id)
            )
        if distance <= _COINCIDENT_TOLERANCE_M:
            raise CollisionGeometryError(
                'Rover IDs %s and %s are coincident; no separating direction exists.'
                % (first_id, second_id)
            )
        if distance < minimum_separation:
            raise CollisionGeometryError(
                'Current separation between rover IDs %s and %s is %.3f m, '
                'below the configured %.3f m.'
                % (first_id, second_id, distance, minimum_separation)
            )
        geometries.append(PairwiseGeometry(
            first_id=first_id,
            second_id=second_id,
            distance=distance,
            normal=(delta_x / distance, delta_y / distance),
        ))
    return tuple(geometries)


def centralized_separation_constraints(
    positions,
    robot_ids,
    current_positions,
    minimum_separation,
    numeric_margin=SEPARATION_NUMERIC_MARGIN_M,
):
    """Build affine fixed-plane constraints for all pairs and future steps."""
    index = {robot_id: value for value, robot_id in enumerate(robot_ids)}
    required_separation = minimum_separation + numeric_margin
    constraints = []
    for geometry in pairwise_geometries(
        current_positions, minimum_separation
    ):
        first_index = index[geometry.first_id]
        second_index = index[geometry.second_id]
        for step in range(1, int(positions.shape[0])):
            difference = (
                positions[step, first_index] - positions[step, second_index]
            )
            constraints.append(
                geometry.normal[0] * difference[0]
                + geometry.normal[1] * difference[1]
                >= required_separation
            )
    return tuple(constraints)


def distributed_closing_limits(
    positions,
    minimum_separation,
    numeric_margin=SEPARATION_NUMERIC_MARGIN_M,
):
    """Return each agent's half of every pair's projected closing budget."""
    limits = {robot_id: [] for robot_id in sorted(positions)}
    for geometry in pairwise_geometries(positions, minimum_separation):
        closing_budget = (
            geometry.distance - minimum_separation - numeric_margin
        ) / 2.0
        if closing_budget < 0.0:
            raise CollisionGeometryError(
                'Initial closing budget between rover IDs %s and %s is %.6f m; '
                'their %.3f m separation does not include the %.3f m minimum '
                'plus %.3f m numerical margin.'
                % (
                    geometry.first_id,
                    geometry.second_id,
                    closing_budget,
                    geometry.distance,
                    minimum_separation,
                    numeric_margin,
                )
            )
        limits[geometry.first_id].append((
            geometry.second_id,
            geometry.normal,
            closing_budget,
        ))
        limits[geometry.second_id].append((
            geometry.first_id,
            (-geometry.normal[0], -geometry.normal[1]),
            closing_budget,
        ))
    return {
        robot_id: tuple(values)
        for robot_id, values in sorted(limits.items())
    }


def points_satisfy_centralized_planes(
    current_positions,
    future_positions,
    minimum_separation,
    numeric_margin=SEPARATION_NUMERIC_MARGIN_M,
):
    """Check post-processed points against every fixed separating plane."""
    threshold = minimum_separation + numeric_margin
    for geometry in pairwise_geometries(
        current_positions, minimum_separation
    ):
        first = future_positions[geometry.first_id]
        second = future_positions[geometry.second_id]
        projected_separation = (
            geometry.normal[0] * (float(first[0]) - float(second[0]))
            + geometry.normal[1] * (float(first[1]) - float(second[1]))
        )
        if projected_separation < threshold - 1e-7:
            return False
    return True


def points_satisfy_distributed_limits(
    current_positions,
    future_positions,
    closing_limits,
):
    """Check post-processing against each agent's individual half-budget."""
    for robot_id, limits in closing_limits.items():
        displacement = (
            float(future_positions[robot_id][0])
            - float(current_positions[robot_id][0]),
            float(future_positions[robot_id][1])
            - float(current_positions[robot_id][1]),
        )
        for _neighbor_id, normal, closing_budget in limits:
            projected_displacement = (
                normal[0] * displacement[0] + normal[1] * displacement[1]
            )
            if projected_displacement < -closing_budget - 1e-7:
                return False
    return True
