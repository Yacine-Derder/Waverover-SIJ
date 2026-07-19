"""Dependency-light swarm diagnostics."""

import math
from itertools import combinations

import numpy as np


def minimum_pairwise_distance(points):
    values = list(points)
    if len(values) < 2:
        return math.inf
    return min(
        math.dist(values[first], values[second])
        for first in range(len(values))
        for second in range(first + 1, len(values))
    )


def minimum_pairwise_with_ids(points_by_id):
    """Return deterministic closest-pair distance and IDs."""
    return min(
        (
            math.dist(points_by_id[first], points_by_id[second]),
            first,
            second,
        )
        for first, second in combinations(sorted(points_by_id), 2)
    ) if len(points_by_id) >= 2 else (math.inf, None, None)


def algebraic_connectivity(node_positions, maximum_range):
    node_ids = tuple(sorted(node_positions))
    if len(node_ids) < 2:
        return 0.0
    adjacency = np.zeros((len(node_ids), len(node_ids)))
    for first in range(len(node_ids)):
        for second in range(first + 1, len(node_ids)):
            if math.dist(
                node_positions[node_ids[first]],
                node_positions[node_ids[second]],
            ) <= maximum_range:
                adjacency[first, second] = 1.0
                adjacency[second, first] = 1.0
    laplacian = np.diag(np.sum(adjacency, axis=0)) - adjacency
    eigenvalues = np.linalg.eigvalsh(laplacian)
    return max(0.0, float(np.sort(eigenvalues)[1]))


def weighted_algebraic_connectivity(
    node_positions,
    ideal_range,
    maximum_range,
    alpha=5.0,
):
    """Paper-style logistic link weights with a hard maximum-range cutoff."""
    node_ids = tuple(sorted(node_positions))
    if len(node_ids) < 2:
        return 0.0
    adjacency = np.zeros((len(node_ids), len(node_ids)))
    for first, second in combinations(range(len(node_ids)), 2):
        distance = math.dist(
            node_positions[node_ids[first]], node_positions[node_ids[second]]
        )
        if distance <= maximum_range:
            weight = math.exp(-alpha * (distance - ideal_range))
            weight /= 1.0 + weight
            adjacency[first, second] = weight
            adjacency[second, first] = weight
    laplacian = np.diag(np.sum(adjacency, axis=0)) - adjacency
    eigenvalues = np.linalg.eigvalsh(laplacian)
    return max(0.0, float(np.sort(eigenvalues)[1]))


def connected_components(node_positions, maximum_range):
    pending = set(node_positions)
    components = []
    while pending:
        root = min(pending)
        pending.remove(root)
        component = {root}
        frontier = [root]
        while frontier:
            first = frontier.pop()
            neighbors = [
                second for second in sorted(pending)
                if math.dist(
                    node_positions[first], node_positions[second]
                ) <= maximum_range
            ]
            for second in neighbors:
                pending.remove(second)
                component.add(second)
                frontier.append(second)
        components.append(tuple(sorted(component)))
    return tuple(components)
