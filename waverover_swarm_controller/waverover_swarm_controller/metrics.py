"""Dependency-light swarm diagnostics."""

import math

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
