"""Locally executed per-agent distributed MPC with Fiedler edge selection."""

import math
import time

import numpy as np

from ..models import ControllerResult
from .base import (
    ControllerUnavailableError,
    SwarmController,
    minimum_lookahead,
    optional_dependency,
)


def select_fiedler_edges(snapshot, maximum_range):
    node_ids = (snapshot.station.station_id,) + tuple(sorted(snapshot.robots))
    positions = [snapshot.station.position] + [
        snapshot.robots[robot_id].position for robot_id in sorted(snapshot.robots)
    ]
    count = len(node_ids)
    if count < 2:
        return ()
    adjacency = np.zeros((count, count))
    for first in range(count):
        for second in range(first + 1, count):
            distance = math.dist(positions[first], positions[second])
            if distance < maximum_range:
                adjacency[first, second] = 1.0 / (0.01 + distance)
                adjacency[second, first] = adjacency[first, second]
    maximum = float(np.max(adjacency))
    if maximum <= 0.0:
        return ()
    adjacency /= maximum
    laplacian = np.diag(np.sum(adjacency, axis=0)) - adjacency
    eigenvalues, eigenvectors = np.linalg.eigh(laplacian)
    order = np.argsort(eigenvalues)
    fiedler = eigenvectors[:, order[1]]
    edges = set()
    for index in range(count):
        mask = (
            (adjacency[:, index] > 1e-7)
            & (np.abs(fiedler) <= abs(fiedler[index]) + 1e-12)
        )
        neighbors = np.arange(count)[mask]
        neighbors = neighbors[neighbors != index]
        if len(neighbors):
            metric = -fiedler[neighbors] * fiedler[index] * adjacency[neighbors, index]
            neighbor = int(neighbors[np.argmin(metric)])
            edges.add(tuple(sorted((node_ids[index], node_ids[neighbor]))))
    station_neighbors = sorted(
        second if first == snapshot.station.station_id else first
        for first, second in edges
        if snapshot.station.station_id in (first, second)
    )
    for index, first in enumerate(station_neighbors):
        for second in station_neighbors[index + 1:]:
            edges.add(tuple(sorted((first, second))))
    return tuple(sorted(edges))


class DistributedMpcController(SwarmController):
    def __init__(self, config):
        super().__init__(config)
        self._previous_predictions = {}

    def availability(self):
        return optional_dependency('cvxpy', self.__class__.__name__)

    def _target_for(self, robot, snapshot):
        return min(
            snapshot.targets.values(),
            key=lambda target: (
                math.dist(robot.position, target.position) / max(target.weight, 1e-9),
                target.target_id,
            ),
        )

    def _solve_agent(self, robot_id, snapshot, edges, neighbor_predictions):
        import cvxpy as cp
        robot = snapshot.robots[robot_id]
        horizon = self.config.controller.mpc_horizon
        path = cp.Variable((horizon + 1, 2))
        constraints = [path[0] == np.asarray(robot.position)]
        objective = 0
        target = self._target_for(robot, snapshot)
        connected_neighbors = []
        for first, second in edges:
            if robot_id == first:
                connected_neighbors.append(second)
            elif robot_id == second:
                connected_neighbors.append(first)
        maximum_link = self.config.communication.maximum_range_m - (
            2.0 * self.config.vehicle.turn_radius_m
        )
        for step in range(1, horizon + 1):
            constraints.append(
                cp.norm(path[step] - path[step - 1])
                <= self.config.controller.mpc_max_step_m
            )
            objective += target.weight * cp.norm(
                path[step] - np.asarray(target.position)
            )
            for neighbor_id in connected_neighbors:
                if neighbor_id == snapshot.station.station_id:
                    neighbor_position = np.asarray(snapshot.station.position)
                else:
                    neighbor_path = neighbor_predictions.get(neighbor_id)
                    if neighbor_path is None:
                        neighbor_position = np.asarray(
                            snapshot.robots[neighbor_id].position
                        )
                    else:
                        neighbor_position = np.asarray(
                            neighbor_path[min(step, len(neighbor_path) - 1)]
                        )
                constraints.append(
                    cp.norm(path[step] - neighbor_position) <= maximum_link
                )
        problem = cp.Problem(cp.Minimize(objective), constraints)
        try:
            problem.solve(solver=cp.CLARABEL, warm_start=True)
        except (cp.SolverError, AttributeError) as error:
            raise ControllerUnavailableError(
                'Distributed agent %s solve failed: %s' % (robot_id, error)
            ) from error
        if problem.status not in ('optimal', 'optimal_inaccurate') or path.value is None:
            raise RuntimeError(
                'Distributed agent %s status is %s.' % (robot_id, problem.status)
            )
        return tuple(
            tuple(float(value) for value in path.value[step])
            for step in range(horizon + 1)
        ), problem.status

    def compute(self, snapshot):
        started = time.monotonic()
        available, reason = self.availability()
        if not available:
            raise ControllerUnavailableError(reason)
        robot_ids = tuple(sorted(snapshot.robots))
        self._previous_predictions = {
            key: value for key, value in self._previous_predictions.items()
            if key in snapshot.robots
        }
        if not robot_ids:
            return ControllerResult(
                setpoints={},
                solver_status='no_robots',
                solve_duration_sec=time.monotonic() - started,
                created_at=time.monotonic(),
            )
        if snapshot.station is None:
            return ControllerResult(
                setpoints={},
                solver_status='missing_station',
                solve_duration_sec=time.monotonic() - started,
                diagnostic='Distributed MPC requires a station.',
                created_at=time.monotonic(),
            )
        if not snapshot.targets:
            stationary = {
                key: snapshot.robots[key].position for key in robot_ids
            }
            return ControllerResult(
                setpoints=stationary,
                predicted_paths={key: (point,) for key, point in stationary.items()},
                solver_status='no_targets',
                solve_duration_sec=time.monotonic() - started,
                created_at=time.monotonic(),
            )
        edges = select_fiedler_edges(
            snapshot, self.config.communication.maximum_range_m
        )
        cycle_predictions = {
            robot_id: self._previous_predictions.get(
                robot_id,
                tuple(
                    snapshot.robots[robot_id].position
                    for _ in range(self.config.controller.mpc_horizon + 1)
                ),
            )
            for robot_id in robot_ids
        }
        solved = {}
        statuses = []
        for robot_id in robot_ids:
            visible = (
                cycle_predictions
                if self.config.controller.distributed_update_semantics == 'jacobi'
                else {**cycle_predictions, **solved}
            )
            solved[robot_id], status = self._solve_agent(
                robot_id, snapshot, edges, visible
            )
            statuses.append(status)
        self._previous_predictions = solved
        setpoints = {
            robot_id: minimum_lookahead(
                snapshot.robots[robot_id],
                solved[robot_id][1],
                self.config.controller.minimum_mpc_lookahead_m,
                self.config.controller.mpc_max_step_m,
            )
            for robot_id in robot_ids
        }
        return ControllerResult(
            setpoints=setpoints,
            predicted_paths=solved,
            selected_edges=edges,
            solver_status='+'.join(sorted(set(statuses))),
            solve_duration_sec=time.monotonic() - started,
            diagnostic=(
                'Fiedler edges; locally executed %s per-agent updates.'
                % self.config.controller.distributed_update_semantics
            ),
            created_at=time.monotonic(),
        )
