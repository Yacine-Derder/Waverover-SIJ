"""Locally executed per-agent distributed MPC with Fiedler edge selection."""

import math
import time

import numpy as np

from .base import (
    ControllerUnavailableError,
    minimum_lookahead,
    optimization_hard_link_limit,
    optional_dependency,
    replace_first_future_points,
    SwarmController,
)
from ..models import ControllerResult


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
        self._last_agent_diagnostics = {}
        self._recovery_mode = False
        self._last_selected_edges = ()
        self._last_solver_status = 'not_run'

    def availability(self):
        return optional_dependency('cvxpy', self.__class__.__name__)

    @staticmethod
    def _selected_neighbors(robot_id, edges):
        return tuple(sorted(
            second if first == robot_id else first
            for first, second in edges if robot_id in (first, second)
        ))

    def _target_coefficients(self, robot_id, snapshot, edges):
        """Generalize the simulator gamma allocation to arbitrary weights."""
        robot = snapshot.robots[robot_id]
        neighbors = self._selected_neighbors(robot_id, edges)
        rover_neighbors = tuple(
            key for key in neighbors if key != snapshot.station.station_id
        )
        targets = tuple(snapshot.targets[key] for key in sorted(snapshot.targets))
        burden = max(1, len(neighbors))
        coefficients = {
            target.target_id: max(0.0, float(target.weight))
            for target in targets
        }
        suppressed = set()
        for target in targets:
            own_distance = math.dist(robot.position, target.position)
            for neighbor_id in rover_neighbors:
                neighbor_distance = math.dist(
                    snapshot.robots[neighbor_id].position, target.position
                )
                # Stable ID breaks exact ties so precisely one connected rover
                # retains the objective instead of every rover relinquishing it.
                if (
                    neighbor_distance < own_distance - 1e-12
                    or (
                        abs(neighbor_distance - own_distance) <= 1e-12
                        and neighbor_id < robot_id
                    )
                ):
                    suppressed.add(target.target_id)
                    break
        maximum_weight = max(coefficients.values(), default=0.0)
        relinquishes_high_value = any(
            target_id in suppressed
            and coefficients[target_id] >= maximum_weight - 1e-12
            and maximum_weight > 1e-12
            for target_id in coefficients
        )
        if relinquishes_high_value:
            coefficients = {
                target_id: 0.1 * value
                for target_id, value in coefficients.items()
            }
        for target_id in suppressed:
            coefficients[target_id] = 0.0
        coefficients = {
            target_id: value / burden
            for target_id, value in coefficients.items()
        }
        return coefficients, neighbors, burden, tuple(sorted(suppressed))

    @staticmethod
    def _prediction_at_current(previous, current, horizon):
        """Translate a prior trajectory to the newly measured current pose."""
        if not previous:
            return tuple(current for _ in range(horizon + 1))
        offset = (
            float(current[0]) - float(previous[0][0]),
            float(current[1]) - float(previous[0][1]),
        )
        translated = tuple(
            (
                float(point[0]) + offset[0],
                float(point[1]) + offset[1],
            )
            for point in previous
        )
        if len(translated) < horizon + 1:
            translated += (translated[-1],) * (horizon + 1 - len(translated))
        return (tuple(current),) + translated[1:horizon + 1]

    def _solve_agent(
        self,
        robot_id,
        snapshot,
        edges,
        neighbor_predictions,
    ):
        import cvxpy as cp
        robot = snapshot.robots[robot_id]
        horizon = self.config.controller.mpc_horizon
        path = cp.Variable((horizon + 1, 2))
        constraints = [path[0] == np.asarray(robot.position)]
        objective = 0
        target_coefficients, connected_neighbors, burden, suppressed = (
            self._target_coefficients(robot_id, snapshot, edges)
        )
        selected_rover_neighbors = tuple(
            key for key in connected_neighbors
            if key != snapshot.station.station_id
        )
        objective_neighbors = list(connected_neighbors)
        closest_fallback = None
        if not selected_rover_neighbors:
            candidates = [
                (
                    math.dist(
                        robot.position, snapshot.robots[key].position
                    ),
                    key,
                )
                for key in sorted(snapshot.robots) if key != robot_id
            ]
            candidates = [
                candidate for candidate in candidates
                if candidate[0] < self.config.communication.maximum_range_m
            ]
            if candidates:
                closest_fallback = min(candidates)[1]
                if closest_fallback not in objective_neighbors:
                    objective_neighbors.append(closest_fallback)
        objective_neighbors = tuple(sorted(objective_neighbors))
        maximum_link = optimization_hard_link_limit(self.config)
        connectivity_slack = cp.Variable(
            (horizon, len(connected_neighbors)), nonneg=True
        ) if self._recovery_mode and connected_neighbors else None
        if connectivity_slack is not None:
            objective += (
                self.config.controller.connectivity_recovery_slack_penalty
                * cp.sum(connectivity_slack)
            )
        for step in range(1, horizon + 1):
            constraints.append(
                cp.norm(path[step] - path[step - 1])
                <= self.config.controller.mpc_max_step_m
            )
            fence = self.config.safety.geofence
            constraints.extend((
                path[step, 0] >= fence.x_min,
                path[step, 0] <= fence.x_max,
                path[step, 1] >= fence.y_min,
                path[step, 1] <= fence.y_max,
            ))
            for target_id in sorted(snapshot.targets):
                coefficient = target_coefficients[target_id]
                if coefficient > 0.0:
                    objective += coefficient * cp.norm(
                        path[step]
                        - np.asarray(snapshot.targets[target_id].position)
                    )
            for connected_index, neighbor_id in enumerate(connected_neighbors):
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
                limit = maximum_link
                if connectivity_slack is not None:
                    limit += connectivity_slack[step - 1, connected_index]
                constraints.append(cp.norm(path[step] - neighbor_position) <= limit)
            for neighbor_id in objective_neighbors:
                if neighbor_id == snapshot.station.station_id:
                    neighbor_position = np.asarray(snapshot.station.position)
                else:
                    neighbor_path = neighbor_predictions.get(neighbor_id)
                    neighbor_position = (
                        np.asarray(snapshot.robots[neighbor_id].position)
                        if neighbor_path is None else
                        np.asarray(neighbor_path[
                            min(step, len(neighbor_path) - 1)
                        ])
                    )
                objective += (
                    self.config.controller.distributed_inter_agent_weight
                    * cp.maximum(
                        cp.norm(path[step] - neighbor_position),
                        self.config.communication.ideal_range_m,
                    )
                )
        problem = cp.Problem(cp.Minimize(objective), constraints)
        try:
            problem.solve(
                solver=cp.CLARABEL,
                warm_start=True,
                time_limit=self.config.safety.controller_result_timeout_sec,
            )
        except (cp.SolverError, AttributeError) as error:
            self._last_solver_status = 'solver_exception'
            self._last_agent_diagnostics[robot_id] = {
                'selected_neighbors': connected_neighbors,
                'objective_neighbors': objective_neighbors,
                'solver_status': 'solver_exception',
                'solver_exception': {
                    'type': type(error).__name__, 'message': str(error)
                },
            }
            raise ControllerUnavailableError(
                'Distributed agent %s solve failed: %s' % (robot_id, error)
            ) from error
        self._last_solver_status = str(problem.status)
        if problem.status not in ('optimal', 'optimal_inaccurate') or path.value is None:
            self._last_agent_diagnostics[robot_id] = {
                'selected_neighbors': connected_neighbors,
                'objective_neighbors': objective_neighbors,
                'solver_status': str(problem.status),
            }
            raise RuntimeError(
                'Distributed agent %s status is %s.' % (robot_id, problem.status)
            )
        solved_path = (robot.position,) + tuple(
            tuple(float(value) for value in path.value[step])
            for step in range(1, horizon + 1)
        )
        target_contribution = sum(
            target_coefficients[target_id]
            * math.dist(point, snapshot.targets[target_id].position)
            for point in solved_path[1:]
            for target_id in sorted(snapshot.targets)
        )
        neighbor_contribution = 0.0
        for step, point in enumerate(solved_path[1:], 1):
            for neighbor_id in objective_neighbors:
                if neighbor_id == snapshot.station.station_id:
                    neighbor_position = snapshot.station.position
                else:
                    neighbor_path = neighbor_predictions.get(neighbor_id)
                    neighbor_position = (
                        snapshot.robots[neighbor_id].position
                        if neighbor_path is None else
                        neighbor_path[min(step, len(neighbor_path) - 1)]
                    )
                neighbor_contribution += (
                    self.config.controller.distributed_inter_agent_weight
                    * max(
                        math.dist(point, neighbor_position),
                        self.config.communication.ideal_range_m,
                    )
                )
        dominant = max(
            target_coefficients,
            key=lambda key: (target_coefficients[key], key),
            default=None,
        )
        if dominant is not None and target_coefficients[dominant] <= 1e-12:
            dominant = None
        self._last_agent_diagnostics[robot_id] = {
            'effective_target_coefficients': target_coefficients,
            'dominant_target_id': dominant,
            'selected_neighbors': connected_neighbors,
            'objective_neighbors': objective_neighbors,
            'closest_rover_objective_fallback': closest_fallback,
            'relay_burden': len(connected_neighbors),
            'target_normalization_factor': float(burden),
            'suppressed_targets': suppressed,
            'target_objective_contribution': float(target_contribution),
            'neighbor_objective_contribution': float(neighbor_contribution),
            'solver_status': problem.status,
            'hard_link_limit_m': maximum_link,
            'maximum_connectivity_slack_m': (
                float(np.max(np.maximum(connectivity_slack.value, 0.0)))
                if connectivity_slack is not None
                and connectivity_slack.value is not None else 0.0
            ),
            'total_connectivity_slack_m': (
                float(np.sum(np.maximum(connectivity_slack.value, 0.0)))
                if connectivity_slack is not None
                and connectivity_slack.value is not None else 0.0
            ),
        }
        return solved_path, problem.status

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
                target_epoch=snapshot.target_epoch,
            )
        if snapshot.station is None:
            return ControllerResult(
                setpoints={},
                solver_status='missing_station',
                solve_duration_sec=time.monotonic() - started,
                diagnostic='Distributed MPC requires a station.',
                created_at=time.monotonic(),
                target_epoch=snapshot.target_epoch,
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
                target_epoch=snapshot.target_epoch,
            )
        edges = select_fiedler_edges(
            snapshot, self.config.communication.maximum_range_m
        )
        self._last_selected_edges = edges
        cycle_predictions = {
            robot_id: self._prediction_at_current(
                self._previous_predictions.get(robot_id),
                snapshot.robots[robot_id].position,
                self.config.controller.mpc_horizon,
            )
            for robot_id in robot_ids
        }
        solved = {}
        self._last_agent_diagnostics = {}
        statuses = []
        connectivity_retries = 0
        for robot_id in robot_ids:
            visible = (
                cycle_predictions
                if self.config.controller.distributed_update_semantics == 'jacobi'
                else {**cycle_predictions, **solved}
            )
            try:
                solved[robot_id], status = self._solve_agent(
                    robot_id, snapshot, edges, visible
                )
            except RuntimeError as error:
                if 'status is infeasible' not in str(error):
                    raise
                # A changed Fiedler graph can make shifted prior trajectories
                # mutually inconsistent. Retry the same local problem against
                # current stationary neighbor references.
                stationary_neighbors = {
                    neighbor_id: tuple(
                        snapshot.robots[neighbor_id].position
                        for _ in range(self.config.controller.mpc_horizon + 1)
                    )
                    for neighbor_id in robot_ids
                }
                solved[robot_id], status = self._solve_agent(
                    robot_id,
                    snapshot,
                    edges,
                    stationary_neighbors,
                )
                connectivity_retries += 1
            statuses.append(status)
        lookahead_setpoints = {
            robot_id: minimum_lookahead(
                snapshot.robots[robot_id],
                solved[robot_id][1],
                self.config.controller.minimum_mpc_lookahead_m,
                self.config.controller.mpc_max_step_m,
            )
            for robot_id in robot_ids
        }
        setpoints = lookahead_setpoints
        predicted_paths = replace_first_future_points(solved, setpoints)
        self._previous_predictions = predicted_paths
        maximum_slack = max((
            values['maximum_connectivity_slack_m']
            for values in self._last_agent_diagnostics.values()
        ), default=0.0)
        total_slack = sum(
            values['total_connectivity_slack_m']
            for values in self._last_agent_diagnostics.values()
        )
        return ControllerResult(
            setpoints=setpoints,
            target_assignments={
                robot_id: values['dominant_target_id']
                for robot_id, values in self._last_agent_diagnostics.items()
                if values['dominant_target_id'] is not None
            },
            predicted_paths=predicted_paths,
            selected_edges=edges,
            solver_status='+'.join(sorted(set(statuses))),
            solve_duration_sec=time.monotonic() - started,
            diagnostic=(
                'Team-aware Fiedler objective; locally executed %s per-agent updates; '
                'stationary-connectivity retries=%d.'
                % (
                    self.config.controller.distributed_update_semantics,
                    connectivity_retries,
                )
            ),
            created_at=time.monotonic(),
            target_epoch=snapshot.target_epoch,
            controller_diagnostics={
                'update_semantics': (
                    self.config.controller.distributed_update_semantics
                ),
                'inter_agent_weight': (
                    self.config.controller.distributed_inter_agent_weight
                ),
                'agents': dict(sorted(self._last_agent_diagnostics.items())),
                'hard_link_limit_m': optimization_hard_link_limit(self.config),
                'maximum_connectivity_slack_m': maximum_slack,
                'total_connectivity_slack_m': total_slack,
            },
            optimization_mode=(
                'recovery_mpc' if self._recovery_mode else 'normal_mpc'
            ),
        )

    def compute_recovery(self, snapshot):
        self._recovery_mode = True
        try:
            return self.compute(snapshot)
        finally:
            self._recovery_mode = False
