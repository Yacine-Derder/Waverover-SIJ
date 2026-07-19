"""Centralized convex position and assignment controller port."""

import math
import time

import numpy as np

from ..models import ControllerResult
from .base import (
    ControllerUnavailableError,
    SwarmController,
    optional_dependency,
    replace_first_future_points,
)
from .collision_avoidance import (
    centralized_separation_constraints,
    points_satisfy_centralized_planes,
)


def _tree_edges(snapshot):
    remaining = set(sorted(snapshot.robots))
    connected = {snapshot.station.station_id: snapshot.station.position}
    edges = []
    while remaining:
        candidates = []
        for robot_id in sorted(remaining):
            robot_position = snapshot.robots[robot_id].position
            for node_id, node_position in sorted(connected.items()):
                candidates.append((
                    math.dist(robot_position, node_position),
                    robot_id,
                    node_id,
                ))
        _, robot_id, parent_id = min(candidates)
        edges.append((parent_id, robot_id))
        connected[robot_id] = snapshot.robots[robot_id].position
        remaining.remove(robot_id)
    return tuple(edges)


def project_connectivity_safe(snapshot, setpoints, edges, maximum_distance):
    """Project carrots through the simulator ConnectedDrone parent balls."""
    output = dict(setpoints)
    known = {snapshot.station.station_id: snapshot.station.position}
    pending = list(edges)
    while pending:
        progressed = False
        for edge in tuple(pending):
            parent, robot_id = edge
            if parent not in known:
                continue
            parent_position = np.asarray(known[parent], dtype=float)
            point = np.asarray(output[robot_id], dtype=float)
            delta = point - parent_position
            distance = np.linalg.norm(delta)
            if distance > maximum_distance and distance > 1e-12:
                point = parent_position + maximum_distance * delta / distance
                output[robot_id] = tuple(float(value) for value in point)
            known[robot_id] = output[robot_id]
            pending.remove(edge)
            progressed = True
        if not progressed:
            raise RuntimeError('Connectivity edge graph is not rooted at station.')
    return output


class ConvexController(SwarmController):
    horizon_steps = 1

    def availability(self):
        available, reason = optional_dependency('cvxpy', self.__class__.__name__)
        if not available:
            return available, reason
        return optional_dependency('scipy', self.__class__.__name__)

    def _assign_targets(self, snapshot):
        from scipy.optimize import linear_sum_assignment
        robot_ids = tuple(sorted(snapshot.robots))
        targets = tuple(snapshot.targets[key] for key in sorted(snapshot.targets))
        if not targets:
            return {}
        robot_positions = np.asarray([
            snapshot.robots[robot_id].position for robot_id in robot_ids
        ])
        target_positions = np.asarray([target.position for target in targets])
        cost = np.linalg.norm(
            robot_positions[:, None, :] - target_positions[None, :, :],
            axis=2,
        )
        rows, columns = linear_sum_assignment(cost)
        assignments = {}
        for row, column in zip(rows, columns):
            assignments[robot_ids[row]] = targets[column]
        for robot_id in robot_ids:
            if robot_id not in assignments:
                nearest = min(
                    targets,
                    key=lambda target: (
                        math.dist(snapshot.robots[robot_id].position, target.position),
                        target.target_id,
                    ),
                )
                assignments[robot_id] = nearest
        return assignments

    def _solve(self, snapshot, horizon_steps):
        import cvxpy as cp
        robot_ids = tuple(sorted(snapshot.robots))
        if snapshot.station is None:
            return {}, {}, (), {}, 'missing_station'
        if not robot_ids:
            return {}, {}, (), {}, 'no_robots'
        if not snapshot.targets:
            stationary = {
                robot_id: snapshot.robots[robot_id].position
                for robot_id in robot_ids
            }
            paths = {robot_id: (point,) for robot_id, point in stationary.items()}
            return stationary, paths, (), {}, 'no_targets'

        assignments = self._assign_targets(snapshot)
        edges = _tree_edges(snapshot)
        count = len(robot_ids)
        index = {robot_id: value for value, robot_id in enumerate(robot_ids)}
        current = np.asarray([
            snapshot.robots[robot_id].position for robot_id in robot_ids
        ])
        positions = cp.Variable((horizon_steps + 1, count, 2))
        constraints = [positions[0] == current]
        constraints.extend(centralized_separation_constraints(
            positions,
            robot_ids,
            {
                robot_id: snapshot.robots[robot_id].position
                for robot_id in robot_ids
            },
            self.config.safety.minimum_separation_m,
        ))
        objective = 0
        maximum_link = self.config.communication.maximum_range_m - (
            2.0 * self.config.vehicle.turn_radius_m
        )
        for step in range(1, horizon_steps + 1):
            for robot_id in robot_ids:
                robot_index = index[robot_id]
                constraints.append(cp.norm(
                    positions[step, robot_index] - positions[step - 1, robot_index]
                ) <= self.config.controller.mpc_max_step_m)
                target = assignments[robot_id]
                weight = 1000.0 if target.is_main else max(target.weight, 1.0)
                objective += weight * cp.norm(
                    positions[step, robot_index] - np.asarray(target.position)
                )
            for parent, robot_id in edges:
                child = positions[step, index[robot_id]]
                if parent == snapshot.station.station_id:
                    parent_position = np.asarray(snapshot.station.position)
                else:
                    parent_position = positions[step, index[parent]]
                constraints.append(cp.norm(child - parent_position) <= maximum_link)
        problem = cp.Problem(cp.Minimize(objective), constraints)
        try:
            problem.solve(solver=cp.CLARABEL, warm_start=True)
        except (cp.SolverError, AttributeError) as error:
            raise ControllerUnavailableError(
                'Convex solve failed: %s' % error
            ) from error
        if problem.status not in ('optimal', 'optimal_inaccurate') or positions.value is None:
            raise RuntimeError('Convex problem status is %s.' % problem.status)
        paths = {
            robot_id: (
                snapshot.robots[robot_id].position,
            ) + tuple(
                tuple(float(value) for value in positions.value[step, index[robot_id]])
                for step in range(1, horizon_steps + 1)
            )
            for robot_id in robot_ids
        }
        optimized_setpoints = {
            robot_id: paths[robot_id][1] for robot_id in robot_ids
        }
        projected_setpoints = project_connectivity_safe(
            snapshot,
            optimized_setpoints,
            edges,
            self.config.communication.maximum_range_m
            - self.config.vehicle.turn_radius_m,
        )
        current_positions = {
            robot_id: snapshot.robots[robot_id].position
            for robot_id in robot_ids
        }
        setpoints = (
            projected_setpoints
            if points_satisfy_centralized_planes(
                current_positions,
                projected_setpoints,
                self.config.safety.minimum_separation_m,
            )
            else optimized_setpoints
        )
        paths = replace_first_future_points(paths, setpoints)
        target_assignments = {
            robot_id: target.target_id
            for robot_id, target in assignments.items()
        }
        return setpoints, paths, edges, target_assignments, problem.status

    def compute(self, snapshot):
        started = time.monotonic()
        available, reason = self.availability()
        if not available:
            raise ControllerUnavailableError(reason)
        setpoints, paths, edges, target_assignments, status = self._solve(
            snapshot, self.horizon_steps
        )
        return ControllerResult(
            setpoints=setpoints,
            target_assignments=target_assignments,
            predicted_paths=paths,
            selected_edges=edges,
            solver_status=status,
            solve_duration_sec=time.monotonic() - started,
            diagnostic='Alternating assignment port with connectivity-safe output projection.',
            created_at=time.monotonic(),
        )
