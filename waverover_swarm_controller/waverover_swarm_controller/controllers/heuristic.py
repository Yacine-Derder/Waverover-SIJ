"""Deterministic port of the simulator's centralized heuristic."""

import math
import time

import numpy as np

from ..models import ControllerResult
from .base import ControllerUnavailableError, SwarmController


STATION_ENDPOINT_TOLERANCE_M = 1e-9


class HeuristicController(SwarmController):
    def availability(self):
        try:
            from scipy.optimize import linear_sum_assignment  # noqa: F401
        except ImportError:
            return False, 'heuristic requires scipy.optimize.'
        return True, ''

    def _link_cost(self, distance):
        communication = self.config.communication
        if distance >= communication.maximum_range_m:
            return math.inf
        cost = 500.0
        if distance > communication.ideal_range_m:
            cost += (distance - communication.ideal_range_m) ** 2
        cost += 1.0 / max(communication.maximum_range_m - distance, 1e-9)
        return cost

    def optimal_relay_count(self, distance):
        if not math.isfinite(distance) or distance <= 0.0:
            return 0
        communication = self.config.communication
        margin_range = communication.maximum_range_m - (
            2.0 * self.config.vehicle.turn_radius_m
        )
        if margin_range <= 0.0:
            raise ControllerUnavailableError(
                'maximum communication range must exceed twice turn radius.'
            )
        first_feasible = max(1, int(distance / margin_range) + 1)
        upper = max(first_feasible + 1, int(distance / communication.ideal_range_m) + 1)
        best = first_feasible
        best_cost = math.inf
        for count in range(first_feasible, upper + 1):
            cost = self._link_cost(distance / count) * count
            if cost < best_cost:
                best = count
                best_cost = cost
        return best

    @staticmethod
    def _directions(points, station):
        directions = []
        for point in points:
            delta = point - station
            norm = np.linalg.norm(delta)
            directions.append(delta / norm if norm > 1e-12 else np.zeros(2))
        return np.asarray(directions)

    def _kmeans(self, points, count):
        if count <= 0:
            return np.empty((0, 2))
        ordered = np.asarray(sorted((tuple(point) for point in points)))
        center = np.mean(ordered, axis=0)
        initial = np.asarray([
            center + np.asarray([
                math.cos(2.0 * math.pi * index / count),
                math.sin(2.0 * math.pi * index / count),
            ])
            for index in range(count)
        ])
        centroids = initial
        for _ in range(100):
            distance = np.linalg.norm(
                ordered[:, None, :] - centroids[None, :, :], axis=2
            )
            labels = np.argmin(distance, axis=1)
            updated = centroids.copy()
            for index in range(count):
                members = ordered[labels == index]
                if len(members):
                    updated[index] = np.mean(members, axis=0)
            if np.allclose(updated, centroids, atol=1e-10, rtol=0.0):
                break
            centroids = updated
        return np.asarray(sorted(tuple(value) for value in centroids))

    def _relay_positions(self, station, target, count, observation=0.0):
        if count <= 0:
            return []
        delta = target - station
        distance = np.linalg.norm(delta)
        unit = delta / distance if distance > 1e-12 else np.zeros(2)
        covered = max(0.0, distance - observation)
        positions = [
            station + ((index + 1.0) / count) * covered * unit
            for index in range(count)
        ]
        # The station is already a fixed graph node, not a rover relay slot.
        return [
            point for point in positions
            if np.linalg.norm(point - station) > STATION_ENDPOINT_TOLERANCE_M
        ]

    @staticmethod
    def _validate_nonstation_setpoints(snapshot, setpoints):
        station = np.asarray(snapshot.station.position, dtype=float)
        duplicates = [
            robot_id for robot_id, point in sorted(setpoints.items())
            if np.linalg.norm(np.asarray(point, dtype=float) - station)
            <= STATION_ENDPOINT_TOLERANCE_M
        ]
        if duplicates:
            raise ControllerUnavailableError(
                'No non-station heuristic position is available for rover(s): '
                + ', '.join(duplicates) + '.'
            )

    def desired_positions(self, snapshot):
        robot_ids = tuple(sorted(snapshot.robots))
        if not robot_ids:
            return {}
        station = np.asarray(snapshot.station.position, dtype=float)
        targets = tuple(snapshot.targets[key] for key in sorted(snapshot.targets))
        positions = []
        priority = next((target for target in targets if target.is_priority), None)
        priority_count = 0
        if priority is not None:
            target_position = np.asarray(priority.position, dtype=float)
            distance = np.linalg.norm(target_position - station)
            priority_count = self.optimal_relay_count(distance)
            observation = 0.0
            if priority_count > len(robot_ids):
                priority_count = len(robot_ids)
                usable_range = self.config.communication.maximum_range_m - (
                    2.0 * self.config.vehicle.turn_radius_m
                )
                observation = max(0.0, distance - priority_count * usable_range)
            priority_positions = self._relay_positions(
                station, target_position, priority_count, observation
            )
            positions.extend(priority_positions)
            priority_count = len(priority_positions)

        unassigned = len(robot_ids) - priority_count
        background = [
            np.asarray(target.position, dtype=float)
            for target in targets if not target.is_priority
        ]
        cluster_count = min(unassigned, len(background))
        background_positions = []
        while cluster_count > 0:
            centroids = self._kmeans(background, cluster_count)
            candidate = []
            for centroid in centroids:
                count = self.optimal_relay_count(np.linalg.norm(centroid - station))
                candidate.extend(self._relay_positions(station, centroid, count))
            if len(candidate) <= unassigned:
                background_positions = candidate
                break
            cluster_count -= 1
        positions.extend(background_positions)

        from scipy.optimize import linear_sum_assignment
        # Surplus rovers hold their measured positions. The previous station
        # default duplicated the fixed GCS endpoint as a rover waypoint.
        output = {
            robot_id: tuple(snapshot.robots[robot_id].position)
            for robot_id in robot_ids
        }
        if positions:
            desired = np.asarray(positions)
            current = np.asarray([
                snapshot.robots[robot_id].position for robot_id in robot_ids
            ])
            cost = np.linalg.norm(current[:, None, :] - desired[None, :, :], axis=2)
            rows, columns = linear_sum_assignment(cost)
            for row, column in zip(rows, columns):
                output[robot_ids[row]] = tuple(float(v) for v in desired[column])
        self._validate_nonstation_setpoints(snapshot, output)
        return output

    def _selected_edges(self, snapshot, setpoints):
        remaining = set(sorted(setpoints))
        connected = {snapshot.station.station_id: snapshot.station.position}
        edges = []
        while remaining:
            best = None
            for robot_id in sorted(remaining):
                for node_id, position in sorted(connected.items()):
                    distance = math.dist(setpoints[robot_id], position)
                    candidate = (distance, robot_id, node_id)
                    if best is None or candidate < best:
                        best = candidate
            _, robot_id, node_id = best
            edges.append((node_id, robot_id))
            connected[robot_id] = setpoints[robot_id]
            remaining.remove(robot_id)
        return tuple(edges)

    def compute(self, snapshot):
        started = time.monotonic()
        if snapshot.station is None:
            return ControllerResult(
                setpoints={},
                solver_status='missing_station',
                solve_duration_sec=time.monotonic() - started,
                diagnostic='A station is required for relay generation.',
                created_at=time.monotonic(),
                target_epoch=snapshot.target_epoch,
            )
        available, reason = self.availability()
        if not available:
            raise ControllerUnavailableError(reason)
        setpoints = self.desired_positions(snapshot)
        assignments = {
            robot_id: min(
                snapshot.targets.values(),
                key=lambda target: (
                    math.dist(point, target.position), target.target_id
                ),
            ).target_id
            for robot_id, point in setpoints.items()
        } if snapshot.targets else {}
        return ControllerResult(
            setpoints=setpoints,
            target_assignments=assignments,
            selected_edges=self._selected_edges(snapshot, setpoints),
            solver_status='deterministic_heuristic',
            solve_duration_sec=time.monotonic() - started,
            diagnostic='Ported relay chains, clustering, and Hungarian assignment.',
            created_at=time.monotonic(),
            target_epoch=snapshot.target_epoch,
        )
