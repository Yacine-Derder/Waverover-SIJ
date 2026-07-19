"""PC-hosted per-agent port of the simulator decentralized heuristic."""

import time

import numpy as np

from ..models import ControllerResult
from .heuristic import HeuristicController


class DecentralizedHeuristicController(HeuristicController):
    def __init__(self, config):
        super().__init__(config)
        self._agent_targets = {}

    def _assign_clusters(self, snapshot):
        robot_ids = tuple(sorted(snapshot.robots))
        targets = tuple(snapshot.targets[key] for key in sorted(snapshot.targets))
        self._agent_targets = {
            key: value for key, value in self._agent_targets.items()
            if key in snapshot.robots and value in snapshot.targets
        }
        if not targets:
            return
        main = next((target for target in targets if target.is_main), targets[0])
        ordered_targets = (main,) + tuple(target for target in targets if target != main)
        for index, robot_id in enumerate(robot_ids):
            if robot_id not in self._agent_targets:
                self._agent_targets[robot_id] = ordered_targets[
                    index % len(ordered_targets)
                ].target_id

    def _compute_agent(self, robot_id, snapshot, previous_setpoints):
        station = np.asarray(snapshot.station.position, dtype=float)
        target_id = self._agent_targets.get(robot_id)
        if target_id not in snapshot.targets:
            return tuple(station)
        target = np.asarray(snapshot.targets[target_id].position, dtype=float)
        delta = target - station
        distance = np.linalg.norm(delta)
        if distance <= 1e-12:
            return tuple(station)
        unit = delta / distance
        members = sorted(
            key for key, value in self._agent_targets.items()
            if value == target_id and key in snapshot.robots
        )
        projections = sorted(
            (
                np.dot(np.asarray(snapshot.robots[key].position) - station, unit),
                key,
            )
            for key in members
        )
        rank = next(index for index, (_, key) in enumerate(projections) if key == robot_id)
        previous_position = (
            station if rank == 0
            else np.asarray(previous_setpoints.get(
                projections[rank - 1][1],
                snapshot.robots[projections[rank - 1][1]].position,
            ))
        )
        ideal = target if rank == len(projections) - 1 else (
            previous_position + target
        ) / 2.0
        link = ideal - previous_position
        link_distance = np.linalg.norm(link)
        maximum = self.config.communication.maximum_range_m - (
            2.0 * self.config.vehicle.turn_radius_m
        )
        if link_distance > maximum and link_distance > 1e-12:
            ideal = previous_position + maximum * link / link_distance
        return tuple(float(value) for value in ideal)

    def compute(self, snapshot):
        started = time.monotonic()
        if snapshot.station is None:
            return ControllerResult(
                setpoints={},
                solver_status='missing_station',
                solve_duration_sec=time.monotonic() - started,
                diagnostic='Per-agent relay logic requires a station.',
                created_at=time.monotonic(),
            )
        self._assign_clusters(snapshot)
        setpoints = {}
        # Explicit per-agent boundary. All agents read the same cycle snapshot;
        # deterministic earlier setpoints model the simulator's local chain.
        for robot_id in sorted(snapshot.robots):
            setpoints[robot_id] = self._compute_agent(
                robot_id, snapshot, setpoints
            )
        return ControllerResult(
            setpoints=setpoints,
            target_assignments=self._agent_targets,
            selected_edges=self._selected_edges(snapshot, setpoints),
            solver_status='decentralized_local_agents',
            solve_duration_sec=time.monotonic() - started,
            diagnostic=(
                'Per-agent target-aware relay logic executed locally on PC; '
                'no inter-agent ROS transport.'
            ),
            created_at=time.monotonic(),
        )
