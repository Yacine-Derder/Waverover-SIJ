"""Hybrid target clustering with strictly local reactive relay formation."""

from dataclasses import dataclass
import math
import time

import numpy as np

from .base import ControllerUnavailableError
from .heuristic import HeuristicController
from ..models import ControllerResult


@dataclass(frozen=True)
class RelayCluster:
    cluster_id: str
    target_ids: tuple
    destination_target_id: str
    position: tuple
    priority: bool
    required_robots: int


@dataclass
class LocalRelayState:
    cluster_id: object = None
    predecessor_id: object = None
    successor_id: object = None
    previous_waypoint: object = None
    projection_m: float = 0.0
    assignment_revision: int = 0
    role: str = 'unassigned'
    connected_to_station: bool = False


class DecentralizedHeuristicController(HeuristicController):
    """Rover adaptation of the simulator's hybrid decentralized heuristic.

    Target clustering and line resource allocation are global.  Every relay
    waypoint is then computed from an immutable local view containing only the
    rover, station, assigned cluster, and in-range same-cluster neighbors.
    """

    def __init__(self, config):
        super().__init__(config)
        self._clusters = ()
        self._agents = {}
        self._objective_signature = None
        self._membership_signature = ()
        self._cluster_revision = 0
        self._assignment_revision = 0
        self._compute_count = 0

    @property
    def safe_link_distance_m(self):
        distance = self.config.communication.maximum_range_m - (
            2.0 * self.config.vehicle.turn_radius_m
        )
        if distance <= 0.0:
            raise ControllerUnavailableError(
                'heuristic_decentralized safe link distance must be positive.'
            )
        return distance

    @staticmethod
    def _semantic_objective(snapshot):
        return (
            snapshot.station.station_id,
            tuple(map(float, snapshot.station.position)),
            snapshot.priority_target_id,
            tuple(
                (
                    target_id,
                    float(target.x),
                    float(target.y),
                    float(target.weight),
                    bool(target.is_priority),
                )
                for target_id, target in sorted(snapshot.targets.items())
            ),
        )

    @staticmethod
    def _priority_target_id(snapshot):
        if snapshot.priority_target_id in snapshot.targets:
            return snapshot.priority_target_id
        priority = sorted(
            target.target_id for target in snapshot.targets.values()
            if target.is_priority
        )
        if priority:
            return priority[0]
        return min(
            snapshot.targets,
            key=lambda key: (-snapshot.targets[key].weight, key),
            default=None,
        )

    @staticmethod
    def _unit_direction(point, station):
        delta = np.asarray(point, dtype=float) - np.asarray(station, dtype=float)
        norm = float(np.linalg.norm(delta))
        return delta / norm if norm > 1e-12 else np.zeros(2)

    def _directional_labels(self, snapshot, count):
        """Deterministic persistent K-means on target directions."""
        targets = tuple(snapshot.targets[key] for key in sorted(snapshot.targets))
        directions = np.asarray([
            self._unit_direction(target.position, snapshot.station.position)
            for target in targets
        ])
        previous = [
            self._unit_direction(cluster.position, snapshot.station.position)
            for cluster in self._clusters[:count]
        ]
        centers = list(previous)
        for index in range(len(centers), count):
            angle = 2.0 * math.pi * (index + 0.5) / count
            centers.append(np.asarray((math.cos(angle), math.sin(angle))))
        centers = np.asarray(centers, dtype=float)
        labels = np.zeros(len(targets), dtype=int)
        for _iteration in range(100):
            distances = np.linalg.norm(
                directions[:, None, :] - centers[None, :, :], axis=2
            )
            updated_labels = np.argmin(distances, axis=1)
            updated_centers = centers.copy()
            for index in range(count):
                members = directions[updated_labels == index]
                if len(members):
                    mean = np.mean(members, axis=0)
                    norm = float(np.linalg.norm(mean))
                    updated_centers[index] = (
                        mean / norm if norm > 1e-12 else mean
                    )
            if (
                np.array_equal(updated_labels, labels)
                and np.allclose(updated_centers, centers, atol=1e-12, rtol=0.0)
            ):
                labels = updated_labels
                break
            labels = updated_labels
            centers = updated_centers
        return targets, labels

    def _clusters_for_count(self, snapshot, count):
        targets, labels = self._directional_labels(snapshot, count)
        priority_id = self._priority_target_id(snapshot)
        output = []
        for index in range(count):
            members = tuple(
                target for target, label in zip(targets, labels)
                if label == index
            )
            if not members:
                continue
            target_ids = tuple(sorted(target.target_id for target in members))
            priority = priority_id in target_ids
            if priority:
                destination = snapshot.targets[priority_id]
                position = destination.position
            else:
                destination = min(
                    members, key=lambda target: (-target.weight, target.target_id)
                )
                position = tuple(np.mean(
                    np.asarray([target.position for target in members]), axis=0
                ))
            distance = math.dist(snapshot.station.position, position)
            required = max(1, int(distance / self.safe_link_distance_m) + 1)
            output.append(RelayCluster(
                cluster_id='cluster:' + ','.join(target_ids),
                target_ids=target_ids,
                destination_target_id=destination.target_id,
                position=tuple(float(value) for value in position),
                priority=priority,
                required_robots=required,
            ))
        return tuple(sorted(
            output,
            key=lambda cluster: (
                not cluster.priority,
                -snapshot.targets[cluster.destination_target_id].weight,
                cluster.cluster_id,
            ),
        ))

    def _build_clusters(self, snapshot):
        maximum = min(len(snapshot.robots), len(snapshot.targets))
        if maximum <= 0:
            return ()
        for count in range(maximum, 0, -1):
            clusters = self._clusters_for_count(snapshot, count)
            if count == 1 or sum(
                cluster.required_robots for cluster in clusters
            ) <= len(snapshot.robots):
                return clusters
        return ()

    def _allocate_agents(self, snapshot, clusters):
        robot_ids = tuple(sorted(snapshot.robots))
        capacities = {
            cluster.cluster_id: cluster.required_robots for cluster in clusters
        }
        assignment = {}
        # Preserve valid assignments first, matching the simulator's persistent
        # TargetAwareDrone.target state.
        for cluster in clusters:
            retained = [
                robot_id for robot_id in robot_ids
                if self._agents.get(robot_id, LocalRelayState()).cluster_id
                == cluster.cluster_id
            ][:capacities[cluster.cluster_id]]
            for robot_id in retained:
                assignment[robot_id] = cluster.cluster_id
            capacities[cluster.cluster_id] -= len(retained)
        available = set(robot_ids) - set(assignment)
        for cluster in clusters:
            for _index in range(capacities[cluster.cluster_id]):
                if not available:
                    break
                # The simulator assigns station-visible agents. Prefer those,
                # then use deterministic station proximity as the PC-hosted
                # fallback when the local station neighborhood is depleted.
                robot_id = min(
                    available,
                    key=lambda key: (
                        math.dist(
                            snapshot.robots[key].position,
                            snapshot.station.position,
                        ) > self.config.communication.maximum_range_m,
                        math.dist(
                            snapshot.robots[key].position,
                            snapshot.station.position,
                        ),
                        key,
                    ),
                )
                assignment[robot_id] = cluster.cluster_id
                available.remove(robot_id)
        return assignment

    def _ensure_plan(self, snapshot):
        objective = self._semantic_objective(snapshot)
        membership = tuple(sorted(snapshot.robots))
        rebuild = (
            objective != self._objective_signature
            or membership != self._membership_signature
        )
        if not rebuild:
            return
        clusters = self._build_clusters(snapshot)
        cluster_signature = tuple(
            (
                cluster.cluster_id, cluster.target_ids,
                cluster.destination_target_id, cluster.position,
                cluster.priority, cluster.required_robots,
            )
            for cluster in clusters
        )
        previous_cluster_signature = tuple(
            (
                cluster.cluster_id, cluster.target_ids,
                cluster.destination_target_id, cluster.position,
                cluster.priority, cluster.required_robots,
            )
            for cluster in self._clusters
        )
        if cluster_signature != previous_cluster_signature:
            self._cluster_revision += 1
        assignment = self._allocate_agents(snapshot, clusters)
        previous_assignment = {
            robot_id: state.cluster_id
            for robot_id, state in self._agents.items()
            if state.cluster_id is not None
        }
        if assignment != previous_assignment:
            self._assignment_revision += 1
        self._agents = {
            robot_id: LocalRelayState(
                cluster_id=assignment.get(robot_id),
                previous_waypoint=(
                    self._agents[robot_id].previous_waypoint
                    if robot_id in self._agents
                    and assignment.get(robot_id)
                    == self._agents[robot_id].cluster_id
                    else snapshot.robots[robot_id].position
                ),
                assignment_revision=self._assignment_revision,
            )
            for robot_id in membership
        }
        self._clusters = clusters
        self._objective_signature = objective
        self._membership_signature = membership

    def _local_view(self, robot_id, snapshot, assignment):
        """Return only observable same-cluster neighbors for one agent."""
        cluster_id = assignment.get(robot_id)
        own = snapshot.robots[robot_id]
        return {
            other_id: snapshot.robots[other_id]
            for other_id in sorted(snapshot.robots)
            if other_id != robot_id
            and assignment.get(other_id) == cluster_id
            and cluster_id is not None
            and math.dist(
                own.position, snapshot.robots[other_id].position
            ) <= self.config.communication.maximum_range_m + 1e-12
        }

    def _ordered_neighbor(self, candidates, previous_id, projections, reverse):
        if not candidates:
            return None
        selected = sorted(
            candidates,
            key=lambda key: (projections[key], key),
            reverse=reverse,
        )[0]
        if previous_id in candidates and abs(
            projections[previous_id] - projections[selected]
        ) <= self.config.controller.decentralized_ordering_hysteresis_m:
            return previous_id
        return selected

    def _compute_local_agent(
        self, robot_id, snapshot, cluster, assignment, previous_states
    ):
        own = snapshot.robots[robot_id]
        station = np.asarray(snapshot.station.position, dtype=float)
        target = np.asarray(cluster.position, dtype=float)
        delta = target - station
        distance = float(np.linalg.norm(delta))
        if distance <= 1e-12:
            return own.position, {
                'cluster_id': cluster.cluster_id,
                'local_neighbor_ids': (),
                'predecessor_id': snapshot.station.station_id,
                'successor_id': None,
                'station_is_predecessor': True,
                'role': 'terminal',
                'connectivity_corrected': False,
                'connected_to_station': True,
                'projection_m': 0.0,
                '_predecessor_robot_id': None,
                '_successor_robot_id': None,
            }
        unit = delta / distance
        local = self._local_view(robot_id, snapshot, assignment)
        projections = {
            key: float(np.dot(
                np.asarray(state.position, dtype=float) - station, unit
            ))
            for key, state in local.items()
        }
        own_projection = float(np.dot(
            np.asarray(own.position, dtype=float) - station, unit
        ))
        own_order = (own_projection, robot_id)
        predecessors = [
            key for key in local
            if (projections[key], key) < own_order
        ]
        successors = [
            key for key in local
            if (projections[key], key) > own_order
        ]
        previous = previous_states.get(robot_id, LocalRelayState())
        predecessor_id = self._ordered_neighbor(
            predecessors, previous.predecessor_id, projections, True
        )
        successor_id = self._ordered_neighbor(
            successors, previous.successor_id, projections, False
        )
        predecessor_position = (
            station if predecessor_id is None else np.asarray(
                previous_states[predecessor_id].previous_waypoint
                if previous_states[predecessor_id].previous_waypoint is not None
                else local[predecessor_id].position,
                dtype=float,
            )
        )
        successor_position = (
            target if successor_id is None else np.asarray(
                previous_states[successor_id].previous_waypoint
                if previous_states[successor_id].previous_waypoint is not None
                else local[successor_id].position,
                dtype=float,
            )
        )
        ideal = (
            successor_position if successor_id is None
            else (predecessor_position + successor_position) / 2.0
        )
        link = ideal - predecessor_position
        link_distance = float(np.linalg.norm(link))
        corrected = link_distance > self.safe_link_distance_m + 1e-12
        waypoint = ideal
        if corrected and link_distance > 1e-12:
            waypoint = predecessor_position + (
                self.safe_link_distance_m * link / link_distance
            )
        predecessor_connected = (
            predecessor_id is None
            or previous_states[predecessor_id].connected_to_station
        )
        connected = predecessor_connected and math.dist(
            tuple(predecessor_position), own.position
        ) <= self.config.communication.maximum_range_m + 1e-12
        return tuple(float(value) for value in waypoint), {
            'cluster_id': cluster.cluster_id,
            'local_neighbor_ids': tuple(local),
            'predecessor_id': (
                snapshot.station.station_id
                if predecessor_id is None else predecessor_id
            ),
            'successor_id': (
                cluster.destination_target_id
                if successor_id is None else successor_id
            ),
            'station_is_predecessor': predecessor_id is None,
            'role': 'terminal' if successor_id is None else 'relay',
            'connectivity_corrected': corrected,
            'connected_to_station': connected,
            'projection_m': own_projection,
            '_predecessor_robot_id': predecessor_id,
            '_successor_robot_id': successor_id,
        }

    def reset(self):
        self._clusters = ()
        self._agents = {}
        self._objective_signature = None
        self._membership_signature = ()
        self._cluster_revision = 0
        self._assignment_revision = 0
        self._compute_count = 0

    def compute(self, snapshot):
        started = time.monotonic()
        if snapshot.station is None:
            return ControllerResult(
                setpoints={}, solver_status='missing_station',
                solve_duration_sec=time.monotonic() - started,
                diagnostic='Decentralized relay logic requires a station.',
                created_at=time.monotonic(), target_epoch=snapshot.target_epoch,
            )
        self.safe_link_distance_m
        self._ensure_plan(snapshot)
        self._compute_count += 1
        assignment = {
            robot_id: state.cluster_id
            for robot_id, state in self._agents.items()
        }
        clusters = {
            cluster.cluster_id: cluster for cluster in self._clusters
        }
        previous_states = {
            robot_id: LocalRelayState(**vars(state))
            for robot_id, state in self._agents.items()
        }
        setpoints = {}
        local_diagnostics = {}
        selected_edges = []
        for robot_id in sorted(snapshot.robots):
            cluster_id = assignment.get(robot_id)
            if cluster_id is None or cluster_id not in clusters:
                setpoints[robot_id] = snapshot.robots[robot_id].position
                local_diagnostics[robot_id] = {
                    'cluster_id': None,
                    'local_neighbor_ids': (),
                    'predecessor_id': None,
                    'successor_id': None,
                    'station_is_predecessor': False,
                    'role': 'unassigned_hold',
                    'connectivity_corrected': False,
                    'connected_to_station': False,
                    'projection_m': 0.0,
                }
                continue
            waypoint, details = self._compute_local_agent(
                robot_id, snapshot, clusters[cluster_id], assignment,
                previous_states,
            )
            predecessor_robot = details.pop('_predecessor_robot_id')
            successor_robot = details.pop('_successor_robot_id')
            setpoints[robot_id] = waypoint
            local_diagnostics[robot_id] = details
            selected_edges.append((
                snapshot.station.station_id
                if predecessor_robot is None else predecessor_robot,
                robot_id,
            ))
            self._agents[robot_id] = LocalRelayState(
                cluster_id=cluster_id,
                predecessor_id=predecessor_robot,
                successor_id=successor_robot,
                previous_waypoint=waypoint,
                projection_m=details['projection_m'],
                assignment_revision=self._assignment_revision,
                role=details['role'],
                connected_to_station=details['connected_to_station'],
            )
        for robot_id in set(snapshot.robots) - set(self._agents):
            self._agents[robot_id] = LocalRelayState(
                previous_waypoint=setpoints[robot_id]
            )
        diagnostics = {
            'schedule_type': 'reactive_periodic',
            'reactive_computation_count': self._compute_count,
            'cluster_revision': self._cluster_revision,
            'assignment_revision': self._assignment_revision,
            'safe_link_distance_m': self.safe_link_distance_m,
            'clusters': {
                cluster.cluster_id: {
                    'target_ids': cluster.target_ids,
                    'destination_target_id': cluster.destination_target_id,
                    'position': cluster.position,
                    'priority': cluster.priority,
                    'required_robots': cluster.required_robots,
                }
                for cluster in self._clusters
            },
            'local_agents': local_diagnostics,
            'locally_computed_waypoints': dict(setpoints),
            'ordering_model': 'station_target_projection_with_hysteresis',
        }
        return ControllerResult(
            setpoints=setpoints,
            target_assignments={
                robot_id: clusters[cluster_id].destination_target_id
                for robot_id, cluster_id in assignment.items()
                if cluster_id in clusters
            },
            selected_edges=tuple(selected_edges),
            solver_status='decentralized_hybrid_local_update',
            solve_duration_sec=time.monotonic() - started,
            diagnostic=(
                'Directional target clustering and persistent allocation are '
                'global; relay updates use local same-cluster neighbors only.'
            ),
            created_at=time.monotonic(),
            target_epoch=snapshot.target_epoch,
            controller_diagnostics=diagnostics,
        )
