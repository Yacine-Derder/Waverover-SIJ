"""Common interface and helpers for pure swarm controllers."""

from abc import ABC, abstractmethod
from enum import Enum
import importlib.util
import math


class ControllerUnavailableError(RuntimeError):
    """Raised when an explicitly selected controller cannot run."""


class InvalidControllerResult(RuntimeError):
    """Raised when a controller generates an unsafe numeric result."""


class ControllerSchedule(Enum):
    """Authoritative controller output meaning and recomputation policy."""

    FINAL_DESTINATION = 'final_destination_event_driven'
    RECEDING_HORIZON = 'receding_horizon_periodic'


CONTROLLER_SCHEDULES = {
    'heuristic': ControllerSchedule.FINAL_DESTINATION,
    'heuristic_decentralized': ControllerSchedule.FINAL_DESTINATION,
    'convex': ControllerSchedule.FINAL_DESTINATION,
    'mpc_centralized': ControllerSchedule.RECEDING_HORIZON,
    'mpc_distributed': ControllerSchedule.RECEDING_HORIZON,
}
CONTROLLER_OUTCOME_MODES = {
    'heuristic': None,
    'heuristic_decentralized': None,
    'convex': ('normal_convex', 'recovery_convex'),
    'mpc_centralized': ('normal_mpc', 'recovery_mpc'),
    'mpc_distributed': ('normal_mpc', 'recovery_mpc'),
}


def controller_schedule(algorithm):
    try:
        return CONTROLLER_SCHEDULES[str(algorithm)]
    except KeyError as error:
        raise ControllerUnavailableError(
            'Unknown controller %s.' % algorithm
        ) from error


def controller_outcome_modes(algorithm):
    controller_schedule(algorithm)
    return CONTROLLER_OUTCOME_MODES[str(algorithm)]


class SwarmController(ABC):
    def __init__(self, config):
        self.config = config

    def availability(self):
        return True, ''

    @abstractmethod
    def compute(self, snapshot):
        """Compute one immutable result without publishing ROS messages."""


def optional_dependency(module_name, purpose):
    if importlib.util.find_spec(module_name) is None:
        return False, '%s requires missing Python module %s.' % (
            purpose,
            module_name,
        )
    return True, ''


def finite_point(point):
    return (
        isinstance(point, (tuple, list))
        and len(point) == 2
        and all(math.isfinite(float(value)) for value in point)
    )


def optimization_hard_link_limit(config):
    """Return the unchanged convex/MPC connectivity constraint radius."""
    return (
        config.communication.maximum_range_m
        - 2.0 * config.vehicle.turn_radius_m
    )


def heuristic_snap_link_limit(config):
    """Return the simulator ConnectedDrone carrot projection radius."""
    return config.communication.maximum_range_m - config.vehicle.turn_radius_m


def complete_finite_mapping(snapshot, result):
    return (
        result is not None
        and set(result.setpoints) == set(snapshot.robots)
        and all(finite_point(point) for point in result.setpoints.values())
    )


def deterministic_connectivity_setpoints(config, snapshot, edges):
    """Combine all violated-edge corrections from one immutable snapshot."""
    station_id = snapshot.station.station_id
    valid = set(snapshot.robots) | {station_id}
    canonical = tuple(sorted({
        tuple(sorted((str(first), str(second))))
        for first, second in edges
        if first != second and first in valid and second in valid
    }))
    positions = {
        robot_id: snapshot.robots[robot_id].position
        for robot_id in sorted(snapshot.robots)
    }
    positions[station_id] = snapshot.station.position
    limit = optimization_hard_link_limit(config)
    output = {}
    for robot_id in sorted(snapshot.robots):
        corrections = []
        for first, second in canonical:
            if robot_id not in (first, second):
                continue
            neighbor_id = second if first == robot_id else first
            own = positions[robot_id]
            neighbor = positions[neighbor_id]
            dx = neighbor[0] - own[0]
            dy = neighbor[1] - own[1]
            distance = math.hypot(dx, dy)
            violation = distance - limit
            if violation > 1e-12 and distance > 1e-12:
                corrections.append((
                    neighbor_id,
                    violation * dx / distance,
                    violation * dy / distance,
                    violation,
                ))
        if not corrections:
            output[robot_id] = positions[robot_id]
            continue
        dx = sum(value[1] for value in corrections)
        dy = sum(value[2] for value in corrections)
        norm = math.hypot(dx, dy)
        if norm <= 1e-12:
            _, dx, dy, norm = max(
                corrections, key=lambda value: (value[3], value[0])
            )
        travel = (
            min(config.controller.mpc_max_step_m, norm)
            if controller_schedule(config.controller.algorithm)
            is ControllerSchedule.RECEDING_HORIZON else norm
        )
        output[robot_id] = (
            positions[robot_id][0] + travel * dx / norm,
            positions[robot_id][1] + travel * dy / norm,
        )
    return output, canonical


def minimum_lookahead(robot, point, minimum_distance, maximum_step):
    dx = float(point[0]) - robot.x
    dy = float(point[1]) - robot.y
    distance = math.hypot(dx, dy)
    if not math.isfinite(distance):
        raise InvalidControllerResult(
            'Cannot construct an MPC carrot from a non-finite direction.'
        )
    if distance <= 1e-12:
        return robot.position
    requested = min(max(distance, minimum_distance), maximum_step)
    scale = requested / distance
    return (robot.x + scale * dx, robot.y + scale * dy)


def replace_first_future_points(predicted_paths, setpoints):
    """Return paths whose first future point is exactly the dispatched point."""
    output = {}
    for robot_id, path in predicted_paths.items():
        points = list(path)
        if len(points) >= 2 and robot_id in setpoints:
            points[1] = tuple(float(value) for value in setpoints[robot_id])
        output[robot_id] = tuple(points)
    return output


def controller_from_config(config):
    algorithm = config.controller.algorithm
    if algorithm == 'heuristic':
        from .heuristic import HeuristicController
        controller = HeuristicController(config)
    elif algorithm == 'heuristic_decentralized':
        from .heuristic_decentralized import DecentralizedHeuristicController
        controller = DecentralizedHeuristicController(config)
    elif algorithm == 'convex':
        from .convex import ConvexController
        controller = ConvexController(config)
    elif algorithm == 'mpc_centralized':
        from .mpc_centralized import CentralizedMpcController
        controller = CentralizedMpcController(config)
    elif algorithm == 'mpc_distributed':
        from .mpc_distributed import DistributedMpcController
        controller = DistributedMpcController(config)
    else:
        raise ControllerUnavailableError('Unknown controller %s.' % algorithm)
    return controller
