"""Common interface and helpers for pure swarm controllers."""

from abc import ABC, abstractmethod
from dataclasses import asdict, replace
import importlib.util
import math


class ControllerUnavailableError(RuntimeError):
    """Raised when an explicitly selected controller cannot run."""


class InvalidControllerResult(RuntimeError):
    """Raised when a controller generates an unsafe numeric result."""


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


def minimum_lookahead(robot, point, minimum_distance, maximum_step):
    dx = float(point[0]) - robot.x
    dy = float(point[1]) - robot.y
    distance = math.hypot(dx, dy)
    if not math.isfinite(distance) or distance <= 1e-12:
        raise InvalidControllerResult(
            'Cannot construct an MPC carrot from a zero/non-finite direction.'
        )
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
    return BestEffortRepairController(controller, config)


class BestEffortRepairController:
    """Apply identical final preferred-separation behavior to every solver."""

    def __init__(self, controller, config):
        object.__setattr__(self, '_controller', controller)
        object.__setattr__(self, 'config', config)

    def __getattr__(self, name):
        return getattr(self._controller, name)

    def __setattr__(self, name, value):
        if name in ('_controller', 'config'):
            object.__setattr__(self, name, value)
        else:
            setattr(self._controller, name, value)

    def compute(self, snapshot):
        from ..waypoint_repair import repair_waypoints

        result = self._controller.compute(snapshot)
        repaired, report = repair_waypoints(
            result.setpoints, {}, self.config.safety.geofence,
            self.config.safety.preferred_separation_m,
            self.config.safety.collision_repair_max_iterations,
            snapshot.target_epoch,
        )
        metadata = asdict(report)
        metadata['predicted_paths_after_first_step'] = 'pre_repair'
        return replace(
            result,
            setpoints=repaired,
            predicted_paths=replace_first_future_points(
                result.predicted_paths, repaired
            ),
            collision_repair=metadata,
        )
