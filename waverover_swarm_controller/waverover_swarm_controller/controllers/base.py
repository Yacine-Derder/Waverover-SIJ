"""Common interface and helpers for pure swarm controllers."""

from abc import ABC, abstractmethod
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


def controller_from_config(config):
    algorithm = config.controller.algorithm
    if algorithm == 'heuristic':
        from .heuristic import HeuristicController
        return HeuristicController(config)
    if algorithm == 'heuristic_decentralized':
        from .heuristic_decentralized import DecentralizedHeuristicController
        return DecentralizedHeuristicController(config)
    if algorithm == 'convex':
        from .convex import ConvexController
        return ConvexController(config)
    if algorithm == 'mpc_centralized':
        from .mpc_centralized import CentralizedMpcController
        return CentralizedMpcController(config)
    if algorithm == 'mpc_distributed':
        from .mpc_distributed import DistributedMpcController
        return DistributedMpcController(config)
    raise ControllerUnavailableError('Unknown controller %s.' % algorithm)
