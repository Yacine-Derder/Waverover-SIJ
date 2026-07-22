"""Immutable snapshots exchanged by pure swarm-control components."""

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping, Optional, Tuple


Point2D = Tuple[float, float]
Path2D = Tuple[Point2D, ...]
Edge = Tuple[str, str]


def _frozen_mapping(values):
    return MappingProxyType(dict(sorted(values.items())))


@dataclass(frozen=True)
class RobotState:
    robot_id: str
    x: float
    y: float
    yaw: float
    receipt_time: float
    message_time: Optional[float] = None

    @property
    def position(self):
        return (self.x, self.y)


@dataclass(frozen=True, init=False)
class TargetState:
    target_id: str
    x: float
    y: float
    weight: float
    is_priority: bool = False

    def __init__(
        self, target_id, x, y, weight, is_priority=False, **legacy
    ):
        if 'is_main' in legacy:
            is_priority = legacy.pop('is_main')
        if legacy:
            raise TypeError('Unexpected TargetState fields: %s' % sorted(legacy))
        object.__setattr__(self, 'target_id', str(target_id))
        object.__setattr__(self, 'x', float(x))
        object.__setattr__(self, 'y', float(y))
        object.__setattr__(self, 'weight', float(weight))
        object.__setattr__(self, 'is_priority', bool(is_priority))

    @property
    def position(self):
        return (self.x, self.y)

    @property
    def is_main(self):
        """Legacy reader compatibility; new runtime code uses is_priority."""
        return self.is_priority


@dataclass(frozen=True)
class StationState:
    station_id: str
    x: float
    y: float

    @property
    def position(self):
        return (self.x, self.y)


@dataclass(frozen=True)
class SwarmSnapshot:
    frame_id: str
    robots: Mapping[str, RobotState]
    targets: Mapping[str, TargetState]
    station: StationState
    created_at: float
    priority_target_id: Optional[str] = None
    target_epoch: int = 0
    target_epoch_started_at: float = 0.0
    target_switch_reason: str = 'legacy_static'
    target_selection_seed: Optional[int] = None
    next_target_switch_at: Optional[float] = None

    def __post_init__(self):
        object.__setattr__(self, 'robots', _frozen_mapping(self.robots))
        object.__setattr__(self, 'targets', _frozen_mapping(self.targets))


@dataclass(frozen=True)
class ControllerResult:
    setpoints: Mapping[str, Point2D]
    target_assignments: Mapping[str, str] = field(default_factory=dict)
    predicted_paths: Mapping[str, Path2D] = field(default_factory=dict)
    selected_edges: Tuple[Edge, ...] = ()
    solver_status: str = 'not_run'
    solve_duration_sec: float = 0.0
    diagnostic: str = ''
    created_at: float = 0.0
    target_epoch: int = 0
    collision_repair: Mapping = field(default_factory=dict)
    controller_diagnostics: Mapping = field(default_factory=dict)
    optimization_mode: str = ''

    def __post_init__(self):
        object.__setattr__(self, 'setpoints', _frozen_mapping(self.setpoints))
        object.__setattr__(
            self,
            'target_assignments',
            _frozen_mapping({
                str(robot_id): str(target_id)
                for robot_id, target_id in self.target_assignments.items()
            }),
        )
        object.__setattr__(
            self,
            'predicted_paths',
            _frozen_mapping(self.predicted_paths),
        )
        object.__setattr__(self, 'collision_repair', _frozen_mapping(
            self.collision_repair
        ))
        object.__setattr__(self, 'controller_diagnostics', _frozen_mapping(
            self.controller_diagnostics
        ))
        canonical_edges = tuple(
            sorted((str(first), str(second)) for first, second in self.selected_edges)
        )
        object.__setattr__(self, 'selected_edges', canonical_edges)
