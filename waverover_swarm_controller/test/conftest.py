from dataclasses import replace
from pathlib import Path

import pytest

from waverover_swarm_controller.config import load_experiment
from waverover_swarm_controller.models import (
    RobotState,
    StationState,
    SwarmSnapshot,
    TargetState,
)


@pytest.fixture
def example_config():
    path = Path(__file__).parents[1] / 'config' / 'experiment.yaml'
    config = load_experiment(path)
    return replace(config, robot_ids=('robot_2', 'robot_10', 'robot_30'))


@pytest.fixture
def snapshot():
    robots = {
        'robot_2': RobotState('robot_2', 0.25, 0.10, 0.0, 10.0),
        'robot_10': RobotState('robot_10', 0.30, 0.75, 0.2, 10.0),
        'robot_30': RobotState('robot_30', 0.20, -0.70, -0.2, 10.0),
    }
    targets = {
        'main_target': TargetState(
            'main_target', 2.5, 0.0, 10.0, is_main=True
        ),
        'secondary_z': TargetState('secondary_z', 0.0, 2.5, 1.0),
    }
    return SwarmSnapshot(
        frame_id='robotics_lab',
        robots=robots,
        targets=targets,
        station=StationState('station_0', 0.0, 0.0),
        created_at=10.0,
    )
