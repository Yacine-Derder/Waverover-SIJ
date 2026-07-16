import importlib.util
from pathlib import Path

from launch import LaunchContext
import pytest


def _load_slam_launch_module():
    launch_path = Path(__file__).parents[1] / 'launch' / 'slam.launch.py'
    spec = importlib.util.spec_from_file_location(
        'waverover_slam_launch',
        launch_path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_slam_only_launch_redirects_mcs_to_unified_launch():
    module = _load_slam_launch_module()
    context = LaunchContext()
    context.launch_configurations.update({
        'robot_name': '29',
        'control_mode': 'fixed_wing',
        'pose_source': 'MCS',
    })

    with pytest.raises(RuntimeError, match='robot.launch.py'):
        module._launch_robot(context)
