import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def package_root():
    return Path(__file__).parents[1]


def test_coordinator_uses_existing_interfaces_and_never_cmd_vel():
    source = (
        package_root()
        / 'waverover_swarm_controller'
        / 'coordinator_node.py'
    ).read_text(encoding='utf-8')

    assert 'load_stack_config(require_identity=False)' in source
    assert "robot_topic(stack_config, 'waypoints'" in source
    assert "robot_topic(stack_config, 'end_trial'" in source
    assert 'mcs_pose_topic(stack_config' in source
    assert 'cmd_vel' not in source


def test_launch_is_standalone_dry_run_by_default():
    launch_path = package_root() / 'launch' / 'swarm_controller.launch.py'
    spec = importlib.util.spec_from_file_location('swarm_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()

    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]
    assert arguments['dry_run'].default_value[0].text == 'true'
    assert arguments['algorithm'].default_value[0].text == 'heuristic'
    assert len(nodes) == 1
    assert nodes[0]._Node__package == 'waverover_swarm_controller'
