import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


def test_robot_name_launch_parameter_stays_a_string():
    launch_path = (
        Path(__file__).parents[1] / 'launch' / 'waypoint_ui.launch.py'
    )
    spec = importlib.util.spec_from_file_location(
        'waypoint_ui_launch',
        launch_path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    context = LaunchContext()
    context.launch_configurations['robot_name'] = '29'
    context.launch_configurations['pose_source'] = 'MCS'
    context.launch_configurations['terminal_device'] = ''
    node = next(
        entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, Node)
    )
    parameters = evaluate_parameters(context, node._Node__parameters)[0]

    assert parameters['robot_name'] == '29'
    assert isinstance(parameters['robot_name'], str)
    assert parameters['pose_source'] == 'MCS'
    assert isinstance(parameters['terminal_device'], str)

    robot_argument = next(
        entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
        and entity.name == 'robot_name'
    )
    assert robot_argument.default_value is None
