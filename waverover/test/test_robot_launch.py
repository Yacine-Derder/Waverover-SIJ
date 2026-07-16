import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


def load_robot_launch_module():
    path = Path(__file__).parents[1] / 'launch' / 'robot.launch.py'
    spec = importlib.util.spec_from_file_location('robot_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_includes(module, pose_source, control_mode='fixed_wing'):
    captured = []

    def fake_include(package_name, launch_file, arguments):
        result = (package_name, launch_file, arguments)
        captured.append(result)
        return result

    module._include = fake_include
    context = LaunchContext()
    context.launch_configurations.update({
        'robot_name': '29',
        'control_mode': control_mode,
        'pose_source': pose_source,
    })
    module._launch_onboard_stack(context)
    return captured


def test_slam_mode_selects_slam_pipeline_and_waypoint_controller():
    module = load_robot_launch_module()

    includes = launch_includes(module, 'SLAM')

    assert [(item[0], item[1]) for item in includes] == [
        ('waverover', 'slam.launch.py'),
        ('waverover_controller', 'waypoint_controller.launch.py'),
    ]
    assert module.selected_onboard_components('SLAM') == (
        'lidar',
        'static_tf',
        'rf2o',
        'slam',
        'bridge',
        'waypoint_controller',
    )


def test_mcs_mode_selects_only_bridge_and_waypoint_controller():
    module = load_robot_launch_module()

    includes = launch_includes(module, 'MCS')

    assert [(item[0], item[1]) for item in includes] == [
        ('ros2waverover', 'wave_rover_launch.py'),
        ('waverover_controller', 'waypoint_controller.launch.py'),
    ]
    assert module.selected_onboard_components('MCS') == (
        'bridge',
        'waypoint_controller',
    )


def test_manual_lr_mode_does_not_start_an_incompatible_controller():
    module = load_robot_launch_module()

    includes = launch_includes(module, 'MCS', 'manual_lr')

    assert [(item[0], item[1]) for item in includes] == [
        ('ros2waverover', 'wave_rover_launch.py'),
    ]


def test_onboard_launch_defaults_to_explicit_test_identity(
    tmp_path,
    monkeypatch,
):
    identity = tmp_path / 'identity.yaml'
    identity.write_text('robot_name: "test_131"\n', encoding='utf-8')
    monkeypatch.setenv('WAVEROVER_IDENTITY_FILE', str(identity))
    module = load_robot_launch_module()
    declaration = next(
        action
        for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
        and action.name == 'robot_name'
    )
    context = LaunchContext()
    assert ''.join(
        item.perform(context) for item in declaration.default_value
    ) == 'test_131'
