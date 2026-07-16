import pytest

from waverover_waypoint_ui.waypoint_ui import (
    map_frame,
    parse_terminal_command,
    validate_robot_id,
    waypoint_frame,
    waypoint_topic,
)


def test_robot_topic_and_frame_derivation():
    assert waypoint_topic('29') == '/robot_29/waypoints'
    assert waypoint_topic('30') == '/robot_30/waypoints'
    assert map_frame('29') == 'robot_29/map'
    assert waypoint_frame('30', 'SLAM') == 'robot_30/map'
    assert waypoint_frame('30', 'MCS') == 'robotics_lab'


@pytest.mark.parametrize('value', ['', '29/other', '29-other', '  '])
def test_invalid_robot_ids_are_rejected(value):
    with pytest.raises(ValueError):
        validate_robot_id(value)


def test_robot_id_is_trimmed():
    assert validate_robot_id(' 29 ') == '29'


def test_two_coordinates_use_current_robot():
    assert parse_terminal_command('1.25 -0.5', '29') == (
        'send',
        '29',
        1.25,
        -0.5,
    )


def test_robot_and_coordinates_can_be_entered_together():
    assert parse_terminal_command('30, 2, 3.5', '29') == (
        'send',
        '30',
        2.0,
        3.5,
    )


@pytest.mark.parametrize('command', ['robot 30', 'use 30', '30'])
def test_robot_selection_commands(command):
    assert parse_terminal_command(command, '29') == ('robot', '30')


@pytest.mark.parametrize('command', ['nan 1', '1 inf', '1 2 3 4'])
def test_invalid_terminal_waypoints_are_rejected(command):
    with pytest.raises(ValueError):
        parse_terminal_command(command, '29')
