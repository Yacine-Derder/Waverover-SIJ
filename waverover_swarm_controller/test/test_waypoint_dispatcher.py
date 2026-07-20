from dataclasses import replace

import pytest

from waverover_swarm_controller.waypoint_dispatcher import WaypointDispatcher


def test_dispatcher_keeps_one_active_and_replaces_only_pending(
    example_config, snapshot
):
    dispatcher = WaypointDispatcher(
        snapshot.robots, example_config.waypoint_dispatch
    )
    dispatcher.update_pending({key: (1.0, 1.0) for key in snapshot.robots})
    actions = dispatcher.tick(snapshot, 20.0, commands_enabled=True)
    assert len(actions) == 3
    assert all(state.active_waypoint == (1.0, 1.0)
               for state in dispatcher.states.values())

    dispatcher.update_pending({key: (2.0, 2.0) for key in snapshot.robots})
    dispatcher.update_pending({key: (3.0, 3.0) for key in snapshot.robots})
    assert dispatcher.tick(snapshot, 20.1, commands_enabled=True) == []
    assert all(state.pending_waypoint == (3.0, 3.0)
               for state in dispatcher.states.values())


def test_reached_waypoint_waits_for_handoff_delay(example_config, snapshot):
    config = replace(
        example_config.waypoint_dispatch,
        reached_distance_m=0.2,
        handoff_delay_sec=0.15,
    )
    dispatcher = WaypointDispatcher(snapshot.robots, config)
    first = {key: state.position for key, state in snapshot.robots.items()}
    dispatcher.update_pending(first)
    dispatcher.tick(snapshot, 10.0, commands_enabled=True)
    dispatcher.update_pending({key: (2.0, 0.0) for key in snapshot.robots})

    assert dispatcher.tick(snapshot, 10.1, commands_enabled=True) == []
    assert dispatcher.tick(snapshot, 10.24, commands_enabled=True) == []
    actions = dispatcher.tick(snapshot, 10.26, commands_enabled=True)
    assert len(actions) == 3
    assert all(action.point == (2.0, 0.0) for action in actions)


def test_dry_run_never_marks_rover_commanded(example_config, snapshot):
    dispatcher = WaypointDispatcher(
        snapshot.robots, example_config.waypoint_dispatch
    )
    dispatcher.update_pending({key: (1.0, 1.0) for key in snapshot.robots})

    assert dispatcher.tick(snapshot, 10.0, commands_enabled=False) == []
    assert dispatcher.commanded_robot_ids == ()


def test_session_history_contains_only_actually_dispatched_rovers(
    example_config, snapshot
):
    dispatcher = WaypointDispatcher(
        snapshot.robots, example_config.waypoint_dispatch
    )
    dispatcher.update_pending({'robot_2': (1.0, 1.0)})
    dispatcher.tick(snapshot, 10.0, commands_enabled=True)
    dispatcher.stop()
    dispatcher.stop()

    assert dispatcher.commanded_robot_ids == ('robot_2',)


def test_refresh_republishes_active_without_replacing_it_or_resetting_state(
    example_config, snapshot
):
    config = replace(
        example_config.waypoint_dispatch,
        refresh_period_sec=1.0,
        handoff_delay_sec=2.0,
    )
    dispatcher = WaypointDispatcher(snapshot.robots, config)
    first = {key: state.position for key, state in snapshot.robots.items()}
    dispatcher.update_pending(first)
    dispatcher.tick(snapshot, 10.0, commands_enabled=True)
    dispatcher.update_pending({key: (3.0, 3.0) for key in snapshot.robots})

    assert dispatcher.tick(snapshot, 10.5, commands_enabled=True) == []
    actions = dispatcher.tick(snapshot, 11.0, commands_enabled=True)

    assert len(actions) == 3
    assert all(action.kind == 'refresh' for action in actions)
    assert all(action.point == first[action.robot_id] for action in actions)
    for robot_id, state in dispatcher.states.items():
        assert state.active_waypoint == first[robot_id]
        assert state.pending_waypoint == (3.0, 3.0)
        assert state.active_published_at == 10.0
        assert state.last_published_at == 11.0
        assert state.reached_since == 10.5
        assert state.refresh_count == 1


def test_overdue_active_waypoint_warns_but_refreshes_without_fault(
    example_config, snapshot
):
    dispatcher = WaypointDispatcher(
        snapshot.robots,
        replace(
            example_config.waypoint_dispatch,
            refresh_period_sec=1.0,
            active_waypoint_warning_sec=1.0,
        ),
    )
    dispatcher.update_pending({key: (3.0, 3.0) for key in snapshot.robots})
    dispatcher.tick(snapshot, 10.0, commands_enabled=True)
    actions = dispatcher.tick(snapshot, 11.1, commands_enabled=True)

    assert len(actions) == 3
    assert all(action.kind == 'refresh' for action in actions)
    assert not dispatcher.faulted
    observed = dispatcher.observability(11.1)
    assert all(value['active_waypoint_overdue'] for value in observed.values())
    assert all(value['active_waypoint_age_sec'] == pytest.approx(1.1)
               for value in observed.values())
    assert all(value['last_publication_age_sec'] == 0.0
               for value in observed.values())
