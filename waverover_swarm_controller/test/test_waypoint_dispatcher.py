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


def test_only_matching_acknowledgement_hands_off(example_config, snapshot):
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

    assert dispatcher.tick(snapshot, 10.26, commands_enabled=True) == []
    state = dispatcher.states['robot_2']
    assert dispatcher.acknowledge(
        'robot_2', 'wrong', state.active_token, state.active_waypoint,
        10.27, snapshot.frame_id,
    ) == []
    actions = []
    for robot_id, state in dispatcher.states.items():
        actions.extend(dispatcher.acknowledge(
            robot_id, snapshot.frame_id, state.active_token,
            state.active_waypoint, 10.28, snapshot.frame_id,
        ))
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


def test_completed_destination_hysteresis_and_real_drift(example_config, snapshot):
    dispatcher = WaypointDispatcher(snapshot.robots, example_config.waypoint_dispatch)
    point = snapshot.robots['robot_2'].position
    dispatcher.update_pending({'robot_2': point}, target_epoch=7)
    first = dispatcher.tick(snapshot, 1.0, True)[0]
    dispatcher.acknowledge(
        'robot_2', snapshot.frame_id, first.token, point, 1.1,
        snapshot.frame_id, measured_position=point,
    )
    dispatcher.update_pending({'robot_2': point}, target_epoch=7)
    assert dispatcher.tick(snapshot, 1.2, True) == []
    assert dispatcher.states['robot_2'].suppression_reason == (
        'completed_destination_hold_continuation'
    )

    drifted = replace(
        snapshot,
        robots={
            **snapshot.robots,
            'robot_2': replace(snapshot.robots['robot_2'], x=point[0] + 0.31),
        },
    )
    dispatcher.update_pending({'robot_2': point}, target_epoch=7)
    assert dispatcher.tick(drifted, 1.3, True)[0].token != first.token


def test_waypoint_failed_requires_exact_token_frame_and_coordinates(
    example_config, snapshot
):
    dispatcher = WaypointDispatcher(snapshot.robots, example_config.waypoint_dispatch)
    dispatcher.update_pending({'robot_2': (1.0, 1.0)}, target_epoch=2)
    action = dispatcher.tick(snapshot, 1.0, True)[0]
    assert not dispatcher.fail(
        'robot_2', snapshot.frame_id, (99, 1), action.point, 1.1,
        snapshot.frame_id,
    )
    assert dispatcher.fail(
        'robot_2', snapshot.frame_id, action.token, action.point, 1.2,
        snapshot.frame_id,
    )
    dispatcher.update_pending({'robot_2': action.point}, target_epoch=2)
    assert dispatcher.tick(snapshot, 1.3, True) == []
    assert dispatcher.states['robot_2'].failure_count == 1


def test_reactive_batch_supersedes_active_with_new_tokens_and_ignores_old_ack(
    example_config, snapshot
):
    dispatcher = WaypointDispatcher(
        ('a', 'b'), example_config.waypoint_dispatch, example_config.safety
    )
    dispatcher.update_pending(
        {'a': (0.0, 0.0), 'b': (1.0, 0.0)},
        objective_revision=1, command_revision=1,
        allow_supersession=True, waypoint_deadband_m=0.05,
    )
    first = dispatcher.tick(snapshot, 1.0, True)
    old = {action.robot_id: action for action in first}
    dispatcher.update_pending(
        {'a': (0.5, 0.0), 'b': (1.5, 0.0)},
        objective_revision=1, command_revision=2,
        allow_supersession=True, waypoint_deadband_m=0.05,
    )
    replacements = dispatcher.tick(snapshot, 1.1, True)
    new = {action.robot_id: action for action in replacements}

    assert len(replacements) == 2
    assert all(action.reason == 'reactive_supersession'
               for action in replacements)
    assert all(new[key].token != old[key].token for key in new)
    assert dispatcher.acknowledge(
        'a', 'robotics_lab', old['a'].token, old['a'].point,
        1.2, 'robotics_lab',
    ) == []
    assert dispatcher.states['a'].active_token == new['a'].token
    assert dispatcher.states['a'].superseded_acknowledgement_count == 1


def test_reactive_deadband_retains_active_token_and_cancels_older_pending(
    example_config, snapshot
):
    dispatcher = WaypointDispatcher(
        ('a',), example_config.waypoint_dispatch, example_config.safety
    )
    dispatcher.update_pending(
        {'a': (1.0, 0.0)}, objective_revision=1, command_revision=1,
        allow_supersession=True, waypoint_deadband_m=0.05,
    )
    first = dispatcher.tick(snapshot, 1.0, True)[0]
    dispatcher.update_pending(
        {'a': (1.2, 0.0)}, objective_revision=1, command_revision=2,
        allow_supersession=True, waypoint_deadband_m=0.05,
    )
    dispatcher.update_pending(
        {'a': (1.01, 0.0)}, objective_revision=1, command_revision=3,
        allow_supersession=True, waypoint_deadband_m=0.05,
    )

    assert dispatcher.tick(snapshot, 1.1, True) == []
    state = dispatcher.states['a']
    assert state.active_token == first.token
    assert state.active_waypoint == first.point
    assert state.pending_waypoint is None
    assert state.retained_due_to_deadband
    assert state.deadband_retention_count == 1


def test_failed_reactive_replacement_batch_keeps_every_active_command(
    example_config, snapshot
):
    dispatcher = WaypointDispatcher(
        ('a', 'b'), example_config.waypoint_dispatch, example_config.safety
    )
    dispatcher.update_pending(
        {'a': (0.0, 0.0), 'b': (1.0, 0.0)},
        objective_revision=1, command_revision=1,
        allow_supersession=True, waypoint_deadband_m=0.05,
    )
    first = dispatcher.tick(snapshot, 1.0, True)
    old = {action.robot_id: action for action in first}
    dispatcher.safety_config = replace(
        example_config.safety,
        geofence=replace(
            example_config.safety.geofence,
            x_min=0.0, x_max=0.1, y_min=0.0, y_max=0.1,
        ),
    )
    dispatcher.update_pending(
        {'a': (0.0, 0.0), 'b': (0.1, 0.1)},
        objective_revision=1, command_revision=2,
        allow_supersession=True, waypoint_deadband_m=0.0,
    )

    assert dispatcher.tick(snapshot, 1.1, True) == []
    assert dispatcher.last_activation_failure.startswith(
        'outgoing_waypoint_separation_failed'
    )
    assert {
        key: dispatcher.states[key].active_token for key in ('a', 'b')
    } == {key: old[key].token for key in ('a', 'b')}


def test_reactive_repeat_after_endpoint_repair_retains_token(
    example_config, snapshot
):
    dispatcher = WaypointDispatcher(
        ('a', 'b'), example_config.waypoint_dispatch, example_config.safety
    )
    requested = {'a': (0.0, 0.0), 'b': (0.1, 0.0)}
    dispatcher.update_pending(
        requested, objective_revision=1, command_revision=1,
        allow_supersession=True, waypoint_deadband_m=0.05,
    )
    first = dispatcher.tick(snapshot, 1.0, True)
    tokens = {action.robot_id: action.token for action in first}
    assert any(action.point != requested[action.robot_id] for action in first)

    dispatcher.update_pending(
        requested, objective_revision=1, command_revision=2,
        allow_supersession=True, waypoint_deadband_m=0.05,
    )
    assert dispatcher.tick(snapshot, 1.1, True) == []
    assert {
        robot_id: state.active_token
        for robot_id, state in dispatcher.states.items()
    } == tokens


def test_stale_reactive_revision_cannot_replace_or_refresh_newest(
    example_config, snapshot
):
    dispatcher = WaypointDispatcher(
        ('a',), replace(
            example_config.waypoint_dispatch, refresh_period_sec=1.0
        )
    )
    dispatcher.update_pending(
        {'a': (1.0, 0.0)}, objective_revision=2, command_revision=5,
        allow_supersession=True, waypoint_deadband_m=0.05,
    )
    newest = dispatcher.tick(snapshot, 1.0, True)[0]
    dispatcher.update_pending(
        {'a': (2.0, 0.0)}, objective_revision=2, command_revision=4,
        allow_supersession=True, waypoint_deadband_m=0.05,
    )

    refresh = dispatcher.tick(snapshot, 2.0, True)
    assert len(refresh) == 1
    assert refresh[0].kind == 'refresh'
    assert refresh[0].token == newest.token
    assert refresh[0].point == newest.point


def test_stop_clears_reactive_commands_and_superseded_cache(
    example_config, snapshot
):
    dispatcher = WaypointDispatcher(
        ('a',), example_config.waypoint_dispatch
    )
    dispatcher.update_pending(
        {'a': (1.0, 0.0)}, command_revision=1,
        allow_supersession=True, waypoint_deadband_m=0.05,
    )
    dispatcher.tick(snapshot, 1.0, True)
    dispatcher.update_pending(
        {'a': (2.0, 0.0)}, command_revision=2,
        allow_supersession=True, waypoint_deadband_m=0.05,
    )
    dispatcher.tick(snapshot, 1.1, True)
    assert dispatcher.states['a'].superseded_tokens

    dispatcher.stop()
    state = dispatcher.states['a']
    assert state.active_waypoint is None
    assert state.pending_waypoint is None
    assert state.active_token is None
    assert state.superseded_tokens == []
