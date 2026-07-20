from dataclasses import replace

from waverover_swarm_controller.models import TargetState
from waverover_swarm_controller.target_dynamics import TargetManager


def targets(order=('target_2', 'target_0', 'target_1')):
    return tuple(
        TargetState(target_id, index, 0.0, 1.0)
        for index, target_id in enumerate(order)
    )


def sequence(config, order=('target_2', 'target_0', 'target_1')):
    manager = TargetManager(targets(order), config, start_time=100.0)
    return [
        manager.snapshot(now)[1]['priority_target_id']
        for now in (100.0, 110.0, 120.0, 150.0)
    ]


def test_priority_sequence_is_deterministic_order_independent(example_config):
    config = example_config.target_dynamics
    first = sequence(config)
    second = sequence(config, tuple(reversed(('target_2', 'target_0', 'target_1'))))
    assert first == second
    assert all(left != right for left, right in zip(first, first[1:]))


def test_missed_boundaries_advance_rng_and_one_target_is_stable(example_config):
    config = replace(
        example_config.target_dynamics,
        initial_priority_target_id=None,
        switch_period_sec=10.0,
    )
    direct = TargetManager(targets(), config, start_time=0.0)
    direct.snapshot(0.0)
    caught_up = direct.snapshot(35.0)[1]
    stepped = TargetManager(targets(), config, start_time=0.0)
    for now in (0.0, 10.0, 20.0, 30.0):
        state = stepped.snapshot(now)[1]
    assert caught_up['priority_target_id'] == state['priority_target_id']
    assert caught_up['target_epoch'] == 3

    only = TargetManager((TargetState('target_0', 0, 0, 1),), config)
    assert only.snapshot(100.0)[1]['priority_target_id'] == 'target_0'
