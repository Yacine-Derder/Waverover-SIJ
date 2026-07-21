from waverover.health_monitor import WatchdogState


def test_watchdog_grace_consecutive_failures_and_safe_states():
    state = WatchdogState(0.0)
    thresholds = {'cmd_vel': 2.0, 'imu': 2.0}
    assert state.evaluate(10.0, 15.0, thresholds, 'fixed_wing', None, None, True) is None
    assert state.evaluate(
        16.0, 15.0, thresholds, 'fixed_wing', None, 0.1, True,
        'waiting_first_waypoint',
    ) is None
    assert state.evaluate(
        17.0, 15.0, thresholds, 'fixed_wing', None, 0.1, True
    ) == ['cmd_vel_stale']
    assert state.consecutive_failures == 1


def test_manual_mode_and_external_mcs_are_not_restart_predicates():
    state = WatchdogState(0.0)
    assert state.evaluate(
        20.0, 1.0, {'cmd_vel': 2.0, 'imu': 2.0},
        'manual_lr', None, 0.1, True,
    ) is None
