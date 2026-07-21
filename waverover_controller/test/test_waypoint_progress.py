from waverover_controller.waypoint_controller import WaypointProgress


def test_bounded_recovery_retains_token_and_eventually_fails():
    progress = WaypointProgress((4, 5), 0.25, 0.25, 0.0)
    assert progress.update(0.25, 3.0, 3.0, 0.03, 0.75, 2) == 'start_recovery'
    assert progress.token == (4, 5)
    assert progress.update(0.25, 3.5, 3.0, 0.03, 0.75, 2) == 'straight_escape'
    assert progress.update(0.25, 3.8, 3.0, 0.03, 0.75, 2) == 'resume_navigation'
    assert progress.update(0.25, 3.9, 3.0, 0.03, 0.75, 2) == 'start_recovery'
    assert progress.update(0.25, 4.7, 3.0, 0.03, 0.75, 2) == 'resume_navigation'
    assert progress.update(0.25, 4.8, 3.0, 0.03, 0.75, 2) == 'failed'


def test_genuine_progress_resets_timeout_and_resumes_navigation():
    progress = WaypointProgress((1, 2), 1.0, 1.0, 0.0)
    assert progress.update(0.96, 2.9, 3.0, 0.03, 0.75, 3) == 'improved'
    assert progress.last_improvement_at == 2.9
    assert progress.update(0.95, 5.0, 3.0, 0.03, 0.75, 3) == 'normal'
