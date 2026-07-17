from types import SimpleNamespace

import pytest

from waverover_swarm_controller.models import StationState, TargetState
from waverover_swarm_controller.pose_aggregation import (
    PoseAggregator,
    SnapshotUnavailableError,
)


def pose(frame='robotics_lab', x=0.0, y=0.0, quaternion=(0, 0, 0, 2)):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=frame,
            stamp=SimpleNamespace(sec=5, nanosec=250000000),
        ),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=0.0),
            orientation=SimpleNamespace(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3],
            ),
        ),
    )


def aggregator():
    return PoseAggregator(('131', '132'), 'robotics_lab', 0.5, 0.1)


def test_valid_normalized_poses_form_fresh_synchronized_snapshot():
    poses = aggregator()
    assert poses.receive('131', pose(x=1.0), receipt_time=10.0)
    assert poses.receive('132', pose(y=1.0), receipt_time=10.05)

    snapshot = poses.snapshot(
        (TargetState('target', 2.0, 0.0, 1.0, True),),
        StationState('station', 0.0, 0.0),
        now=10.1,
    )

    assert tuple(snapshot.robots) == ('131', '132')
    assert snapshot.robots['131'].yaw == pytest.approx(0.0)
    assert snapshot.robots['131'].message_time == pytest.approx(5.25)


@pytest.mark.parametrize(
    'message',
    [
        pose(frame='wrong'),
        pose(x=float('nan')),
        pose(quaternion=(0, 0, 0, 0)),
    ],
)
def test_invalid_pose_is_rejected(message):
    assert not aggregator().receive('131', message, receipt_time=10.0)


def test_missing_stale_and_skewed_snapshots_are_rejected():
    poses = aggregator()
    poses.receive('131', pose(), receipt_time=10.0)
    with pytest.raises(SnapshotUnavailableError, match='Missing'):
        poses.snapshot((), StationState('station', 0.0, 0.0), now=10.1)

    poses.receive('132', pose(), receipt_time=10.2)
    with pytest.raises(SnapshotUnavailableError, match='skew'):
        poses.snapshot((), StationState('station', 0.0, 0.0), now=10.2)
    with pytest.raises(SnapshotUnavailableError, match='Stale'):
        poses.snapshot((), StationState('station', 0.0, 0.0), now=11.0)
