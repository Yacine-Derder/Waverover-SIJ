import math

from geometry_msgs.msg import PoseStamped
import pytest

from waverover_controller.waypoint_controller import (
    McsPoseProvider,
    PoseUnavailableError,
)


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, message):
        self.infos.append(message)

    def warn(self, message):
        self.warnings.append(message)


class FakeNode:
    def __init__(self):
        self.logger = FakeLogger()
        self.subscription = None

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscription = (message_type, topic, callback, qos)
        return self.subscription

    def get_logger(self):
        return self.logger


def make_pose(frame='robotics_lab', x=1.0, y=2.0, yaw=0.0):
    message = PoseStamped()
    message.header.frame_id = frame
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def make_provider(timeout=0.5):
    node = FakeNode()
    provider = McsPoseProvider(
        node,
        '/macortex_bridge/waverover_29/pose',
        'robotics_lab',
        timeout,
        10,
    )
    return node, provider


def test_mcs_provider_requires_a_valid_pose_and_extracts_yaw():
    node, provider = make_provider()

    with pytest.raises(PoseUnavailableError, match='no valid MCS pose'):
        provider.lookup_pose()

    provider._pose_callback(make_pose(x=3.0, y=-1.0, yaw=math.pi / 2.0))
    pose = provider.lookup_pose()

    assert pose.x == pytest.approx(3.0)
    assert pose.y == pytest.approx(-1.0)
    assert pose.yaw == pytest.approx(math.pi / 2.0)
    assert len(node.logger.infos) == 1


def test_mcs_provider_rejects_wrong_frame_and_non_finite_values():
    node, provider = make_provider()

    provider._pose_callback(make_pose(frame='waverover_29/map'))
    invalid = make_pose()
    invalid.pose.position.x = float('nan')
    provider._pose_callback(invalid)

    with pytest.raises(PoseUnavailableError):
        provider.lookup_pose()
    assert len(node.logger.warnings) == 2


def test_mcs_provider_rejects_zero_quaternion():
    node, provider = make_provider()
    invalid = make_pose()
    invalid.pose.orientation.z = 0.0
    invalid.pose.orientation.w = 0.0

    provider._pose_callback(invalid)

    with pytest.raises(PoseUnavailableError):
        provider.lookup_pose()
    assert 'zero-length quaternion' in node.logger.warnings[0]


def test_mcs_provider_stops_using_a_stale_pose_and_recovers():
    _, provider = make_provider(timeout=0.05)
    provider._pose_callback(make_pose())
    provider.latest_pose_received_at -= 1.0

    with pytest.raises(PoseUnavailableError, match='stale'):
        provider.lookup_pose()

    provider._pose_callback(make_pose(x=4.0))
    assert provider.lookup_pose().x == pytest.approx(4.0)
