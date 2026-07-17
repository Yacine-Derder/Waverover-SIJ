from io import StringIO
from types import SimpleNamespace

from builtin_interfaces.msg import Time

from waverover_waypoint_ui import waypoint_ui
from waverover_waypoint_ui.waypoint_ui import (
    WaypointPublisher,
    WaypointTerminal,
    end_trial_topic,
    waypoint_topic,
)


class FakeTimer:
    def __init__(self, events=None):
        self.canceled = False
        self.events = events

    def is_canceled(self):
        return self.canceled

    def cancel(self):
        self.canceled = True
        if self.events is not None:
            self.events.append('refresh_stopped')

    def reset(self):
        self.canceled = False


class FakePublisher:
    def __init__(self, topic, events=None):
        self.topic = topic
        self.messages = []
        self.events = events

    def publish(self, message):
        self.messages.append(message)
        if self.events is not None:
            self.events.append('end_published')


def make_session():
    published = []
    session = SimpleNamespace(
        commanded_robot_ids=set(),
        latest_waypoints={},
        _refresh_timer=FakeTimer(),
        _cleanup_complete=False,
        _end_trial_publishers={},
        _end_trial_qos=object(),
    )

    def publish_target(robot_id, x, y):
        published.append((robot_id, x, y))
        return SimpleNamespace(header=SimpleNamespace(frame_id='robotics_lab'))

    session._publish_target = publish_target
    session.create_publisher = lambda _type, topic, _qos: FakePublisher(topic)
    session.stop_refreshes = lambda: WaypointPublisher.stop_refreshes(session)
    session.publish_end_trial = lambda: WaypointPublisher.publish_end_trial(
        session
    )
    session.end_trial = lambda: WaypointPublisher.end_trial(session)
    return session, published


def publish(session, robot_id, x, y):
    return WaypointPublisher.publish_waypoint(session, robot_id, x, y)


def test_latest_command_replaces_one_rover_target_and_two_are_retained():
    session, published = make_session()

    publish(session, '131', 1.0, 1.0)
    publish(session, '132', 1.0, 0.0)
    publish(session, '131', 2.0, 3.0)

    assert session.latest_waypoints == {
        '131': (2.0, 3.0),
        '132': (1.0, 0.0),
    }
    assert session.commanded_robot_ids == {'131', '132'}
    WaypointPublisher.refresh_waypoints(session)
    assert published[-2:] == [
        ('131', 2.0, 3.0),
        ('132', 1.0, 0.0),
    ]


def test_refresh_builds_fresh_stamped_per_rover_messages_and_topics():
    class Clock:
        def __init__(self):
            self.seconds = 0

        def now(self):
            self.seconds += 1
            return SimpleNamespace(to_msg=lambda: Time(sec=self.seconds))

    publishers = {}
    clock = Clock()
    target = SimpleNamespace(
        pose_source='SLAM',
        get_clock=lambda: clock,
    )

    def ensure(robot_id):
        topic = waypoint_topic(robot_id)
        publishers.setdefault(robot_id, FakePublisher(topic))
        return publishers[robot_id]

    target._ensure_publishers = ensure
    first = WaypointPublisher._publish_target(target, '131', 1.0, 2.0)
    second = WaypointPublisher._publish_target(target, '132', 3.0, 4.0)

    assert publishers['131'].topic == '/waverover_131/waypoints'
    assert publishers['132'].topic == '/waverover_132/waypoints'
    assert first.header.stamp.sec < second.header.stamp.sec
    assert first.header.frame_id == 'waverover_131/map'
    assert second.header.frame_id == 'waverover_132/map'


def test_first_waypoint_precreates_waypoint_and_end_trial_publishers():
    created_topics = []
    target = SimpleNamespace(
        _waypoint_publishers={},
        _end_trial_publishers={},
        _waypoint_qos=object(),
        _end_trial_qos=object(),
    )

    def create_publisher(_message_type, topic, _qos):
        created_topics.append(topic)
        return FakePublisher(topic)

    target.create_publisher = create_publisher
    WaypointPublisher._ensure_publishers(target, '131')

    assert created_topics == [
        '/waverover_131/waypoints',
        '/waverover_131/end_trial',
    ]


def test_selection_without_waypoint_is_not_commanded():
    session, _ = make_session()
    node = SimpleNamespace(
        default_robot_id='131',
        terminal_device='',
        commanded_robot_ids=session.commanded_robot_ids,
        latest_waypoints=session.latest_waypoints,
        pose_source='MCS',
        refresh_rate_hz=1.0,
    )
    terminal = WaypointTerminal(
        node,
        input_stream=SimpleNamespace(),
        output_stream=SimpleNamespace(
            write=lambda _text: None,
            flush=lambda: None,
        ),
    )

    assert terminal._handle_command('robot 132')
    assert session.commanded_robot_ids == set()


def test_end_stops_refresh_before_signaling_and_keeps_session_history():
    events = []
    session, _ = make_session()
    session._refresh_timer = FakeTimer(events)
    session.commanded_robot_ids.update(('132', '131'))
    session.latest_waypoints['131'] = (1.0, 1.0)
    session._end_trial_publishers = {
        robot_id: FakePublisher(end_trial_topic(robot_id), events)
        for robot_id in session.commanded_robot_ids
    }

    signaled = WaypointPublisher.end_trial(session)

    assert events == ['refresh_stopped', 'end_published', 'end_published']
    assert signaled == ['131', '132']
    assert session.latest_waypoints == {}
    assert session.commanded_robot_ids == {'131', '132'}


def test_cleanup_targets_only_commanded_rovers_and_is_idempotent():
    session, _ = make_session()
    publish(session, '131', 1.0, 1.0)

    first = WaypointPublisher.cleanup(session)
    second = WaypointPublisher.cleanup(session)

    assert first == ['131']
    assert second == []
    publisher = session._end_trial_publishers['131']
    assert len(publisher.messages) == 1
    assert publisher.topic == '/waverover_131/end_trial'


def test_quit_eof_and_interrupt_termination_can_share_idempotent_cleanup(
    monkeypatch,
):
    monkeypatch.setattr(waypoint_ui.rclpy, 'ok', lambda: True)
    for input_stream in (
        StringIO('quit\n'),
        StringIO(''),
        SimpleNamespace(
            fileno=lambda: (_ for _ in ()).throw(OSError()),
            readline=lambda: (_ for _ in ()).throw(KeyboardInterrupt()),
        ),
    ):
        session, _ = make_session()
        publish(session, '131', 1.0, 1.0)
        node = SimpleNamespace(
            default_robot_id='131',
            terminal_device='',
            commanded_robot_ids=session.commanded_robot_ids,
            latest_waypoints=session.latest_waypoints,
            pose_source='MCS',
            refresh_rate_hz=1.0,
        )
        terminal = WaypointTerminal(
            node,
            input_stream=input_stream,
            output_stream=StringIO(),
        )
        terminal.run()

        assert WaypointPublisher.cleanup(session) == ['131']
        assert WaypointPublisher.cleanup(session) == []
