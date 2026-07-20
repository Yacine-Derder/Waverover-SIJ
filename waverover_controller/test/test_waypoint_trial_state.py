from collections import deque
from types import SimpleNamespace

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty

from waverover_controller.waypoint_controller import (
    FIXED_WING_STOP_ANGULAR_X,
    WaypointController,
)


class Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_controller(control_mode='fixed_wing'):
    controller = SimpleNamespace(
        global_frame='robotics_lab',
        waypoint_queue=deque(),
        received_any_waypoint=False,
        last_bank_direction='right',
        loiter_direction='right',
        loitering=False,
        trial_ended=False,
        _pose_failure_active=False,
        _last_throttled_log={},
        control_mode=control_mode,
        cmd_vel_publisher=Publisher(),
        waypoint_reached_publisher=Publisher(),
    )
    controller.get_logger = lambda: Logger()
    controller._info_throttled = lambda *_args, **_kwargs: None
    controller.publish_safe_stop = (
        lambda: WaypointController.publish_safe_stop(controller)
    )
    return controller


def waypoint(x=1.0, y=2.0, frame='robotics_lab'):
    message = PointStamped()
    message.header.frame_id = frame
    message.point.x = x
    message.point.y = y
    return message


def stamped_waypoint(token, x=1.0, y=2.0):
    message = waypoint(x=x, y=y)
    message.header.stamp.sec, message.header.stamp.nanosec = token
    return message


def receive(controller, message):
    WaypointController._waypoint_callback(controller, message)


def test_refreshed_duplicates_coalesce_and_reached_coordinate_is_accepted_again():
    controller = make_controller(control_mode='twist')
    receive(controller, waypoint())
    receive(controller, waypoint(x=1.0 + 0.5e-6))
    receive(controller, waypoint())
    assert list(controller.waypoint_queue) == [(1.0, 2.0)]

    controller.goal_tolerance_m = 0.1
    controller._lookup_pose_or_stop = lambda: SimpleNamespace(
        x=1.0, y=2.0, yaw=0.0
    )
    controller.publish_stop = lambda: None
    WaypointController._control_step(controller)
    assert not controller.waypoint_queue

    receive(controller, waypoint())
    assert list(controller.waypoint_queue) == [(1.0, 2.0)]


def test_reached_ack_echoes_token_and_delayed_refresh_cannot_requeue():
    controller = make_controller(control_mode='twist')
    token = (123, 456)
    receive(controller, stamped_waypoint(token))
    controller.goal_tolerance_m = 0.1
    controller._lookup_pose_or_stop = lambda: SimpleNamespace(
        x=1.0, y=2.0, yaw=0.0
    )
    controller.publish_stop = lambda: None
    WaypointController._control_step(controller)
    acknowledgement = controller.waypoint_reached_publisher.messages[-1]
    assert (
        acknowledgement.header.stamp.sec,
        acknowledgement.header.stamp.nanosec,
    ) == token
    receive(controller, stamped_waypoint(token))
    assert not controller.waypoint_queue
    receive(controller, stamped_waypoint((123, 457)))
    assert list(controller.waypoint_queue) == [(1.0, 2.0)]


def test_end_trial_clears_queue_exits_loiter_and_uses_fixed_wing_marker():
    controller = make_controller()
    controller.waypoint_queue.extend(((1.0, 2.0), (3.0, 4.0)))
    controller.loitering = True

    WaypointController._end_trial_callback(controller, Empty())

    assert not controller.waypoint_queue
    assert controller.trial_ended
    assert not controller.loitering
    assert controller.loiter_direction is None
    assert controller.last_bank_direction is None
    assert controller.cmd_vel_publisher.messages[-1].angular.x == (
        FIXED_WING_STOP_ANGULAR_X
    )


def test_trial_ended_control_cycle_stays_stopped_and_never_loiters():
    controller = make_controller()
    controller.trial_ended = True
    controller._control_empty_queue = lambda: (_ for _ in ()).throw(
        AssertionError('must not enter empty-queue loiter logic')
    )

    WaypointController._control_step(controller)
    WaypointController._control_step(controller)

    assert len(controller.cmd_vel_publisher.messages) == 2
    assert all(
        message.angular.x == FIXED_WING_STOP_ANGULAR_X
        for message in controller.cmd_vel_publisher.messages
    )


def test_valid_waypoint_starts_new_trial_but_invalid_waypoint_does_not():
    controller = make_controller()
    controller.trial_ended = True

    receive(controller, waypoint(frame='wrong'))
    assert controller.trial_ended
    assert not controller.waypoint_queue

    receive(controller, waypoint())
    assert not controller.trial_ended
    assert list(controller.waypoint_queue) == [(1.0, 2.0)]


def test_twist_mode_end_trial_uses_ordinary_zero_twist():
    controller = make_controller(control_mode='twist')

    WaypointController._end_trial_callback(controller, Empty())

    stop = controller.cmd_vel_publisher.messages[-1]
    assert stop.linear.x == 0.0
    assert stop.angular.x == 0.0
    assert stop.angular.z == 0.0
