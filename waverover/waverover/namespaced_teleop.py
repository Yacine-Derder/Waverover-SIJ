import argparse
import sys

import teleop_twist_keyboard

from waverover.stack_config import (
    load_stack_config,
    required,
    robot_namespace,
    StackConfigError,
)


def main():
    stack_config = load_stack_config()
    parser = argparse.ArgumentParser(
        description='Run teleop_twist_keyboard in a WaveRover namespace.'
    )
    parser.add_argument(
        '--robot-name',
        default=str(required(stack_config, 'robot_name')),
    )
    parser.add_argument('--speed', type=float)
    parser.add_argument('--turn', type=float)
    args = parser.parse_args()

    robot_name = args.robot_name.strip()
    try:
        robot_ns = robot_namespace(stack_config, robot_name)
    except StackConfigError as error:
        parser.error(str(error))

    ros_args = [
        sys.argv[0],
        '--ros-args',
        '-r',
        '__ns:=/%s' % robot_ns,
    ]
    if args.speed is not None:
        ros_args.extend(['-p', 'speed:=%s' % args.speed])
    if args.turn is not None:
        ros_args.extend(['-p', 'turn:=%s' % args.turn])

    sys.argv = ros_args
    teleop_twist_keyboard.main()


if __name__ == '__main__':
    main()
