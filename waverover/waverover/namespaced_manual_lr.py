import argparse
import os

from waverover.stack_config import (
    load_stack_config,
    required,
    robot_namespace,
)


def manual_lr_command(stack_config, robot_name):
    """Build the namespaced manual L/R UI command."""
    robot_ns = robot_namespace(stack_config, robot_name)
    defaults = required(stack_config, 'manual_lr_ui')
    topic = required(stack_config, 'topics', 'manual_lr')
    command = [
        'ros2',
        'run',
        'ros2waverover',
        'manual_lr_ui',
        '--ros-args',
        '-r',
        '__ns:=/%s' % robot_ns,
        '-p',
        'topic:=%s' % topic,
    ]
    for parameter_name in (
        'publish_rate_hz',
        'step',
        'large_step',
        'min_speed',
        'max_speed',
    ):
        command.extend([
            '-p',
            '%s:=%s' % (parameter_name, defaults[parameter_name]),
        ])
    return command


def main():
    stack_config = load_stack_config()
    parser = argparse.ArgumentParser(
        description='Run the manual L/R UI in a WaveRover namespace.'
    )
    parser.add_argument(
        '--robot-name',
        default=str(required(stack_config, 'robot_name')),
    )
    args = parser.parse_args()
    command = manual_lr_command(stack_config, args.robot_name)
    os.execvp(command[0], command)


if __name__ == '__main__':
    main()
