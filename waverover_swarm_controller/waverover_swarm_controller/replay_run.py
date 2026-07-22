"""Interactive and headless 2D replay of a recorded experiment."""

import argparse
import bisect
import math
from pathlib import Path

from .analysis_metrics import interpolate_angle
from .offline_data import load_run_data


def _pose_series(data, robot_id):
    topic = '/macortex_bridge/waverover_%s/pose' % robot_id
    output = []
    for sample in data['samples'].get(topic, []):
        pose = sample['message'].pose
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        output.append((
            sample['timestamp_sec'] - data['start_time'],
            float(pose.position.x),
            float(pose.position.y),
            yaw,
        ))
    return output


def interpolate_pose(series, selected_time, maximum_gap):
    if not series:
        return None
    times = [value[0] for value in series]
    index = bisect.bisect_left(times, selected_time)
    if index <= 0:
        return series[0][1:]
    if index >= len(series):
        return series[-1][1:]
    first = series[index - 1]
    second = series[index]
    gap = second[0] - first[0]
    if gap <= 0.0 or gap > maximum_gap:
        return first[1:] if selected_time - first[0] <= second[0] - selected_time else second[1:]
    fraction = (selected_time - first[0]) / gap
    return (
        first[1] + fraction * (second[1] - first[1]),
        first[2] + fraction * (second[2] - first[2]),
        interpolate_angle(first[3], second[3], fraction),
    )


class ReplayApp:
    def __init__(self, data, no_show=False):
        if no_show:
            import matplotlib
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button, CheckButtons, RadioButtons, Slider

        self.plt = plt
        self.data = data
        self.config = data['config']
        self.duration = max(0.0, data['end_time'] - data['start_time'])
        self.time = 0.0
        self.playing = False
        self.speed = 1.0
        self.flags = {
            'trails': True,
            'predicted paths': True,
            'selected edges': True,
            'communication graph': True,
            'communication circles': False,
        }
        self.pose_series = {
            robot_id: _pose_series(data, robot_id)
            for robot_id in self.config.robot_ids
        }
        self.telemetry = sorted(
            data['telemetry'], key=lambda value: value['_timestamp_sec']
        )
        self.figure, self.axis = plt.subplots(figsize=(11, 8))
        self.figure.subplots_adjust(bottom=0.24, right=0.78)
        slider_axis = self.figure.add_axes([0.12, 0.12, 0.58, 0.03])
        self.slider = Slider(
            slider_axis, 'Time [s]', 0.0, max(self.duration, 1e-6), valinit=0.0
        )
        self.slider.on_changed(self._seek)
        play_axis = self.figure.add_axes([0.12, 0.04, 0.10, 0.05])
        self.play_button = Button(play_axis, 'Play/Pause')
        self.play_button.on_clicked(self._toggle_play)
        back_axis = self.figure.add_axes([0.24, 0.04, 0.08, 0.05])
        forward_axis = self.figure.add_axes([0.33, 0.04, 0.08, 0.05])
        self.back_button = Button(back_axis, 'Step -')
        self.forward_button = Button(forward_axis, 'Step +')
        self.back_button.on_clicked(lambda _event: self.step(-1.0))
        self.forward_button.on_clicked(lambda _event: self.step(1.0))
        toggles_axis = self.figure.add_axes([0.80, 0.48, 0.19, 0.30])
        self.checks = CheckButtons(
            toggles_axis,
            list(self.flags),
            list(self.flags.values()),
        )
        self.checks.on_clicked(self._toggle_flag)
        speed_axis = self.figure.add_axes([0.80, 0.20, 0.15, 0.22])
        self.speeds = RadioButtons(
            speed_axis, ('0.25x', '0.5x', '1x', '2x', '4x'), active=2
        )
        self.speeds.on_clicked(
            lambda label: setattr(self, 'speed', float(label[:-1]))
        )
        self.timer = self.figure.canvas.new_timer(interval=100)
        self.timer.add_callback(self._tick)
        self.timer.start()
        self.draw()

    def _seek(self, value):
        self.time = float(value)
        self.draw()

    def _toggle_play(self, _event):
        self.playing = not self.playing

    def _toggle_flag(self, label):
        self.flags[label] = not self.flags[label]
        self.draw()

    def _tick(self):
        if self.playing:
            self.time = min(self.duration, self.time + 0.1 * self.speed)
            self.slider.set_val(self.time)
            if self.time >= self.duration:
                self.playing = False
        return True

    def step(self, amount):
        self.slider.set_val(max(0.0, min(self.duration, self.time + amount)))

    def _telemetry_at(self):
        if not self.telemetry:
            return {}
        target = self.data['start_time'] + self.time
        return min(
            self.telemetry,
            key=lambda value: abs(value['_timestamp_sec'] - target),
        )

    @staticmethod
    def _triangle(x, y, yaw, size=0.10):
        local = ((size, 0.0), (-0.65 * size, 0.55 * size),
                 (-0.65 * size, -0.55 * size))
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return [
            (x + cosine * dx - sine * dy, y + sine * dx + cosine * dy)
            for dx, dy in local
        ]

    def draw(self):
        from matplotlib.patches import Circle, Polygon, Rectangle

        axis = self.axis
        axis.clear()
        config = self.config
        fence = config.safety.geofence
        axis.add_patch(Rectangle(
            (fence.x_min, fence.y_min),
            fence.x_max - fence.x_min,
            fence.y_max - fence.y_min,
            fill=False,
            linestyle='--',
            color='black',
        ))
        axis.scatter([config.station.x], [config.station.y], marker='s',
                     s=80, color='black')
        axis.text(config.station.x, config.station.y, config.station.station_id)
        telemetry = self._telemetry_at()
        recorded_targets = telemetry.get('targets', {})
        priority_id = telemetry.get('priority_target_id')
        for target in config.targets:
            runtime = recorded_targets.get(target.target_id, {})
            weight = runtime.get('weight', target.weight)
            is_priority = target.target_id == priority_id
            axis.scatter(
                [target.x], [target.y], s=120,
                marker='*' if is_priority else 'X',
                color='red' if is_priority else 'orange',
            )
            axis.text(
                target.x, target.y,
                '%s (w=%.2g)' % (target.target_id, weight),
            )
        poses = {}
        for index, robot_id in enumerate(config.robot_ids):
            pose = interpolate_pose(
                self.pose_series[robot_id],
                self.time,
                config.analysis.maximum_interpolation_gap_sec,
            )
            if pose is None:
                continue
            x, y, yaw = pose
            poses[robot_id] = (x, y)
            color = 'C%d' % (index % 10)
            axis.add_patch(Polygon(
                self._triangle(x, y, yaw), closed=True, color=color
            ))
            axis.text(x, y, robot_id)
            if self.flags['trails']:
                trail = [
                    row for row in self.pose_series[robot_id]
                    if self.time - 5.0 <= row[0] <= self.time
                ]
                if trail:
                    axis.plot([row[1] for row in trail], [row[2] for row in trail],
                              color=color, alpha=0.55)
            if self.flags['communication circles']:
                axis.add_patch(Circle(
                    (x, y), config.communication.ideal_range_m,
                    fill=False, color=color, alpha=0.15
                ))
                axis.add_patch(Circle(
                    (x, y), config.safety.minimum_separation_m,
                    fill=False, color='red', alpha=0.12
                ))
        nodes = {config.station.station_id: config.station.position, **poses}
        if self.flags['communication graph']:
            node_ids = sorted(nodes)
            for first_index, first in enumerate(node_ids):
                for second in node_ids[first_index + 1:]:
                    distance = math.dist(nodes[first], nodes[second])
                    if distance <= config.communication.maximum_range_m:
                        quality = max(0.0, min(
                            1.0,
                            (config.communication.maximum_range_m - distance)
                            / config.communication.maximum_range_m,
                        ))
                        axis.plot(
                            [nodes[first][0], nodes[second][0]],
                            [nodes[first][1], nodes[second][1]],
                            color=(1.0 - quality, quality, 0.0),
                            alpha=0.35,
                        )
        if self.flags['selected edges']:
            for first, second in telemetry.get('selected_edges', []):
                if first in nodes and second in nodes:
                    axis.plot(
                        [nodes[first][0], nodes[second][0]],
                        [nodes[first][1], nodes[second][1]],
                        color='blue', linewidth=2.0,
                    )
        if self.flags['predicted paths']:
            for path in telemetry.get('predicted_paths', {}).values():
                if path:
                    axis.plot([point[0] for point in path],
                              [point[1] for point in path], ':', color='purple')
        for key, marker, color in (
            ('setpoints', 'o', 'green'),
            ('active_waypoints', 'D', 'blue'),
            ('pending_waypoints', 'x', 'cyan'),
        ):
            for robot_id, point in telemetry.get(key, {}).items():
                if point is not None:
                    axis.scatter([point[0]], [point[1]], marker=marker, color=color)
        for values in telemetry.get('waypoint_dispatch', {}).values():
            point = values.get('last_acknowledged_waypoint')
            if point is not None:
                axis.scatter(
                    [point[0]], [point[1]], marker='P', color='magenta'
                )
        connectivity = telemetry.get('connectivity', {})
        axis.set_title(
            '%s | mode=%s | solver=%s | state=%s | stop=%s\n'
            't=%.2f s remaining=%.2f s lambda_2=%s'
            % (
                telemetry.get('algorithm', config.controller.algorithm),
                telemetry.get('optimization_mode', ''),
                telemetry.get('solver_status'),
                telemetry.get('result_state'),
                telemetry.get('stop_reason', ''),
                self.time,
                max(0.0, self.duration - self.time),
                connectivity.get('binary_lambda_2'),
            )
        )
        axis.set_xlim(fence.x_min, fence.x_max)
        axis.set_ylim(fence.y_min, fence.y_max)
        axis.set_aspect('equal')
        axis.grid(True, alpha=0.25)
        self.figure.canvas.draw_idle()


def replay(run_directory, selected_time=0.0, output=None, no_show=False):
    data = load_run_data(run_directory)
    app = ReplayApp(data, no_show=no_show)
    app.slider.set_val(max(0.0, min(app.duration, selected_time)))
    if output:
        output_path = Path(output).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        app.figure.savefig(output_path, dpi=150)
    if not no_show:
        app.plt.show()
    else:
        app.plt.close(app.figure)
    return output


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('run_directory')
    parser.add_argument('--time', type=float, default=0.0)
    parser.add_argument('--output')
    parser.add_argument('--no-show', action='store_true')
    arguments = parser.parse_args(args)
    replay(
        arguments.run_directory,
        selected_time=arguments.time,
        output=arguments.output,
        no_show=arguments.no_show,
    )


if __name__ == '__main__':
    main()
