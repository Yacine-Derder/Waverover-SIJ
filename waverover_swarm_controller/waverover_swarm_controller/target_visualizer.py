"""Plot validated swarm target YAML files with optional experiment context."""

import argparse
from pathlib import Path

from .config import GeofenceConfig, load_experiment, load_targets


def load_visualization_data(targets_file, experiment_file=None):
    """Load targets strictly, optionally against an experiment geofence."""
    if experiment_file:
        experiment = load_experiment(experiment_file)
        targets, main_target_id = load_targets(
            Path(targets_file).expanduser().resolve(),
            experiment.safety.geofence,
        )
        return targets, main_target_id, experiment
    unbounded = GeofenceConfig(
        x_min=float('-inf'),
        x_max=float('inf'),
        y_min=float('-inf'),
        y_max=float('inf'),
    )
    targets, main_target_id = load_targets(
        Path(targets_file).expanduser().resolve(), unbounded
    )
    return targets, main_target_id, None


def build_figure(targets, main_target_id, experiment=None, title=None):
    """Create a Matplotlib target figure without displaying or saving it."""
    from matplotlib import pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    figure, axes = plt.subplots()
    main_target = next(
        target for target in targets if target.target_id == main_target_id
    )
    secondary = [
        target for target in targets if target.target_id != main_target_id
    ]
    axes.scatter(
        [main_target.x], [main_target.y],
        marker='*', s=220, color='red', label='Main target', zorder=5,
    )
    if secondary:
        axes.scatter(
            [target.x for target in secondary],
            [target.y for target in secondary],
            marker='o', s=70, color='tab:blue', label='Secondary targets',
            zorder=4,
        )
    for target in targets:
        axes.annotate(
            '%s\nweight=%g' % (target.target_id, target.weight),
            target.position,
            xytext=(6, 6),
            textcoords='offset points',
        )

    if experiment is not None:
        station = experiment.station
        geofence = experiment.safety.geofence
        axes.scatter(
            [station.x], [station.y], marker='s', s=90,
            color='black', label='Station', zorder=5,
        )
        axes.annotate(
            station.station_id,
            station.position,
            xytext=(6, -14),
            textcoords='offset points',
        )
        axes.add_patch(Rectangle(
            (geofence.x_min, geofence.y_min),
            geofence.x_max - geofence.x_min,
            geofence.y_max - geofence.y_min,
            fill=False,
            edgecolor='tab:green',
            linewidth=1.5,
            label='Geofence',
        ))
        axes.add_patch(Circle(
            station.position,
            experiment.communication.ideal_range_m,
            fill=False,
            edgecolor='tab:orange',
            linestyle='--',
            label='Ideal communication range',
        ))
        axes.add_patch(Circle(
            station.position,
            experiment.communication.maximum_range_m,
            fill=False,
            edgecolor='tab:red',
            linestyle=':',
            label='Maximum communication range',
        ))
        x_margin = max(0.25, 0.05 * (geofence.x_max - geofence.x_min))
        y_margin = max(0.25, 0.05 * (geofence.y_max - geofence.y_min))
        axes.set_xlim(geofence.x_min - x_margin, geofence.x_max + x_margin)
        axes.set_ylim(geofence.y_min - y_margin, geofence.y_max + y_margin)
    else:
        x_values = [target.x for target in targets]
        y_values = [target.y for target in targets]
        span = max(
            max(x_values) - min(x_values),
            max(y_values) - min(y_values),
            1.0,
        )
        margin = max(0.25, 0.15 * span)
        axes.set_xlim(min(x_values) - margin, max(x_values) + margin)
        axes.set_ylim(min(y_values) - margin, max(y_values) + margin)

    axes.set_xlabel('x (m)')
    axes.set_ylabel('y (m)')
    axes.set_aspect('equal', adjustable='box')
    axes.grid(True, alpha=0.35)
    axes.set_title(title or 'WaveRover targets')
    axes.legend(loc='best')
    figure.tight_layout()
    return figure


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description='Visualize a validated WaveRover target YAML.'
    )
    parser.add_argument('targets_file', metavar='TARGETS_FILE')
    parser.add_argument('--experiment-file')
    parser.add_argument('--output')
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--title')
    return parser.parse_args(args)


def main(args=None):
    options = parse_args(args)
    if options.no_show:
        import matplotlib
        matplotlib.use('Agg')
    targets, main_target_id, experiment = load_visualization_data(
        options.targets_file, options.experiment_file
    )
    figure = build_figure(
        targets, main_target_id, experiment, title=options.title
    )
    if options.output:
        output = Path(options.output).expanduser()
        if output.suffix.lower() not in ('.png', '.pdf', '.svg'):
            raise ValueError('--output must end in .png, .pdf, or .svg.')
        output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output)
    if not options.no_show:
        from matplotlib import pyplot as plt
        plt.show()
    else:
        from matplotlib import pyplot as plt
        plt.close(figure)


if __name__ == '__main__':
    main()
