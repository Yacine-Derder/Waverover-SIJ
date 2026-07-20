from pathlib import Path

import matplotlib

matplotlib.use('Agg')

from matplotlib import pyplot as plt

from waverover_swarm_controller.target_visualizer import (
    build_figure,
    load_visualization_data,
    main,
)


def config_paths():
    config = Path(__file__).parents[1] / 'config'
    return config / 'targets_smoke_6.yaml', config / 'smoke_test_6.yaml'


def test_figure_has_targets_labels_and_experiment_context():
    targets_path, experiment_path = config_paths()
    targets, main_id, experiment = load_visualization_data(
        targets_path, experiment_path
    )

    figure = build_figure(targets, main_id, experiment, title='Smoke targets')
    axes = figure.axes[0]

    assert axes.get_xlabel() == 'x (m)'
    assert axes.get_ylabel() == 'y (m)'
    assert len(axes.collections) >= 2  # neutral targets and station
    assert len(axes.patches) == 3  # geofence and two range circles
    annotation_text = {text.get_text() for text in axes.texts}
    assert any('target_0' in text for text in annotation_text)
    assert any('target_1' in text for text in annotation_text)
    assert experiment.station.station_id in annotation_text
    plt.close(figure)


def test_headless_cli_saves_nonempty_png(tmp_path):
    targets_path, experiment_path = config_paths()
    output = tmp_path / 'nested' / 'targets.png'

    main([
        str(targets_path),
        '--experiment-file', str(experiment_path),
        '--output', str(output),
        '--no-show',
    ])

    assert output.is_file()
    assert output.stat().st_size > 0
    assert output.read_bytes().startswith(b'\x89PNG\r\n\x1a\n')
