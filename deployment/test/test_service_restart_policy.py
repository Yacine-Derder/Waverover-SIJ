from pathlib import Path


def test_service_restart_is_rate_limited():
    drop_in = Path(__file__).parents[1] / (
        'waverover.service.d/10-auto-update.conf'
    )
    text = drop_in.read_text(encoding='utf-8')
    assert 'Restart=always' in text
    assert 'RestartSec=5' in text
    assert 'StartLimitIntervalSec=120' in text
    assert 'StartLimitBurst=5' in text
