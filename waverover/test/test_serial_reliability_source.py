from pathlib import Path


def bridge_source(name):
    return (
        Path(__file__).parents[2] / 'ros2waverover' / name
    ).read_text(encoding='utf-8')


def test_uart_is_queued_coalesced_bounded_and_fail_fast():
    controller = bridge_source('src/source/RobotController.cpp')
    header = bridge_source('src/include/UARTSerialPort.hpp')
    implementation = bridge_source('src/source/UARTSerialPort.cpp')
    assert 'Qt::DirectConnection' not in controller
    assert controller.count('Qt::QueuedConnection') >= 3
    assert 'enqueueVelocity' in controller
    assert 'enqueueOrdered' in controller
    assert 'const int _writeTimeout = 100;' in header
    assert '_latestVelocity' in header
    assert '_maximumReopenAttempts = 3' in header
    assert 'Q_ASSERT(QThread::currentThread() == thread())' in implementation
    assert 'QCoreApplication::exit(2)' in implementation
    assert 'emit Reopened()' in implementation
