#include <UARTSerialPort.hpp>
#include <iostream>
#include <QSerialPortInfo>
#include <QDebug>
#include <QCoreApplication>
#include <QTimer>
#include <QThread>

UARTSerialPort::UARTSerialPort(QString path, int baudrate)
    : _path(path), _baudrate(baudrate) {
    const auto infos = QSerialPortInfo::availablePorts();
    std::cout << "Detected Serial port:";
    for (const QSerialPortInfo &info : infos)
    {
        std::cout << "\n" << info.portName().toStdString() << std::endl;

        if(path == "") {
            path = info.portName();
            path = "/dev/" + info.portName();
        }
    }
    _path = path;

    std::cout << (infos.size() ? "----" : " None.") << std::endl;
    qDebug() << "Opening " << path << " at baudrate " << baudrate << "...";

    _serial.setPortName(path);
    _serial.setBaudRate(baudrate);
    _serial.setDataBits(QSerialPort::Data8);
    _serial.setParity(QSerialPort::NoParity);
    _serial.setStopBits(QSerialPort::OneStop);
    _serial.setFlowControl(QSerialPort::NoFlowControl);
    _serial.setReadBufferSize(65536);

    if (!_serial.open(QIODevice::OpenModeFlag::ReadWrite))
    {
        qDebug() << (QString("Can't open %1, error code %2 : %3")
                     .arg(_serial.portName())
                     .arg(_serial.error())
                     .arg(_serial.errorString()));
        qDebug() << "Serial status: " << _serial.isOpen();
    }
    else
    {
        _serial.setRequestToSend(true);
        _serial.setDataTerminalReady(true);
    }
}

UARTSerialPort::~UARTSerialPort() {
    _serial.close();
}

void UARTSerialPort::enqueueVelocity(QString text)
{
    Q_ASSERT(QThread::currentThread() == thread());
    _latestVelocity = text;
    if (_velocityFlushScheduled)
    {
        return;
    }
    _velocityFlushScheduled = true;
    QTimer::singleShot(0, this, &UARTSerialPort::flushLatestVelocity);
}

void UARTSerialPort::flushLatestVelocity()
{
    Q_ASSERT(QThread::currentThread() == thread());
    const QString newest = _latestVelocity;
    _latestVelocity.clear();
    _velocityFlushScheduled = false;
    writeNow(newest);
    if (!_latestVelocity.isEmpty() && !_velocityFlushScheduled)
    {
        _velocityFlushScheduled = true;
        QTimer::singleShot(0, this, &UARTSerialPort::flushLatestVelocity);
    }
}

void UARTSerialPort::enqueueOrdered(QString text)
{
    Q_ASSERT(QThread::currentThread() == thread());
    writeNow(text);
}

bool UARTSerialPort::writeNow(const QString& text)
{
    Q_ASSERT(QThread::currentThread() == thread());
    if(!isAvailable())
    {
        ++_failedWrites;
        ++_consecutiveFailures;
        if (_consecutiveFailures >= _maximumReopenAttempts)
        {
            qCritical() << "Serial reopen recovery exhausted; terminating bridge.";
            QCoreApplication::exit(2);
        }
        return false;
    }
    const QByteArray payload = (text + "\n").toUtf8();
    qint64 bytesWritten = _serial.write(payload);
    if (bytesWritten < 0)
    {
        ++_failedWrites;
        ++_consecutiveFailures;
        qDebug() << "UART write failed:" << _serial.errorString();
        return false;
    }

    if (!_serial.waitForBytesWritten(_writeTimeout))
    {
        ++_failedWrites;
        ++_timeouts;
        ++_consecutiveFailures;
        qDebug() << "UART write timeout:" << _serial.errorString();
        _serial.close();
        if (_consecutiveFailures >= _maximumReopenAttempts && !reopen())
        {
            qCritical() << "Serial recovery exhausted; terminating bridge.";
            QCoreApplication::exit(2);
        }
        return false;
    }
    ++_successfulWrites;
    _consecutiveFailures = 0;
    _lastSuccessfulWriteMs = QDateTime::currentMSecsSinceEpoch();
    return true;
}

bool UARTSerialPort::readResponse()
{
    Q_ASSERT(QThread::currentThread() == thread());
    QByteArray data = _serial.readAll();

    if (data.isEmpty())
    {
        return false;
    }

    _lineBuffer.append(data);

    int newlineIndex = _lineBuffer.indexOf('\n');
    while (newlineIndex >= 0)
    {
        QByteArray line = _lineBuffer.left(newlineIndex).trimmed();
        _lineBuffer.remove(0, newlineIndex + 1);

        if (!line.isEmpty())
        {
            emit LineReceived(QString::fromUtf8(line));
        }

        newlineIndex = _lineBuffer.indexOf('\n');
    }

    return true;
}

bool UARTSerialPort::isAvailable()
{
    if(_serial.isOpen())
        return true;
    return reopen();
}

bool UARTSerialPort::reopen()
{
    Q_ASSERT(QThread::currentThread() == thread());
    for (int attempt = 0; attempt < _maximumReopenAttempts; ++attempt)
    {
        ++_reopenAttempts;
        _serial.close();
        _serial.setPortName(_path);
        _serial.setBaudRate(_baudrate);
        if (_serial.open(QIODevice::ReadWrite))
        {
            _serial.setRequestToSend(true);
            _serial.setDataTerminalReady(true);
            _consecutiveFailures = 0;
            emit Reopened();
            return true;
        }
    }
    return false;
}
