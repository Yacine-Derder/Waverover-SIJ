#include <UARTSerialPort.hpp>
#include <iostream>
#include <QSerialPortInfo>
#include <QDebug>

UARTSerialPort::UARTSerialPort(QString path, int baudrate) {
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

void UARTSerialPort::sendRequestSync(QString text)
{
    std::lock_guard<std::mutex> lock(_serial_mutex);
    if(!isAvailable())
    {
        qDebug() << "Serial Port is down!";
        return;
    }
    QByteArray payload = (text + "\n").toUtf8();
    qint64 bytesWritten = _serial.write(payload);
    if (bytesWritten < 0)
    {
        qDebug() << "UART write failed:" << _serial.errorString();
        return;
    }

    if (!_serial.waitForBytesWritten(_writeTimeout))
    {
        qDebug() << "UART write timeout:" << _serial.errorString();
        return;
    }
}

bool UARTSerialPort::readResponse()
{
    QByteArray data;
    {
        std::lock_guard<std::mutex> lock(_serial_mutex);
        data = _serial.readAll();
    }

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

    else
    {
        qDebug() << "Trying to recover the Serial Port...";
        _serial.close();
        return _serial.open(QIODevice::ReadWrite);
    }
    return false;
}
