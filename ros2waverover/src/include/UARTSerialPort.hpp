#pragma once
#include <QString>
#include <QSerialPort>
#include <QObject>
#include <mutex>


class UARTSerialPort : public QObject {
    Q_OBJECT

    public:
        UARTSerialPort(QString path, int baudrate);
        ~UARTSerialPort();

        bool isAvailable();

    public slots:
        void sendRequestSync(QString);
        bool readResponse();

    signals:
        void LineReceived(QString line);

    private:
        QSerialPort _serial;
        QByteArray _lineBuffer;

        std::mutex _serial_mutex;

        const int _writeTimeout = 10000;

};
