#pragma once
#include <QString>
#include <QSerialPort>
#include <QObject>
#include <QDateTime>


class UARTSerialPort : public QObject {
    Q_OBJECT

    public:
        UARTSerialPort(QString path, int baudrate);
        ~UARTSerialPort();

        bool isAvailable();

        quint64 successfulWrites() const { return _successfulWrites; }
        quint64 failedWrites() const { return _failedWrites; }
        quint64 timeouts() const { return _timeouts; }
        quint64 reopenAttempts() const { return _reopenAttempts; }
        int consecutiveFailures() const { return _consecutiveFailures; }
        qint64 lastSuccessfulWriteMs() const { return _lastSuccessfulWriteMs; }

    public slots:
        void enqueueVelocity(QString);
        void enqueueOrdered(QString);
        void flushLatestVelocity();
        bool readResponse();

    signals:
        void LineReceived(QString line);
        void Reopened();

    private:
        QSerialPort _serial;
        QByteArray _lineBuffer;

        bool writeNow(const QString& text);
        bool reopen();
        QString _path;
        int _baudrate;
        QString _latestVelocity;
        bool _velocityFlushScheduled = false;
        quint64 _successfulWrites = 0;
        quint64 _failedWrites = 0;
        quint64 _timeouts = 0;
        quint64 _reopenAttempts = 0;
        int _consecutiveFailures = 0;
        qint64 _lastSuccessfulWriteMs = 0;
        const int _writeTimeout = 100;
        const int _maximumReopenAttempts = 3;

};
