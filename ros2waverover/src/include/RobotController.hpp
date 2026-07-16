#pragma once
#include <QString>
#include <QObject>
#include <memory>
#include <thread>
#include <RoverCommands.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <chrono>
#include <string>

class UARTSerialPort;
class ROS2Subscriber;
class QTimer;

class RobotController: public QObject {
    Q_OBJECT
    public:
        RobotController();
        ~RobotController();

        bool SendCmdVel(geometry_msgs::msg::Twist::SharedPtr cmd_vel);

public slots:
        void SerialLineReceived(QString line);

    signals:
        void SendRequestSync(QString);

    private:
        enum class ControlMode {
            Twist,
            FixedWing,
            ManualLR
        };

        std::shared_ptr<UARTSerialPort> _pUARTSerialPort;
        std::shared_ptr<ROS2Subscriber> _pROS2Subscriber;
        std::unique_ptr<std::thread> _execThread;
        rclcpp::Executor::SharedPtr _executor;
        QTimer* _serialReadTimer;
        void LoadControlParameters();
        void RunRos2Exectutor();
        void SetupControlSubscriptions();
        void ConfigureImuStream();
        void PublishImuFrame(const QString& line);
        bool SendVelocityCommand(double linear_x, double angular_z);
        bool SendLeftRightVelocityCommand(double left, double right);
        bool SendFixedWingCmdVel(const geometry_msgs::msg::Twist& msg);
        bool SendManualLeftRightCommand(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        void CheckManualCommandTimeout();
        bool SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command);
        bool SendCommandWithValue(WAVE_ROVER_COMMAND_TYPE command, int value);
        bool SendEmergencyStop();
        ControlMode _controlMode = ControlMode::FixedWing;
        std::string _controlModeName = "fixed_wing";
        std::string _cmdVelTopic = "cmd_vel";
        std::string _manualLrTopic = "manual_lr";
        double _manualLrTimeoutSec = 0.5;
        bool _manualCommandActive = false;
        std::chrono::steady_clock::time_point _lastManualCommandTime;
        rclcpp::TimerBase::SharedPtr _manualTimeoutTimer;
        bool _enableImuStream = true;
        int _imuRateHz = 50;
};
