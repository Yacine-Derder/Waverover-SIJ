#include <RobotController.hpp>
#include <UARTSerialPort.hpp>
#include <ROS2Subscriber.hpp>
#include <json.hpp>
#include <RoverCommands.hpp>
#include <QDebug>
#include <QDateTime>
#include <QTimer>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <stdexcept>

namespace {
constexpr int FEEDBACK_BASE_INFO = 1001;
constexpr double TWIST_INPUT_LIMIT = 1.0;
constexpr double TWIST_WHEEL_COMMAND_LIMIT = 0.5;
constexpr double WHEEL_COMMAND_LIMIT = 0.5;
constexpr double FIXED_WING_STRAIGHT_SPEED = 0.22;
constexpr double FIXED_WING_TURN_OUTER_SPEED = 0.48;
constexpr double FIXED_WING_BANK_INNER_SPEED = -0.1;
constexpr double FIXED_WING_STOP_ANGULAR_X = 1.0;
constexpr double DEFAULT_MANUAL_LR_TIMEOUT_SEC = 0.5;
constexpr double ZERO_TWIST_EPSILON = 1e-9;

double Clamp(double value, double minimum, double maximum)
{
    return std::max(std::min(value, maximum), minimum);
}

bool NearlyZero(double value)
{
    return std::abs(value) <= ZERO_TWIST_EPSILON;
}

bool IsZeroTwist(const geometry_msgs::msg::Twist& msg)
{
    return NearlyZero(msg.linear.x) &&
        NearlyZero(msg.linear.y) &&
        NearlyZero(msg.linear.z) &&
        NearlyZero(msg.angular.x) &&
        NearlyZero(msg.angular.y) &&
        NearlyZero(msg.angular.z);
}

bool IsFixedWingStopRequest(const geometry_msgs::msg::Twist& msg)
{
    return NearlyZero(msg.linear.x) &&
        NearlyZero(msg.linear.y) &&
        NearlyZero(msg.linear.z) &&
        NearlyZero(msg.angular.x - FIXED_WING_STOP_ANGULAR_X) &&
        NearlyZero(msg.angular.y) &&
        NearlyZero(msg.angular.z);
}

bool ExtractNumberField(const std::string& text, const std::string& field, double& value)
{
    const std::string key = "\"" + field + "\"";
    std::size_t keyPosition = text.find(key);
    if (keyPosition == std::string::npos)
    {
        return false;
    }

    std::size_t valuePosition = text.find(':', keyPosition + key.size());
    if (valuePosition == std::string::npos)
    {
        return false;
    }

    const char* valueStart = text.c_str() + valuePosition + 1;
    char* valueEnd = nullptr;
    value = std::strtod(valueStart, &valueEnd);
    return valueEnd != valueStart;
}

bool ExtractIntField(const std::string& text, const std::string& field, int& value)
{
    double numericValue = 0.0;
    if (!ExtractNumberField(text, field, numericValue))
    {
        return false;
    }

    value = static_cast<int>(numericValue);
    return true;
}
}

RobotController::RobotController() {
    qDebug() << "v1.0.6";

    _pROS2Subscriber = std::make_shared<ROS2Subscriber>();
    LoadControlParameters();

    std::string UART_address = _pROS2Subscriber->get_parameter("UART_address").as_string();
    int baud_rate = _pROS2Subscriber->get_parameter("baud_rate").as_int();
    _enableImuStream = _pROS2Subscriber->declare_parameter("enable_imu_stream", true);
    _imuRateHz = _pROS2Subscriber->declare_parameter("imu_rate_hz", 50);

    qDebug() << "UART Address: " << QString::fromStdString(UART_address);
    qDebug() << "Baud rate: " << baud_rate;
    qDebug() << "IMU stream enabled: " << _enableImuStream;
    qDebug() << "IMU stream rate: " << _imuRateHz << "Hz";

    _pUARTSerialPort = std::make_shared<UARTSerialPort>(
        QString::fromStdString(UART_address),
        baud_rate
    );
    QObject::connect(
        this,
        &RobotController::SendVelocityRequest,
        _pUARTSerialPort.get(),
        &UARTSerialPort::enqueueVelocity,
        Qt::QueuedConnection
    );
    QObject::connect(
        this,
        &RobotController::SendOrderedRequest,
        _pUARTSerialPort.get(),
        &UARTSerialPort::enqueueOrdered,
        Qt::QueuedConnection
    );
    QObject::connect(
        _pUARTSerialPort.get(),
        &UARTSerialPort::LineReceived,
        this,
        &RobotController::SerialLineReceived
    );
    QObject::connect(
        _pUARTSerialPort.get(),
        &UARTSerialPort::Reopened,
        this,
        &RobotController::ConfigureImuStream,
        Qt::QueuedConnection
    );

    _serialReadTimer = new QTimer(this);
    _serialReadTimer->setInterval(5);
    QObject::connect(_serialReadTimer, &QTimer::timeout, this, [this]() {
        _pUARTSerialPort->readResponse();
    });
    _serialReadTimer->start();
    _serialHealthTimer = new QTimer(this);
    _serialHealthTimer->setInterval(1000);
    QObject::connect(
        _serialHealthTimer, &QTimer::timeout,
        this, &RobotController::PublishSerialHealth
    );
    _serialHealthTimer->start();

    SetupControlSubscriptions();

    _executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    _execThread = std::make_unique<std::thread>(&RobotController::RunRos2Exectutor, this);

    qDebug() << "Initialization sequence...";

    QTimer::singleShot(0, this, [this]() {
        ConfigureImuStream();
    });
}

RobotController::~RobotController() {
    if (_controlMode == ControlMode::FixedWing)
    {
        RCLCPP_INFO(
            _pROS2Subscriber->get_logger(),
            "Skipping bridge-shutdown emergency stop in fixed_wing mode; "
            "the explicit fixed-wing stop marker controls rover stopping."
        );
    }
    else
    {
        nlohmann::json stop = {};
        stop["T"] = static_cast<int>(WAVE_ROVER_COMMAND_TYPE::EMERGENCY_STOP);
        // Destruction occurs in the QSerialPort owner thread after the event
        // loop stops, so a queued signal would never be flushed.
        _pUARTSerialPort->enqueueOrdered(
            QString::fromStdString(stop.dump())
        );
    }
    rclcpp::shutdown();
    _executor->cancel();
    if (_execThread->joinable()) {
        _execThread->join();
    }
    qDebug() << "ROS2 Node shut down.";
}

void RobotController::LoadControlParameters() {
    const auto logger = _pROS2Subscriber->get_logger();

    _controlModeName = _pROS2Subscriber->declare_parameter<std::string>(
        "control_mode",
        "fixed_wing"
    );
    _cmdVelTopic = _pROS2Subscriber->declare_parameter<std::string>(
        "cmd_vel_topic",
        "cmd_vel"
    );
    _manualLrTopic = _pROS2Subscriber->declare_parameter<std::string>(
        "manual_lr_topic",
        "manual_lr"
    );
    _manualLrTimeoutSec = _pROS2Subscriber->declare_parameter<double>(
        "manual_lr_timeout_sec",
        DEFAULT_MANUAL_LR_TIMEOUT_SEC
    );

    auto fail = [&logger](const std::string& message) {
        RCLCPP_FATAL(logger, "%s", message.c_str());
        throw std::invalid_argument(message);
    };

    if (_controlModeName == "twist")
    {
        _controlMode = ControlMode::Twist;
    }
    else if (_controlModeName == "fixed_wing")
    {
        _controlMode = ControlMode::FixedWing;
    }
    else if (_controlModeName == "manual_lr")
    {
        _controlMode = ControlMode::ManualLR;
    }
    else
    {
        fail("Invalid control_mode '" + _controlModeName + "'. Expected 'twist', 'fixed_wing', or 'manual_lr'.");
    }

    if (_manualLrTopic.empty())
    {
        fail("manual_lr_topic must not be empty.");
    }

    if (_cmdVelTopic.empty())
    {
        fail("cmd_vel_topic must not be empty.");
    }

    if (_manualLrTimeoutSec <= 0.0)
    {
        fail("manual_lr_timeout_sec must be greater than 0.0.");
    }

    RCLCPP_INFO(logger, "Control mode: %s", _controlModeName.c_str());

    if (_controlMode == ControlMode::FixedWing)
    {
        RCLCPP_INFO(
            logger,
            "Using fixed_wing mapping: straight=(0.25,0.25), left=(-0.1,0.5), right=(0.5,-0.1), explicit stop marker=(0.0,0.0); zero Twist commands straight."
        );
    }
    else if (_controlMode == ControlMode::ManualLR)
    {
        RCLCPP_INFO(
            logger,
            "Using direct manual left/right wheel speed topic %s with %.3fs timeout.",
            _manualLrTopic.c_str(),
            _manualLrTimeoutSec
        );
    }
    else
    {
        RCLCPP_INFO(
            logger,
            "Using differential-drive mapping on topic %s.",
            _cmdVelTopic.c_str()
        );
    }
}

void RobotController::RunRos2Exectutor() {
    std::cout << "STARTING EXECUTOR" << std::endl;
    _executor->add_node(_pROS2Subscriber);
    _executor->spin();
    _executor->remove_node(_pROS2Subscriber);
}

void RobotController::SetupControlSubscriptions() {
    if (_controlMode == ControlMode::ManualLR)
    {
        SendLeftRightVelocityCommand(0.0, 0.0);
        _pROS2Subscriber->SubscribeToManualLRTopic(
            QString::fromStdString(_manualLrTopic),
            [&](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                SendManualLeftRightCommand(msg);
            }
        );

        const int timeoutCheckMs = std::max(
            50,
            static_cast<int>((_manualLrTimeoutSec * 500.0) + 0.5)
        );
        _manualTimeoutTimer = _pROS2Subscriber->create_wall_timer(
            std::chrono::milliseconds(timeoutCheckMs),
            [this]() {
                CheckManualCommandTimeout();
            }
        );
        return;
    }

    _pROS2Subscriber->SubscribeToTopic(
        QString::fromStdString(_cmdVelTopic),
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            SendCmdVel(msg);
        }
    );
}

void RobotController::ConfigureImuStream() {
    if (!_enableImuStream)
    {
        return;
    }

    if (_imuRateHz <= 0)
    {
        qDebug() << "Invalid IMU stream rate, keeping firmware default:" << _imuRateHz;
    }
    else
    {
        const int imuPeriodMs = std::max(1, static_cast<int>((1000.0 / _imuRateHz) + 0.5));
        qDebug() << "Setting IMU feedback period:" << imuPeriodMs << "ms";
        SendCommandWithValue(WAVE_ROVER_COMMAND_TYPE::BASE_FEEDBACK_RATE, imuPeriodMs);
    }

    SendCommandWithValue(WAVE_ROVER_COMMAND_TYPE::BASE_FEEDBACK_ENABLE, 1);
}

void RobotController::SerialLineReceived(QString line) {
    PublishImuFrame(line);
}

void RobotController::PublishSerialHealth() {
    nlohmann::json status = {
        {"successful_writes", _pUARTSerialPort->successfulWrites()},
        {"failed_writes", _pUARTSerialPort->failedWrites()},
        {"timeouts", _pUARTSerialPort->timeouts()},
        {"reopen_attempts", _pUARTSerialPort->reopenAttempts()},
        {"consecutive_failures", _pUARTSerialPort->consecutiveFailures()},
        {"last_successful_write_unix_ms", _pUARTSerialPort->lastSuccessfulWriteMs()},
        {"malformed_imu_frames", _malformedImuFrames},
        {"last_valid_imu_frame_unix_ms", _lastValidImuFrameMs}
    };
    _pROS2Subscriber->PublishSerialHealth(status.dump());
}

void RobotController::PublishImuFrame(const QString& line) {
    const std::string text = line.toStdString();
    int frameType = 0;
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
    double gx = 0.0;
    double gy = 0.0;
    double gz = 0.0;

    try
    {
        nlohmann::json message_json = nlohmann::json::parse(text);

        if (!message_json.contains("T"))
        {
            return;
        }

        frameType = message_json["T"].get<int>();
        if (frameType != FEEDBACK_BASE_INFO)
        {
            return;
        }

        const char* requiredFields[] = {"ax", "ay", "az", "gx", "gy", "gz"};
        for (const char* field : requiredFields)
        {
            if (!message_json.contains(field) || !message_json[field].is_number())
            {
                ++_malformedImuFrames;
                RCLCPP_WARN_THROTTLE(
                    _pROS2Subscriber->get_logger(),
                    *_pROS2Subscriber->get_clock(), 5000,
                    "Skipping malformed IMU frames; count=%llu.",
                    static_cast<unsigned long long>(_malformedImuFrames));
                return;
            }
        }

        ax = message_json["ax"].get<double>();
        ay = message_json["ay"].get<double>();
        az = message_json["az"].get<double>();
        gx = message_json["gx"].get<double>();
        gy = message_json["gy"].get<double>();
        gz = message_json["gz"].get<double>();
    }
    catch (const std::exception& error)
    {
        if (!ExtractIntField(text, "T", frameType) || frameType != FEEDBACK_BASE_INFO)
        {
            return;
        }

        if (!ExtractNumberField(text, "ax", ax) ||
            !ExtractNumberField(text, "ay", ay) ||
            !ExtractNumberField(text, "az", az) ||
            !ExtractNumberField(text, "gx", gx) ||
            !ExtractNumberField(text, "gy", gy) ||
            !ExtractNumberField(text, "gz", gz))
        {
            ++_malformedImuFrames;
            RCLCPP_WARN_THROTTLE(
                _pROS2Subscriber->get_logger(),
                *_pROS2Subscriber->get_clock(), 5000,
                "Skipping malformed IMU frames; count=%llu (%s).",
                static_cast<unsigned long long>(_malformedImuFrames),
                error.what());
            return;
        }
    }

    _pROS2Subscriber->PublishImu(ax, ay, az, gx, gy, gz);
    _lastValidImuFrameMs = QDateTime::currentMSecsSinceEpoch();
}

bool RobotController::SendCmdVel(geometry_msgs::msg::Twist::SharedPtr msg){
    if (!msg)
    {
        return false;
    }

    if (_controlMode == ControlMode::ManualLR)
    {
        RCLCPP_DEBUG(
            _pROS2Subscriber->get_logger(),
            "Ignoring %s while control_mode is manual_lr.",
            _cmdVelTopic.c_str()
        );
        return false;
    }

    if (_controlMode == ControlMode::FixedWing)
    {
        return SendFixedWingCmdVel(*msg);
    }

    return SendVelocityCommand(msg->linear.x, msg->angular.z);
}

bool RobotController::SendFixedWingCmdVel(const geometry_msgs::msg::Twist& msg) {
    if (IsFixedWingStopRequest(msg))
    {
        RCLCPP_DEBUG(
            _pROS2Subscriber->get_logger(),
            "fixed_wing explicit stop marker -> stop."
        );
        return SendLeftRightVelocityCommand(0.0, 0.0);
    }

    if (IsZeroTwist(msg))
    {
        RCLCPP_DEBUG(
            _pROS2Subscriber->get_logger(),
            "fixed_wing discrete teleop: zero Twist -> straight."
        );
        return SendLeftRightVelocityCommand(FIXED_WING_STRAIGHT_SPEED, FIXED_WING_STRAIGHT_SPEED);
    }

    // teleop_twist_keyboard reverses angular.z on the reverse diagonals (M and >).
    const bool reverseTeleopDiagonal = msg.linear.x < -ZERO_TWIST_EPSILON;
    if (msg.angular.z > ZERO_TWIST_EPSILON)
    {
        if (reverseTeleopDiagonal)
        {
            RCLCPP_DEBUG(
                _pROS2Subscriber->get_logger(),
                "fixed_wing discrete teleop: linear.x=%.3f angular.z=%.3f -> bank right.",
                msg.linear.x,
                msg.angular.z
            );
            return SendLeftRightVelocityCommand(FIXED_WING_TURN_OUTER_SPEED, FIXED_WING_BANK_INNER_SPEED);
        }

        RCLCPP_DEBUG(
            _pROS2Subscriber->get_logger(),
            "fixed_wing discrete teleop: linear.x=%.3f angular.z=%.3f -> bank left.",
            msg.linear.x,
            msg.angular.z
        );
        return SendLeftRightVelocityCommand(FIXED_WING_BANK_INNER_SPEED, FIXED_WING_TURN_OUTER_SPEED);
    }

    if (msg.angular.z < -ZERO_TWIST_EPSILON)
    {
        if (reverseTeleopDiagonal)
        {
            RCLCPP_DEBUG(
                _pROS2Subscriber->get_logger(),
                "fixed_wing discrete teleop: linear.x=%.3f angular.z=%.3f -> bank left.",
                msg.linear.x,
                msg.angular.z
            );
            return SendLeftRightVelocityCommand(FIXED_WING_BANK_INNER_SPEED, FIXED_WING_TURN_OUTER_SPEED);
        }

        RCLCPP_DEBUG(
            _pROS2Subscriber->get_logger(),
            "fixed_wing discrete teleop: linear.x=%.3f angular.z=%.3f -> bank right.",
            msg.linear.x,
            msg.angular.z
        );
        return SendLeftRightVelocityCommand(FIXED_WING_TURN_OUTER_SPEED, FIXED_WING_BANK_INNER_SPEED);
    }

    if (msg.linear.x > ZERO_TWIST_EPSILON)
    {
        RCLCPP_DEBUG(
            _pROS2Subscriber->get_logger(),
            "fixed_wing discrete teleop: linear.x=%.3f angular.z=%.3f -> straight.",
            msg.linear.x,
            msg.angular.z
        );
        return SendLeftRightVelocityCommand(FIXED_WING_STRAIGHT_SPEED, FIXED_WING_STRAIGHT_SPEED);
    }

    RCLCPP_DEBUG(
        _pROS2Subscriber->get_logger(),
        "fixed_wing discrete teleop: linear.x=%.3f angular.z=%.3f -> straight.",
        msg.linear.x,
        msg.angular.z
    );
    return SendLeftRightVelocityCommand(FIXED_WING_STRAIGHT_SPEED, FIXED_WING_STRAIGHT_SPEED);
}

bool RobotController::SendManualLeftRightCommand(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (!msg || msg->data.size() < 2)
    {
        RCLCPP_WARN(
            _pROS2Subscriber->get_logger(),
            "manual_lr command must contain [left, right]; sending zero wheel speeds."
        );
        _manualCommandActive = false;
        return SendLeftRightVelocityCommand(0.0, 0.0);
    }

    const double left = msg->data[0];
    const double right = msg->data[1];

    if (!std::isfinite(left) || !std::isfinite(right))
    {
        RCLCPP_WARN(
            _pROS2Subscriber->get_logger(),
            "manual_lr command contains non-finite values; sending zero wheel speeds."
        );
        _manualCommandActive = false;
        return SendLeftRightVelocityCommand(0.0, 0.0);
    }

    _lastManualCommandTime = std::chrono::steady_clock::now();
    _manualCommandActive = true;
    return SendLeftRightVelocityCommand(left, right);
}

void RobotController::CheckManualCommandTimeout() {
    if (_controlMode != ControlMode::ManualLR || !_manualCommandActive)
    {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsedSec = std::chrono::duration<double>(now - _lastManualCommandTime).count();
    if (elapsedSec <= _manualLrTimeoutSec)
    {
        return;
    }

    RCLCPP_WARN(
        _pROS2Subscriber->get_logger(),
        "No manual_lr command received for %.3fs; sending zero wheel speeds.",
        elapsedSec
    );
    _manualCommandActive = false;
    SendLeftRightVelocityCommand(0.0, 0.0);
}

bool RobotController::SendVelocityCommand(double linear_x, double angular_z) {
    double x = Clamp(linear_x, -TWIST_INPUT_LIMIT, TWIST_INPUT_LIMIT);
    double z = Clamp(angular_z, -TWIST_INPUT_LIMIT, TWIST_INPUT_LIMIT);

    double l = x - z;
    double r = x + z;

    l = Clamp(l, -TWIST_WHEEL_COMMAND_LIMIT, TWIST_WHEEL_COMMAND_LIMIT);
    r = Clamp(r, -TWIST_WHEEL_COMMAND_LIMIT, TWIST_WHEEL_COMMAND_LIMIT);

    return SendLeftRightVelocityCommand(l, r);
}

bool RobotController::SendLeftRightVelocityCommand(double left, double right) {
    nlohmann::json message_json = {};
    const double safeLeft = std::isfinite(left) ? left : 0.0;
    const double safeRight = std::isfinite(right) ? right : 0.0;

    message_json["T"] = static_cast<int>(WAVE_ROVER_COMMAND_TYPE::SPEED_INPUT);
    message_json["L"] = Clamp(
        safeLeft,
        -WHEEL_COMMAND_LIMIT,
        WHEEL_COMMAND_LIMIT
    );
    message_json["R"] = Clamp(
        safeRight,
        -WHEEL_COMMAND_LIMIT,
        WHEEL_COMMAND_LIMIT
    );

    emit SendVelocityRequest(QString::fromStdString(message_json.dump()));

    return true;
}

bool RobotController::SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command) {
    nlohmann::json message_json = {};
    message_json["T"] = static_cast<int>(command);
    emit SendOrderedRequest(QString::fromStdString(message_json.dump()));
    return true;
}

bool RobotController::SendCommandWithValue(WAVE_ROVER_COMMAND_TYPE command, int value) {
    nlohmann::json message_json = {};
    message_json["T"] = static_cast<int>(command);
    message_json["cmd"] = value;
    emit SendOrderedRequest(QString::fromStdString(message_json.dump()));
    return true;
}

bool RobotController::SendEmergencyStop() {
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::EMERGENCY_STOP);
}
