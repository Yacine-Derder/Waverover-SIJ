#include <ROS2Subscriber.hpp>
#include <QDebug>
#include <iostream>

ROS2Subscriber::ROS2Subscriber() : rclcpp::Node("WaveRobotController") {
    this->declare_parameter("UART_address", "/dev/ttyAMA0");
    this->declare_parameter("baud_rate", 921600);
    this->declare_parameter("imu_topic", "imu/data_raw");
    this->declare_parameter("imu_frame_id", "base_footprint");

    _imu_frame_id = this->get_parameter("imu_frame_id").as_string();
    std::string imu_topic = this->get_parameter("imu_topic").as_string();

    _imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>(
        imu_topic,
        rclcpp::SensorDataQoS().reliable()
    );

    qDebug() << "ROS2 Node initialized.";
}

bool ROS2Subscriber::SubscribeToTopic(
    const QString& topic,
    std::function<void(const geometry_msgs::msg::Twist::SharedPtr)> callback
) {
    qDebug() << "Topic subscription:" << topic;
    _twist_subscriptions = this->create_subscription<geometry_msgs::msg::Twist>(
        topic.toStdString(),
        rclcpp::SensorDataQoS().reliable(),
        [callback](const geometry_msgs::msg::Twist::SharedPtr msg) {
            std::cout << " >>> ";
            callback(msg);
        }
    );

    return true;
}

bool ROS2Subscriber::SubscribeToManualLRTopic(
    const QString& topic,
    std::function<void(const std_msgs::msg::Float32MultiArray::SharedPtr)> callback
) {
    qDebug() << "Manual L/R topic subscription:" << topic;
    _manual_lr_subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        topic.toStdString(),
        rclcpp::QoS(10).reliable(),
        [callback](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            callback(msg);
        }
    );

    return true;
}

void ROS2Subscriber::PublishImu(double ax, double ay, double az, double gx, double gy, double gz)
{
    sensor_msgs::msg::Imu message;
    message.header.stamp = this->now();
    message.header.frame_id = _imu_frame_id;

    message.orientation.w = 1.0;
    message.orientation_covariance[0] = -1.0;

    message.angular_velocity.x = gx;
    message.angular_velocity.y = gy;
    message.angular_velocity.z = gz;

    message.linear_acceleration.x = ax;
    message.linear_acceleration.y = ay;
    message.linear_acceleration.z = az;

    _imu_publisher->publish(message);
}
