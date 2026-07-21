#pragma once
#include <QString>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>

class ROS2Subscriber : public rclcpp::Node
{
public:
    ROS2Subscriber();
    ~ROS2Subscriber() = default;

    bool SubscribeToTopic(const QString& topic, std::function<void(const geometry_msgs::msg::Twist::SharedPtr)> callback);
    bool SubscribeToManualLRTopic(
        const QString& topic,
        std::function<void(const std_msgs::msg::Float32MultiArray::SharedPtr)> callback
    );
    void PublishImu(double ax, double ay, double az, double gx, double gy, double gz);
    void PublishSerialHealth(const std::string& payload);

private:
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _twist_subscriptions;
      rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _manual_lr_subscription;
      rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_publisher;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _serial_health_publisher;
      std::string _imu_frame_id;
};
