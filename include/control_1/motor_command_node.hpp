#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/set_bool.hpp>

class MotorCommandNode : public rclcpp::Node
{
public:
    MotorCommandNode();
    ~MotorCommandNode();

    void publishZeroOutput();

private:
    void rollCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void yawCallback(const std_msgs::msg::Float64::SharedPtr msg);

    void SetLinearXService(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void publishCommand();

    // State 
    double roll_output_;
    double yaw_output_;

    // ROS Interfaces 
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr roll_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_linear_x_service_;
    rclcpp::TimerBase::SharedPtr cmd_timer_;
};
