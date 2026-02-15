#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

class ReferenceManagerNode : public rclcpp::Node
{
public:
    ReferenceManagerNode();

private:
    // ---- Callbacks ----
    void rollCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void yawCallback(const std_msgs::msg::Float64::SharedPtr msg);

    void setCurrentAsReference(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void activateReference(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void publishReference();

    void loadConfigReference();

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters);

    // ---- State ----
    double current_roll_, current_yaw_;
    double reference_roll_, reference_yaw_;
    bool reference_set_;
    bool reference_active_;

    // ---- ROS Interfaces ----
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr roll_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr reference_pub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_current_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr activate_service_;

    rclcpp::TimerBase::SharedPtr reference_timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};
