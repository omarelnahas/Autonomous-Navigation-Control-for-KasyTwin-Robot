#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

class PIDControlNode : public rclcpp::Node
{
public:
    PIDControlNode();

private:
    // ---- Callbacks ----
    void rollCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void yawCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void referenceCallback(
        const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters);

    // ---- Control ----
    void controlLoop();
    double computePID(double error,
                      double& error_integral,
                      double& error_prev,
                      double kp, double ki, double kd,
                      double dt);

    void loadPidParameters();
    double normalizeAngle(double angle);

    std::vector<double> assignWeights(double roll_error,
                                      double yaw_error,
                                      double roll_tol,
                                      double yaw_tol);

    // ---- State ----
    double control_freq_;

    double current_roll_, current_yaw_;
    double reference_roll_, reference_yaw_;
    bool has_roll_, has_yaw_, has_reference_;

    double roll_kp_, roll_ki_, roll_kd_;
    double yaw_kp_, yaw_ki_, yaw_kd_;

    double roll_error_integral_, roll_error_prev_;
    double yaw_error_integral_, yaw_error_prev_;

    // ---- ROS Interfaces ----
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr roll_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr reference_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_control_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_control_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};
