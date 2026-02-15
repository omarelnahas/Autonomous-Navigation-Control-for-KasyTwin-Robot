#include "control_1/pid_control_node.hpp"
#include <cmath>

// ---------------- Constructor ----------------

PIDControlNode::PIDControlNode()
: Node("pid_control_node")
{
    this->declare_parameter("roll_topic","/karo/roll");
    this->declare_parameter("yaw_topic","/karo/yaw");
    this->declare_parameter("reference_topic","/controller/reference");
    this->declare_parameter("roll_control_topic","/controller/roll_output");
    this->declare_parameter("yaw_control_topic","/controller/yaw_output");
    this->declare_parameter("control_frequency", 50.0);

    this->declare_parameter("roll_tolerance", 0.05);
    this->declare_parameter("yaw_tolerance", 0.087);

    this->declare_parameter("roll_kp", 1.0);
    this->declare_parameter("roll_ki", 0.01);
    this->declare_parameter("roll_kd", 0.1);
    this->declare_parameter("yaw_kp", 0.8);
    this->declare_parameter("yaw_ki", 0.01);
    this->declare_parameter("yaw_kd", 0.1);

    std::string roll_topic = this->get_parameter("roll_topic").as_string();
    std::string yaw_topic = this->get_parameter("yaw_topic").as_string();
    std::string reference_topic = this->get_parameter("reference_topic").as_string();
    std::string roll_control_topic = this->get_parameter("roll_control_topic").as_string();
    std::string yaw_control_topic = this->get_parameter("yaw_control_topic").as_string();

    control_freq_ = this->get_parameter("control_frequency").as_double();

    roll_error_integral_ = roll_error_prev_ = 0.0;
    yaw_error_integral_  = yaw_error_prev_  = 0.0;

    current_roll_ = current_yaw_ = 0.0;
    reference_roll_ = reference_yaw_ = 0.0;
    has_roll_ = has_yaw_ = has_reference_ = false;

    loadPidParameters();

    roll_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        roll_topic, 10,
        std::bind(&PIDControlNode::rollCallback, this, std::placeholders::_1));

    yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        yaw_topic, 10,
        std::bind(&PIDControlNode::yawCallback, this, std::placeholders::_1));

    reference_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        reference_topic, 10,
        std::bind(&PIDControlNode::referenceCallback, this, std::placeholders::_1));

    roll_control_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        roll_control_topic, 10);

    yaw_control_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        yaw_control_topic, 10);

    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / control_freq_),
        std::bind(&PIDControlNode::controlLoop, this));

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&PIDControlNode::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "PID Control Node initialized.");
}

// ---------------- Callbacks ----------------

void PIDControlNode::rollCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    current_roll_ = msg->data;
    has_roll_ = true;
}

void PIDControlNode::yawCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    current_yaw_ = msg->data;
    has_yaw_ = true;
}

void PIDControlNode::referenceCallback(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
    reference_roll_ = msg->vector.x;
    reference_yaw_  = msg->vector.z;
    has_reference_ = true;
}

// ---------------- Control ----------------

void PIDControlNode::controlLoop()
{
    if (!has_roll_ || !has_yaw_ || !has_reference_)
        return;

    double roll_tol = this->get_parameter("roll_tolerance").as_double();
    double yaw_tol  = this->get_parameter("yaw_tolerance").as_double();

    double roll_error = normalizeAngle(reference_roll_ - current_roll_);
    double yaw_error  = normalizeAngle(reference_yaw_ - current_yaw_);

    double dt = 1.0 / control_freq_;

    double roll_output = computePID(
        roll_error, roll_error_integral_, roll_error_prev_,
        roll_kp_, roll_ki_, roll_kd_, dt);

    double yaw_output = computePID(
        yaw_error, yaw_error_integral_, yaw_error_prev_,
        yaw_kp_, yaw_ki_, yaw_kd_, dt);

    auto weights = assignWeights(roll_error, yaw_error, roll_tol, yaw_tol);

    std_msgs::msg::Float64 roll_msg;
    roll_msg.data = roll_output * weights[0];
    roll_control_pub_->publish(roll_msg);

    std_msgs::msg::Float64 yaw_msg;
    yaw_msg.data = yaw_output * weights[1];
    yaw_control_pub_->publish(yaw_msg);
}

// ---------------- Helpers ----------------

double PIDControlNode::computePID(double error,
                                  double& error_integral,
                                  double& error_prev,
                                  double kp, double ki, double kd,
                                  double dt)
{
    double p = kp * error;

    error_integral += error * dt;
    error_integral = std::clamp(error_integral, -1.0, 1.0);

    double i = ki * error_integral;
    double d = (dt > 0) ? kd * (error - error_prev) / dt : 0.0;

    error_prev = error;
    return p + i + d;
}

double PIDControlNode::normalizeAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

void PIDControlNode::loadPidParameters()
{
    roll_kp_ = this->get_parameter("roll_kp").as_double();
    roll_ki_ = this->get_parameter("roll_ki").as_double();
    roll_kd_ = this->get_parameter("roll_kd").as_double();

    yaw_kp_ = this->get_parameter("yaw_kp").as_double();
    yaw_ki_ = this->get_parameter("yaw_ki").as_double();
    yaw_kd_ = this->get_parameter("yaw_kd").as_double();
}

std::vector<double> PIDControlNode::assignWeights(
    double roll_error, double yaw_error,
    double roll_tol, double yaw_tol)
{
    double roll_weight = 0.0;
    double yaw_weight = 0.0;

    // Assign weights based on error magnitude (2.0 if beyond 1.5x tolerance, else 1.0)
    if (std::abs(roll_error) > roll_tol)
        roll_weight = (std::abs(roll_error) > 1.5 * roll_tol) ? 2.0 : 1.0;

    if (std::abs(yaw_error) > yaw_tol)
        yaw_weight = (std::abs(yaw_error) > 1.5 * yaw_tol) ? 2.0 : 1.0;

    return {roll_weight, yaw_weight};
}

rcl_interfaces::msg::SetParametersResult
PIDControlNode::parametersCallback(const std::vector<rclcpp::Parameter>&)
{
    loadPidParameters();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

// ---------------- Main ----------------

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDControlNode>());
    rclcpp::shutdown();
    return 0;
}
