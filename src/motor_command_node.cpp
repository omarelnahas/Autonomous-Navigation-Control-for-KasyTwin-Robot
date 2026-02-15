#include "control_1/motor_command_node.hpp"

#include <signal.h>
#include <thread>

// ---------------- Constructor ----------------

MotorCommandNode::MotorCommandNode()
: Node("motor_command_node"),
  roll_output_(0.0),
  yaw_output_(0.0)
{
    this->declare_parameter("roll_control_topic","/controller/roll_output");
    this->declare_parameter("yaw_control_topic","/controller/yaw_output");
    this->declare_parameter("cmd_vel_topic","/karo/pid/cmd_vel");
    this->declare_parameter("max_angular_vel", 0.5);
    this->declare_parameter("linear.x", 0.0);
    this->declare_parameter("linear_velocity_enable", true);

    std::string roll_control_topic = this->get_parameter("roll_control_topic").as_string();
    std::string yaw_control_topic  = this->get_parameter("yaw_control_topic").as_string();
    std::string cmd_vel_topic      = this->get_parameter("cmd_vel_topic").as_string();

    roll_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        roll_control_topic, 10,
        std::bind(&MotorCommandNode::rollCallback, this, std::placeholders::_1));

    yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        yaw_control_topic, 10,
        std::bind(&MotorCommandNode::yawCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    set_linear_x_service_ = this->create_service<std_srvs::srv::SetBool>(
        "set_linear_x",
        std::bind(&MotorCommandNode::SetLinearXService, this,
                  std::placeholders::_1, std::placeholders::_2));

    cmd_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&MotorCommandNode::publishCommand, this));

    RCLCPP_INFO(this->get_logger(), "Motor Command Node initialized");
}

// ---------------- Destructor ----------------

MotorCommandNode::~MotorCommandNode()
{
    publishZeroOutput();
}

// ---------------- Callbacks ----------------

void MotorCommandNode::rollCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    roll_output_ = msg->data;
}

void MotorCommandNode::yawCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    yaw_output_ = msg->data;
}

// ---------------- Services ----------------

void MotorCommandNode::SetLinearXService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    this->set_parameter(
        rclcpp::Parameter("linear_velocity_enable", request->data));

    response->success = true;
    response->message = request->data ?
        "Linear.x enabled" : "Linear.x disabled";
}

// ---------------- Command Publisher ----------------

void MotorCommandNode::publishCommand()
{
    double max_ang_vel = this->get_parameter("max_angular_vel").as_double();
    double linear_x    = this->get_parameter("linear.x").as_double();
    bool linear_enable = this->get_parameter("linear_velocity_enable").as_bool();

    double combined_angular = roll_output_ + yaw_output_;
    combined_angular = std::clamp(combined_angular, -max_ang_vel, max_ang_vel);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = linear_enable ? linear_x : 0.0;
    cmd.angular.z = combined_angular;

    cmd_vel_pub_->publish(cmd);
}

// ---------------- Zero Output ----------------

void MotorCommandNode::publishZeroOutput()
{
    if (cmd_timer_) {
        cmd_timer_->cancel();
    }

    geometry_msgs::msg::Twist zero;
    zero.linear.x = 0.0;
    zero.angular.z = 0.0;

    for (int i = 0; i < 5; ++i) {
        cmd_vel_pub_->publish(zero);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ---------------- Signal Handling ----------------

static std::shared_ptr<MotorCommandNode> g_node;

void signalHandler(int)
{
    if (g_node) {
        g_node->publishZeroOutput();
    }
    rclcpp::shutdown();
}

// ---------------- Main ----------------

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    g_node = std::make_shared<MotorCommandNode>();

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    rclcpp::spin(g_node);

    g_node.reset();
    rclcpp::shutdown();
    return 0;
}
