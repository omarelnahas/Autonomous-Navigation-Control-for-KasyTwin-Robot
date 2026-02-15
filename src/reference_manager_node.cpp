#include "control_1/reference_manager_node.hpp"

// ---------------- Constructor ----------------

ReferenceManagerNode::ReferenceManagerNode()
: Node("reference_manager_node")
{
    // Parameters
    this->declare_parameter("roll_topic", "/karo/roll");
    this->declare_parameter("yaw_topic", "/karo/yaw");
    this->declare_parameter("reference_topic", "/controller/reference");

    this->declare_parameter("use_config_reference", false);
    this->declare_parameter("config_reference_roll", 0.0);
    this->declare_parameter("config_reference_yaw", 0.0);
    this->declare_parameter("auto_activate", false);

    std::string roll_topic = this->get_parameter("roll_topic").as_string();
    std::string yaw_topic = this->get_parameter("yaw_topic").as_string();
    std::string reference_topic = this->get_parameter("reference_topic").as_string();

    // State init
    current_roll_ = current_yaw_ = 0.0;
    reference_roll_ = reference_yaw_ = 0.0;
    reference_set_ = false;
    reference_active_ = false;

    // Subscribers
    roll_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        roll_topic, 10,
        std::bind(&ReferenceManagerNode::rollCallback, this, std::placeholders::_1));

    yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        yaw_topic, 10,
        std::bind(&ReferenceManagerNode::yawCallback, this, std::placeholders::_1));

    // Publisher
    reference_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        reference_topic, 10);

    // Services
    set_current_service_ = this->create_service<std_srvs::srv::Trigger>(
        "set_current_as_reference",
        std::bind(&ReferenceManagerNode::setCurrentAsReference,
                  this, std::placeholders::_1, std::placeholders::_2));

    activate_service_ = this->create_service<std_srvs::srv::SetBool>(
        "activate_reference",
        std::bind(&ReferenceManagerNode::activateReference,
                  this, std::placeholders::_1, std::placeholders::_2));

    // Timer (50 Hz)
    reference_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&ReferenceManagerNode::publishReference, this));

    // Parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ReferenceManagerNode::parametersCallback,
                  this, std::placeholders::_1));

    // Load config reference if enabled
    bool use_config = this->get_parameter("use_config_reference").as_bool();
    if (use_config) {
        loadConfigReference();
    }

    bool auto_activate = this->get_parameter("auto_activate").as_bool();
    if (auto_activate && reference_set_) {
        reference_active_ = true;
        RCLCPP_INFO(this->get_logger(), "Reference auto-activated from config");
    }

    RCLCPP_INFO(this->get_logger(), "Reference Manager Node initialized");

    if (use_config) {
        RCLCPP_INFO(this->get_logger(),
            "Using config reference: roll=%.2f yaw=%.2f",
            reference_roll_, reference_yaw_);
    } else {
        RCLCPP_INFO(this->get_logger(),
            "Waiting for reference (call set_current_as_reference)");
    }
}

// ---------------- Callbacks ----------------

void ReferenceManagerNode::rollCallback(
    const std_msgs::msg::Float64::SharedPtr msg)
{
    current_roll_ = msg->data;
}

void ReferenceManagerNode::yawCallback(
    const std_msgs::msg::Float64::SharedPtr msg)
{
    current_yaw_ = msg->data;
}

// ---------------- Services ----------------

void ReferenceManagerNode::setCurrentAsReference(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    reference_roll_ = current_roll_;
    reference_yaw_  = current_yaw_;
    reference_set_ = true;
    reference_active_ = true;

    RCLCPP_INFO(this->get_logger(),
        "Reference set: roll=%.2f yaw=%.2f",
        reference_roll_, reference_yaw_);

    response->success = true;
    response->message = "Current roll and yaw set as reference.";
}

void ReferenceManagerNode::activateReference(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data && !reference_set_) {
        response->success = false;
        response->message = "Reference not set.";
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    reference_active_ = request->data;
    response->success = true;
    response->message = reference_active_ ? "Reference activated." : "Reference deactivated.";

    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

// ---------------- Publishing ----------------

void ReferenceManagerNode::publishReference()
{
    if (!reference_set_ || !reference_active_)
    {
        return;
    }

    geometry_msgs::msg::Vector3Stamped msg; 
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.vector.x = reference_roll_;
    msg.vector.y = 0.0;
    msg.vector.z = reference_yaw_;
    reference_pub_->publish(msg);
}

// ---------------- Helpers ----------------

void ReferenceManagerNode::loadConfigReference()
{
    reference_roll_ = this->get_parameter("config_reference_roll").as_double();
    reference_yaw_  = this->get_parameter("config_reference_yaw").as_double();
    reference_set_ = true;

    RCLCPP_INFO(this->get_logger(), "Loaded reference from config");
}

rcl_interfaces::msg::SetParametersResult
ReferenceManagerNode::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : parameters) {
        if (param.get_name() == "use_config_reference" && param.as_bool()) {
            loadConfigReference();
        }
        else if (param.get_name() == "config_reference_roll") {
            reference_roll_ = param.as_double();
        }
        else if (param.get_name() == "config_reference_yaw") {
            reference_yaw_ = param.as_double();
        }
        else if (param.get_name() == "auto_activate") {
            if (param.as_bool() && reference_set_) {
                reference_active_ = true;
                RCLCPP_INFO(this->get_logger(), "Reference activated");
            }
        }
    }
    return result;
}

// ---------------- Main ----------------

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReferenceManagerNode>());
    rclcpp::shutdown();
    return 0;
}
