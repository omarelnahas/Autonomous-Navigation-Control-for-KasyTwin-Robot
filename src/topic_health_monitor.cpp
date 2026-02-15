#include "control_1/topic_health_monitor.hpp"

#include <chrono>
#include <algorithm>

// ======================
// Constructor
// ======================
TopicHealthMonitor::TopicHealthMonitor() 
    : Node("topic_health_monitor"),
      safety_stop_sent_(false)
{
    // Declare parameters
    this->declare_parameter<double>("health_check_rate_hz", 1.0);
    this->declare_parameter<bool>("enable_safety_actions", true);
    this->declare_parameter<bool>("publish_diagnostics", true);
    
    // Point Cloud topic parameters
    this->declare_parameter<std::string>("pointcloud_topic", "/point_cloud");
    this->declare_parameter<double>("pointcloud_timeout_sec", 2.0);
    this->declare_parameter<double>("pointcloud_expected_rate_hz", 10.0);
    this->declare_parameter<bool>("pointcloud_is_critical", true);
    
    // Yaw topic parameters
    this->declare_parameter<std::string>("yaw_topic", "/karo/yaw");
    this->declare_parameter<double>("yaw_timeout_sec", 2.0);
    this->declare_parameter<double>("yaw_expected_rate_hz", 50.0);
    this->declare_parameter<bool>("yaw_is_critical", true);
    
    // Roll topic parameters
    this->declare_parameter<std::string>("roll_topic", "/karo/roll");
    this->declare_parameter<double>("roll_timeout_sec", 2.0);
    this->declare_parameter<double>("roll_expected_rate_hz", 50.0);
    this->declare_parameter<bool>("roll_is_critical", false);
    
    // Motor stop service
    this->declare_parameter<std::string>("motor_stop_service", "/set_linear_x");
    
    // Get parameters
    health_check_rate_ = this->get_parameter("health_check_rate_hz").as_double();
    enable_safety_actions_ = this->get_parameter("enable_safety_actions").as_bool();
    publish_diagnostics_ = this->get_parameter("publish_diagnostics").as_bool();
    
    // Store topic names
    pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
    yaw_topic_ = this->get_parameter("yaw_topic").as_string();
    roll_topic_ = this->get_parameter("roll_topic").as_string();
    
    // Initialize topic health info
    rclcpp::Time init_time = this->now();
    
    topic_health_[pointcloud_topic_] = {
        pointcloud_topic_,
        init_time,
        0,
        this->now(),
        0.0,
        this->get_parameter("pointcloud_expected_rate_hz").as_double(),
        this->get_parameter("pointcloud_timeout_sec").as_double(),
        this->get_parameter("pointcloud_is_critical").as_bool()
    };
    
    topic_health_[yaw_topic_] = {
        yaw_topic_,
        init_time,
        0,
        this->now(),
        0.0,
        this->get_parameter("yaw_expected_rate_hz").as_double(),
        this->get_parameter("yaw_timeout_sec").as_double(),
        this->get_parameter("yaw_is_critical").as_bool()
    };
    
    topic_health_[roll_topic_] = {
        roll_topic_,
        init_time,
        0,
        this->now(),
        0.0,
        this->get_parameter("roll_expected_rate_hz").as_double(),
        this->get_parameter("roll_timeout_sec").as_double(),
        this->get_parameter("roll_is_critical").as_bool()
    };
    
    // Create subscribers
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, 10,
        std::bind(&TopicHealthMonitor::pointCloudCallback, this, std::placeholders::_1));
    
    yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        yaw_topic_, 10,
        std::bind(&TopicHealthMonitor::yawCallback, this, std::placeholders::_1));
    
    roll_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        roll_topic_, 10,
        std::bind(&TopicHealthMonitor::rollCallback, this, std::placeholders::_1));
    
    // Create publishers
    if (publish_diagnostics_) {
        diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", 10);
    }
    
    // Create service client
    if (enable_safety_actions_) {
        std::string stop_service = this->get_parameter("motor_stop_service").as_string();
        motor_stop_client_ = this->create_client<std_srvs::srv::SetBool>(stop_service);
    }
    
    // Create timer
    auto period = std::chrono::duration<double>(1.0 / health_check_rate_);
    health_check_timer_ = this->create_wall_timer(
        period, std::bind(&TopicHealthMonitor::checkHealth, this));
    
    RCLCPP_INFO(this->get_logger(), "Topic Health Monitor initialized");
    RCLCPP_INFO(this->get_logger(), "Monitoring %zu topics:", topic_health_.size());
    for (const auto& [topic, info] : topic_health_) {
        RCLCPP_INFO(this->get_logger(), "  - %s (rate: %.1f Hz, timeout: %.1f s, critical: %s)",
                    topic.c_str(), info.expected_rate, info.timeout_sec,
                    info.is_critical ? "YES" : "NO");
    }
}

// ======================
// Topic Callbacks
// ======================
void TopicHealthMonitor::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr) {
    updateTopicHealth(pointcloud_topic_);
}

void TopicHealthMonitor::yawCallback(const std_msgs::msg::Float64::SharedPtr) {
    updateTopicHealth(yaw_topic_);
}

void TopicHealthMonitor::rollCallback(const std_msgs::msg::Float64::SharedPtr) {
    updateTopicHealth(roll_topic_);
}

// ======================
// Health Monitoring
// ======================
void TopicHealthMonitor::updateTopicHealth(const std::string& topic_name) {
    auto& info = topic_health_[topic_name];
    info.last_message_time = this->now();
    info.message_count++;
}

bool TopicHealthMonitor::isTopicHealthy(const std::string& topic_name) {
    const auto& info = topic_health_[topic_name];
    auto now = this->now();
    auto timeout_duration = rclcpp::Duration::from_seconds(info.timeout_sec);
    
    return (now - info.last_message_time) < timeout_duration;
}

void TopicHealthMonitor::checkHealth() {
    auto now = this->now();
    bool any_critical_unhealthy = false;
    
    // Update rates and check health for all topics
    for (auto& [topic, info] : topic_health_) {
        // Calculate message rate
        auto time_since_last_check = (now - info.last_rate_check).seconds();
        if (time_since_last_check > 0.0) {
            info.current_rate = info.message_count / time_since_last_check;
            info.message_count = 0;
            info.last_rate_check = now;
        }
        
        // Check health
        bool is_healthy = isTopicHealthy(topic);
        
        if (!is_healthy) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Topic '%s' not receiving data (timeout: %.1f sec)",
                topic.c_str(), info.timeout_sec);
            
            if (info.is_critical) {
                any_critical_unhealthy = true;
            }
        }
        
    }
    
    // Safety actions
    if (enable_safety_actions_) {
        if (any_critical_unhealthy && !safety_stop_sent_) {
            RCLCPP_ERROR(this->get_logger(), 
                        "Critical topic(s) unhealthy - initiating safety stop");
            
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = false;
            
            if (motor_stop_client_->wait_for_service(std::chrono::milliseconds(100))) {
                auto result_future = motor_stop_client_->async_send_request(request);
                safety_stop_sent_ = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Motor stop service not available!");
            }
        }
        else if (!any_critical_unhealthy && safety_stop_sent_) {
            RCLCPP_INFO(this->get_logger(), 
                       "All critical topics healthy - resuming operation");
            
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = true;
            
            if (motor_stop_client_->wait_for_service(std::chrono::milliseconds(100))) {
                auto result_future = motor_stop_client_->async_send_request(request);
                safety_stop_sent_ = false;
            }
        }
    }
    
    // Publish diagnostics
    if (publish_diagnostics_) {
        publishDiagnostics();
    }
}

// ======================
// Diagnostics
// ======================
diagnostic_msgs::msg::DiagnosticStatus TopicHealthMonitor::createTopicDiagnostic(
    const std::string& topic_name,
    const TopicHealthInfo& info,
    bool is_healthy)
{
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "topic_health: " + topic_name;
    status.hardware_id = "topic_monitor";
    
    // Set level
    if (!is_healthy) {
        status.level = info.is_critical ? 
            diagnostic_msgs::msg::DiagnosticStatus::ERROR :
            diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Topic timeout";
    } else if (info.expected_rate > 0 && 
               info.current_rate < info.expected_rate * 0.8) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Low message rate";
    } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "OK";
    }
    
    // Add key-value pairs
    diagnostic_msgs::msg::KeyValue kv_rate;
    kv_rate.key = "current_rate_hz";
    kv_rate.value = std::to_string(info.current_rate);
    status.values.push_back(kv_rate);
    
    diagnostic_msgs::msg::KeyValue kv_expected;
    kv_expected.key = "expected_rate_hz";
    kv_expected.value = std::to_string(info.expected_rate);
    status.values.push_back(kv_expected);
    
    double time_since_last = (this->now() - info.last_message_time).seconds();
    diagnostic_msgs::msg::KeyValue kv_last;
    kv_last.key = "time_since_last_msg_sec";
    kv_last.value = std::to_string(time_since_last);
    status.values.push_back(kv_last);
    
    diagnostic_msgs::msg::KeyValue kv_critical;
    kv_critical.key = "is_critical";
    kv_critical.value = info.is_critical ? "true" : "false";
    status.values.push_back(kv_critical);
    
    return status;
}

void TopicHealthMonitor::publishDiagnostics() {
    auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
    diag_array.header.stamp = this->now();
    
    for (const auto& [topic, info] : topic_health_) {
        bool is_healthy = isTopicHealthy(topic);
        auto status = createTopicDiagnostic(topic, info, is_healthy);
        diag_array.status.push_back(status);
    }
    
    // Add overall system status
    diagnostic_msgs::msg::DiagnosticStatus overall;
    overall.name = "topic_health_monitor: overall";
    overall.hardware_id = "topic_monitor";
    
    int total_topics = 0;
    int healthy_topics = 0;
    int critical_unhealthy = 0;
    int non_critical_unhealthy = 0;
    
    for (const auto& [topic, info] : topic_health_) {
        total_topics++;
        bool is_healthy = isTopicHealthy(topic);
        
        if (is_healthy) {
            healthy_topics++;
        } else {
            if (info.is_critical) {
                critical_unhealthy++;
            } else {
                non_critical_unhealthy++;
            }
        }
    }
    
    if (critical_unhealthy > 0) {
        overall.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        overall.message = std::to_string(critical_unhealthy) + " critical topic(s) unhealthy";
    } else if (non_critical_unhealthy > 0) {
        overall.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        overall.message = std::to_string(non_critical_unhealthy) + " non-critical topic(s) unhealthy";
    } else {
        overall.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        overall.message = "All " + std::to_string(total_topics) + " topics healthy";
    }
    
    // Add summary values
    diagnostic_msgs::msg::KeyValue kv_total;
    kv_total.key = "total_topics";
    kv_total.value = std::to_string(total_topics);
    overall.values.push_back(kv_total);
    
    diagnostic_msgs::msg::KeyValue kv_healthy;
    kv_healthy.key = "healthy_topics";
    kv_healthy.value = std::to_string(healthy_topics);
    overall.values.push_back(kv_healthy);
    
    diagnostic_msgs::msg::KeyValue kv_unhealthy;
    kv_unhealthy.key = "unhealthy_topics";
    kv_unhealthy.value = std::to_string(total_topics - healthy_topics);
    overall.values.push_back(kv_unhealthy);
    
    diag_array.status.push_back(overall);
    
    diagnostics_pub_->publish(diag_array);
}

// ======================
// main()
// ======================
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    // Use MultiThreadedExecutor to handle callbacks concurrently
    auto node = std::make_shared<TopicHealthMonitor>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}