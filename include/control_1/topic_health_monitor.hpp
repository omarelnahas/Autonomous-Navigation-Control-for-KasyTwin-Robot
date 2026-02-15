#ifndef TOPIC_HEALTH_MONITOR_HPP
#define TOPIC_HEALTH_MONITOR_HPP

#include <memory>
#include <string>
#include <map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

struct TopicHealthInfo {
    std::string name;
    rclcpp::Time last_message_time;
    size_t message_count;
    rclcpp::Time last_rate_check;
    double current_rate;
    double expected_rate;
    double timeout_sec;
    bool is_critical;
};

class TopicHealthMonitor : public rclcpp::Node {
public:
    TopicHealthMonitor();

private:
    // Callbacks for each monitored topic
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void yawCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void rollCallback(const std_msgs::msg::Float64::SharedPtr msg);
    
    // Health check timer
    void checkHealth();
    
    // Diagnostic publishing
    void publishDiagnostics();
    
    // Update topic health info
    void updateTopicHealth(const std::string& topic_name);
    
    // Check if topic is healthy
    bool isTopicHealthy(const std::string& topic_name);
    
    // Create diagnostic status for a topic
    diagnostic_msgs::msg::DiagnosticStatus createTopicDiagnostic(
        const std::string& topic_name, 
        const TopicHealthInfo& info,
        bool is_healthy);
    
    // Parameters
    double health_check_rate_;
    bool enable_safety_actions_;
    bool publish_diagnostics_;
    
    // Stored topic names
    std::string pointcloud_topic_;
    std::string yaw_topic_;
    std::string roll_topic_;
    
    // Topic health tracking
    std::map<std::string, TopicHealthInfo> topic_health_;
    
    // Safety state
    bool safety_stop_sent_;
    
    // ROS2 Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr roll_sub_;
    
    // ROS2 Publishers
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    
    // ROS2 Service Clients
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr motor_stop_client_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr health_check_timer_;
};

#endif // TOPIC_HEALTH_MONITOR_HPP