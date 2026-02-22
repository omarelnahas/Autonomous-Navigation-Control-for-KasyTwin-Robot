#ifndef PCA_NODE_HPP
#define PCA_NODE_HPP

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <vector>

struct PCAResult {
    Eigen::Vector2d centroid;
    Eigen::Vector2d direction;
    Eigen::Vector2d eigenvalues;
    double confidence;
    double angle;
};

class PCA : public rclcpp::Node {
public:
    PCA();

private:
    // Callbacks
    void yawCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void PCA_transform(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Core computation
    PCAResult computePCA(const std::vector<Eigen::Vector2d>& pts);
    
    // NEW: Data preprocessing
    std::vector<Eigen::Vector2d> applyROIFilter(const std::vector<Eigen::Vector2d>& pts);
    std::vector<Eigen::Vector2d> removeOutliers(const std::vector<Eigen::Vector2d>& pts);
    
    // Visualization
    void publishFilteredCloud(
        const std::vector<Eigen::Vector2d>& left_side,
        const std::vector<Eigen::Vector2d>& right_side);
    
    // Utilities
    double normalizeAngle(double angle);

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;

    // Clients
    std::shared_ptr<rclcpp::AsyncParametersClient> client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stop_client_;

    // Parameters
    std::string yaw_topic_;
    std::string point_cloud_topic_;
    double centering_threshold_;
    double centering_gain_;
    double max_correction_;
    int diameter_samples_needed_;
    double diameter_variance_threshold_;
    double diameter_change_threshold_;
    
    // Temporal smoothing parameters
    double temporal_alpha_;
    
    // ROI parameters
    double roi_max_x_;
    double roi_max_y_;

    // State variables
    double current_yaw_;
    bool diameter_lock_;
    double locked_diameter_;
    std::vector<double> diameter_samples_;
    bool stop_sent_;
    bool is_centered_;
    std::chrono::steady_clock::time_point invalid_walls_start_time_;
    bool invalid_walls_timer_started_;
    const std::chrono::milliseconds INVALID_WALLS_TIMEOUT_{1000}; // Adjust duration as needed
    
    // Temporal filtering state
    double previous_base_angle_;
    double previous_yaw_reference_;
    bool first_frame_;
};

#endif // PCA_NODE_HPP