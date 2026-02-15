#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>



class LaserscanToPointcloud : public rclcpp::Node {
public:
    LaserscanToPointcloud() : Node("laserscan_to_pointcloud") {
        // Declare parameters
        this->declare_parameter<std::string>("pointcloud_topic", "/point_cloud");
        this->declare_parameter<double>("min_angle_rad", 0.87266);     // 50 degrees
        this->declare_parameter<double>("max_angle_rad", 2.26892);     // 130 degrees

        // Get parameters
        point_cloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
        min_angle_ = this->get_parameter("min_angle_rad").as_double();
        max_angle_ = this->get_parameter("max_angle_rad").as_double();

        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // Subscribers
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar", qos_profile,
            std::bind(&LaserscanToPointcloud::laser_callback, this, std::placeholders::_1));
        // Publisher
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_topic_, 10);

    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<std::pair<double, double>> scan_points;
        double angle = msg->angle_min;

        // Filter points based on angle range
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            if (!std::isinf(msg->ranges[i]) && !std::isnan(msg->ranges[i])) {
                double abs_angle = std::abs(angle);
                if (abs_angle >= min_angle_ && abs_angle <= max_angle_) {
                    double px = std::cos(angle) * msg->ranges[i];
                    double py = std::sin(angle) * msg->ranges[i];
                    scan_points.emplace_back(px, py);
                }
            }
            angle += msg->angle_increment;
        }

        // Convert to point cloud
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        for (const auto& pt : scan_points) {
            pcl::PointXYZRGB point;
            point.x = pt.first;
            point.y = pt.second;
            point.z = 0.0;
            point.r = 255;
            point.g = 255;
            point.b = 255;
            cloud.points.push_back(point);
        }

        // Publish point cloud
        auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(cloud, *pc2_msg);
        pc2_msg->header.frame_id = "lidar_link";
        pc2_msg->header.stamp = this->now();
        cloud_pub_->publish(*pc2_msg);
    }
    
    // Parameters
    std::string point_cloud_topic_;
    double min_angle_;
    double max_angle_;

    // ROS2 objects
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserscanToPointcloud>());
    rclcpp::shutdown();
    return 0;
}