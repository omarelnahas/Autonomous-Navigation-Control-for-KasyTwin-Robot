#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <iostream>
#include <random>

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
    LaserscanToPointcloud() : Node("laserscan_to_pointcloud"),
        rng_(std::random_device{}()),
        range_noise_(0.0, 0.015),      // Gaussian: 15mm std dev (statistical error from datasheet)
        angular_noise_(0.0, 0.00109),  // Gaussian: half of 0.125deg resolution in radians
        dropout_dist_(0.0, 1.0)        // Uniform for dropout
    {
        // Declare parameters
        this->declare_parameter<std::string>("pointcloud_topic", "/point_cloud");
        this->declare_parameter<double>("min_angle_rad", 0.87266);     // 50 degrees
        this->declare_parameter<double>("max_angle_rad", 2.26892);     // 130 degrees

        // Noise parameters based on SICK multiScan136 (MULS1AA-112211) datasheet
        this->declare_parameter<double>("range_noise_std", 0.015);      // Statistical error <= 15mm (1-sigma)
        this->declare_parameter<double>("range_systematic_bias", 0.05); // Systematic error +/-50mm (set between -0.05 and 0.05)
        this->declare_parameter<double>("angular_noise_std", 0.00109);  // Half of 0.125deg resolution in radians
        this->declare_parameter<double>("dropout_probability", 0.01);   // 1% dropout (sensor has >99% detection probability)

        // Get parameters
        point_cloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
        min_angle_ = this->get_parameter("min_angle_rad").as_double();
        max_angle_ = this->get_parameter("max_angle_rad").as_double();
        range_systematic_bias_ = this->get_parameter("range_systematic_bias").as_double();
        dropout_prob_ = this->get_parameter("dropout_probability").as_double();

        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // Subscribers
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar", qos_profile,
            std::bind(&LaserscanToPointcloud::laser_callback, this, std::placeholders::_1));

        // Publisher
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "LaserscanToPointcloud started with SICK multiScan136 noise model");
        RCLCPP_INFO(this->get_logger(), "  range_noise_std:       %.4f m", this->get_parameter("range_noise_std").as_double());
        RCLCPP_INFO(this->get_logger(), "  range_systematic_bias: %.4f m", range_systematic_bias_);
        RCLCPP_INFO(this->get_logger(), "  angular_noise_std:     %.5f rad", this->get_parameter("angular_noise_std").as_double());
        RCLCPP_INFO(this->get_logger(), "  dropout_probability:   %.4f", dropout_prob_);
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

        // Update noise distributions from parameters (allows runtime tuning via ros2 param set)
        double range_std = this->get_parameter("range_noise_std").as_double();
        double angular_std = this->get_parameter("angular_noise_std").as_double();
        range_systematic_bias_ = this->get_parameter("range_systematic_bias").as_double();
        dropout_prob_ = this->get_parameter("dropout_probability").as_double();
        range_noise_ = std::normal_distribution<double>(0.0, range_std);
        angular_noise_ = std::normal_distribution<double>(0.0, angular_std);

        std::vector<std::pair<double, double>> scan_points;
        double angle = msg->angle_min;

        for (size_t i = 0; i < msg->ranges.size(); i++) {
            if (!std::isinf(msg->ranges[i]) && !std::isnan(msg->ranges[i])) {
                double abs_angle = std::abs(angle);
                if (abs_angle >= min_angle_ && abs_angle <= max_angle_) {

                    // Random point dropout — simulates surface reflectivity issues,
                    // glass, dark surfaces, near-grazing incidence angles
                    // multiScan136 has >99% detection probability so dropout = 1%
                    if (dropout_dist_(rng_) < dropout_prob_) {
                        angle += msg->angle_increment;
                        continue;
                    }

                    // Systematic bias — fixed offset representing sensor calibration error
                    // multiScan136 systematic error: +/-50mm, set bias between -0.05 and 0.05
                    double biased_range = msg->ranges[i] + range_systematic_bias_;

                    // Statistical (random) Gaussian noise on range
                    // multiScan136 statistical error: <= 15mm (1-sigma)
                    double noisy_range = biased_range + range_noise_(rng_);

                    // Gaussian angular jitter — simulates encoder imprecision and vibration
                    // Based on 0.125deg angular resolution of multiScan136
                    double noisy_angle = angle + angular_noise_(rng_);

                    // Clamp range to valid sensor bounds
                    noisy_range = std::max((double)msg->range_min, std::min(noisy_range, (double)msg->range_max));

                    double px = std::cos(noisy_angle) * noisy_range;
                    double py = std::sin(noisy_angle) * noisy_range;
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

    // Noise generators
    std::mt19937 rng_;
    std::normal_distribution<double> range_noise_;
    std::normal_distribution<double> angular_noise_;
    std::uniform_real_distribution<double> dropout_dist_;

    // Noise parameters
    double range_systematic_bias_;
    double dropout_prob_;

    // Node parameters
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