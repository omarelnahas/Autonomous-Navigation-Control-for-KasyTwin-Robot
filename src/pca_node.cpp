#include "control_1/pca_node.hpp"

#include <chrono>
#include <cmath>
#include <iostream>

#include "rclcpp/parameter_client.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// ======================
// Constructor
// ======================
PCA::PCA() : Node("pca_node") {

    this->declare_parameter<std::string>("yaw_topic", "/karo/yaw");
    this->declare_parameter<std::string>("pointcloud_topic", "/point_cloud");
    this->declare_parameter<double>("centering_threshold", 0.15);
    this->declare_parameter<double>("centering_gain", 0.3);
    this->declare_parameter<double>("max_correction_rad", 0.26);
    this->declare_parameter<int>("diameter_samples_for_lock", 20);
    this->declare_parameter<double>("diameter_variance_threshold", 0.05);
    
    // Temporal smoothing parameters
    this->declare_parameter<double>("temporal_alpha", 0.8);  // 0.8 = 80% previous, 20% new
    
    //  ROI parameters
    this->declare_parameter<double>("roi_max_x", 5.0);  // Max distance ahead (meters)
    this->declare_parameter<double>("roi_max_y", 3.0);  // Max lateral distance (meters)

    yaw_topic_ = this->get_parameter("yaw_topic").as_string();
    point_cloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
    centering_threshold_ = this->get_parameter("centering_threshold").as_double();
    centering_gain_ = this->get_parameter("centering_gain").as_double();
    max_correction_ = this->get_parameter("max_correction_rad").as_double();
    diameter_samples_needed_ = this->get_parameter("diameter_samples_for_lock").as_int();
    diameter_variance_threshold_ = this->get_parameter("diameter_variance_threshold").as_double();
    
    temporal_alpha_ = this->get_parameter("temporal_alpha").as_double();
    roi_max_x_ = this->get_parameter("roi_max_x").as_double();
    roi_max_y_ = this->get_parameter("roi_max_y").as_double();
    
    current_yaw_ = 0.0;
    diameter_lock_ = false;
    locked_diameter_ = 0.0;
    stop_sent_ = false;
    is_centered_ = false;
    invalid_walls_timer_started_ = false;
    
    // Initialize temporal filtering state
    previous_base_angle_ = 0.0;
    previous_yaw_reference_ = 0.0;
    first_frame_ = true;

    yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        yaw_topic_, 10,
        std::bind(&PCA::yawCallback, this, std::placeholders::_1));

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic_, 10,
        std::bind(&PCA::PCA_transform, this, std::placeholders::_1));

    client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        this, "/reference_manager");

    stop_client_ = this->create_client<std_srvs::srv::SetBool>(
        "/set_linear_x");
    
    //  Publisher for filtered point clouds (for visualization)
    filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/filtered_point_cloud", 10);

    RCLCPP_INFO(this->get_logger(), "PCA Node initialized");
    RCLCPP_INFO(this->get_logger(), "Diameter will lock after %d consistent samples with variance < %.3f m",
                diameter_samples_needed_, diameter_variance_threshold_);
}

// ======================
// Callbacks
// ======================
void PCA::yawCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    current_yaw_ = msg->data;
}

// ======================
//  ROI Filtering
// ======================
std::vector<Eigen::Vector2d> PCA::applyROIFilter(const std::vector<Eigen::Vector2d>& pts) {
    std::vector<Eigen::Vector2d> filtered;
    filtered.reserve(pts.size());
    
    for (const auto& p : pts) {
        // Basic ROI: Keep points within reasonable pipe region
        if (p.x() > -roi_max_x_ && p.x() < roi_max_x_ && std::abs(p.y()) < roi_max_y_) {
            // Additional diameter-based check if diameter is locked
            if (diameter_lock_) {
                // Add safety margin (20%) to account for measurement noise
                double max_allowed_distance = (locked_diameter_ / 2.0) * 1.2;
                
                // Remove points beyond the pipe radius
                if (std::abs(p.y()) <= max_allowed_distance) {
                    filtered.push_back(p);
                }
                // else: Point is too far from center, skip it
            } else {
                // Diameter not locked yet, just use basic ROI
                filtered.push_back(p);
            }
        }
    }
    
    return filtered;
}


void PCA::PCA_transform(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::vector<Eigen::Vector2d> left_side;
    std::vector<Eigen::Vector2d> right_side;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");

    // Split points into left and right based on y-coordinate
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
        double x = static_cast<double>(*iter_x);
        double y = static_cast<double>(*iter_y);
        (y > 0 ? left_side : right_side).emplace_back(x, y);
    }
    
    //  Apply ROI filtering
    left_side = applyROIFilter(left_side);
    right_side = applyROIFilter(right_side);
    
    //  Publish filtered point cloud for visualization
    publishFilteredCloud(left_side, right_side);

    // Perform PCA on both sides
    PCAResult left_result = computePCA(left_side);
    PCAResult right_result = computePCA(right_side);

    // Validate results based on confidence
    const double CONF_THRESH = 30.0;
    bool left_valid = left_result.confidence > CONF_THRESH;
    bool right_valid = right_result.confidence > CONF_THRESH;

    bool both_invalid = (!left_valid && !right_valid);
    bool insufficient_points = (left_side.size() < 3 && right_side.size() < 3);
    
    if (both_invalid || insufficient_points) {
        bool should_trigger_stop = false;
        
        if (both_invalid) {
            // Start timer on first invalid reading
            if (!invalid_walls_timer_started_) {
                invalid_walls_start_time_ = std::chrono::steady_clock::now();
                invalid_walls_timer_started_ = true;
            }
            
            // Check if timeout has elapsed
            auto elapsed = std::chrono::steady_clock::now() - invalid_walls_start_time_;
            if (elapsed >= INVALID_WALLS_TIMEOUT_) {
                should_trigger_stop = true;
            }
        } else if (insufficient_points) {
            // Immediately trigger for insufficient points
            should_trigger_stop = true;
        }
        
        if (should_trigger_stop) {
            RCLCPP_WARN(this->get_logger(), 
                        "PCA sides are unreliable, yaw not updated | conf L/R: (%.1f, %.1f)",
                        left_result.confidence/CONF_THRESH, right_result.confidence/CONF_THRESH);
            
            diameter_lock_ = false; // Unlock diameter if both sides become invalid
            
            if (!stop_sent_) {
                auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
                request->data = false;
                if (stop_client_->wait_for_service(std::chrono::milliseconds(100))) {
                    auto result_future = stop_client_->async_send_request(request); 
                    stop_sent_ = true;
                }
            }
        }
        return;
    }
    else {
        // Reset timer when walls become valid again
        invalid_walls_timer_started_ = false;
        
        if (stop_sent_) {
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = true;
            if (stop_client_->wait_for_service(std::chrono::milliseconds(100))) {
                auto result_future = stop_client_->async_send_request(request); 
                stop_sent_ = false;
            }
        }
    }
    // ========================================
    // DIAMETER EXTRACTION AND MANAGEMENT
    // ========================================
    double current_diameter = 0.0;

    if (left_valid && right_valid) {
        // Both walls visible - measure diameter
        current_diameter = std::abs(left_result.centroid.y() - right_result.centroid.y());
        
        if (!diameter_lock_) {
            // Collect samples for diameter locking
            diameter_samples_.push_back(current_diameter);
            
            if (diameter_samples_.size() >= static_cast<size_t>(diameter_samples_needed_)) {
                // Calculate mean and variance
                double sum = 0.0;
                for (double d : diameter_samples_) {
                    sum += d;
                }
                double mean = sum / diameter_samples_.size();
                
                double variance = 0.0;
                for (double d : diameter_samples_) {
                    variance += (d - mean) * (d - mean);
                }
                variance /= diameter_samples_.size();
                double std_dev = std::sqrt(variance);
                
                // Lock diameter if variance is low enough
                if (std_dev < diameter_variance_threshold_) {
                    locked_diameter_ = mean;
                    diameter_lock_ = true;
                    RCLCPP_INFO(this->get_logger(), 
                               "Diameter LOCKED at %.3f m (std_dev: %.4f m from %zu samples)",
                               locked_diameter_, std_dev, diameter_samples_.size());
                } else {
                    // Remove oldest sample and keep collecting
                    diameter_samples_.erase(diameter_samples_.begin());
                    RCLCPP_DEBUG(this->get_logger(),
                                "Diameter variance too high (%.4f m), collecting more samples...", std_dev);
                }
            }
        }
    }
    
    // Use locked diameter if available, otherwise current measurement
    double working_diameter = diameter_lock_ ? locked_diameter_ : current_diameter;
    
    // ========================================
    // CENTERING LOGIC WITH DIAMETER-BASED FALLBACK
    // ========================================
    double left_offset = 0.0, right_offset = 0.0;
    
    if (left_valid && right_valid) {
        // Both walls visible - use actual measurements
        left_offset = std::abs(left_result.centroid.y());
        right_offset = std::abs(right_result.centroid.y());
    }
    else if (left_valid && !right_valid && diameter_lock_) {
        // Only left wall visible, use diameter to estimate right
        left_offset = std::abs(left_result.centroid.y());
        right_offset = working_diameter - left_offset;
    }
    else if (!left_valid && right_valid && diameter_lock_) {
        // Only right wall visible, use diameter to estimate left
        right_offset = std::abs(right_result.centroid.y());
        left_offset = working_diameter - right_offset;
    }
    else {
        // Can't determine proper centering
        if (!diameter_lock_) {
            RCLCPP_WARN(this->get_logger(), 
                       "One wall invalid but diameter not yet locked - Centering is not possible - valid L/R: %d/%d",
                       left_valid, right_valid);
            left_offset = right_offset = 0;
            
        }
    }

    // ========================================
    // ANGLE CALCULATION WITH TEMPORAL SMOOTHING
    // ========================================
    double left_weight  = left_valid  ? left_result.confidence  : 0.0;
    double right_weight = right_valid ? right_result.confidence : 0.0;
    double total_weight = left_weight + right_weight;

    double raw_base_angle = 0.0;
    if (total_weight > 1e-6) {
        raw_base_angle =
            (left_result.angle * left_weight +
             right_result.angle * right_weight) / total_weight;
    }
    
    //  Apply temporal smoothing to base angle
    double base_angle;
    if (first_frame_) {
        base_angle = raw_base_angle;
        first_frame_ = false;
    } else {
        base_angle = temporal_alpha_ * previous_base_angle_ + (1.0 - temporal_alpha_) * raw_base_angle;
    }
    
    // Store for next iteration
    previous_base_angle_ = base_angle;

    // ========================================
    // YAW REFERENCE CALCULATION
    // ========================================
    double yaw_reference;
    double offset_difference = left_offset - right_offset;
    is_centered_ = std::abs(offset_difference) < centering_threshold_;
    
    if (is_centered_) {
        yaw_reference = base_angle + current_yaw_;
        RCLCPP_INFO(this->get_logger(), 
                   "CENTERED - ref yaw: %.3f rad | offsets L/R: (%.3f, %.3f) m | diameter: %.3f m %s | valid L/R: %d/%d | confidence L/R: (%.1f, %.1f)",
                   yaw_reference, left_offset, right_offset, working_diameter,
                   diameter_lock_ ? "[LOCKED]" : "[measuring]", left_valid, right_valid, left_result.confidence/CONF_THRESH, right_result.confidence/CONF_THRESH);
    } else {
        double gain;
        double abs_offset = std::abs(offset_difference);

        if (abs_offset > (2 * centering_threshold_)) {
            gain = 2 * centering_gain_;  
        } else if (abs_offset > (1.5 * centering_threshold_)) {
            gain = 1.5 * centering_gain_;   
        } else {
            gain = centering_gain_;  
        }
        double centering_correction = 
            std::clamp(gain * offset_difference,
                    -max_correction_, max_correction_);
        yaw_reference = base_angle + centering_correction + current_yaw_;
        RCLCPP_INFO(this->get_logger(),
                   "NOT CENTERED - offset diff: %.3f m | ref yaw: %.3f rad | offsets L/R: (%.3f, %.3f) m | diameter: %.3f m %s | valid L/R: %d/%d | steering %s by %.3f rad | confidence L/R: (%.1f, %.1f)",
                   offset_difference, yaw_reference, left_offset, right_offset, working_diameter,
                   diameter_lock_ ? "[LOCKED]" : "[measuring]", left_valid, right_valid, 
                   centering_correction < 0 ? "right" : "left", centering_correction, left_result.confidence/CONF_THRESH, right_result.confidence/CONF_THRESH);
    }

    yaw_reference = normalizeAngle(yaw_reference);
    if (client_->wait_for_service(std::chrono::milliseconds(100))) {
        client_->set_parameters({
            rclcpp::Parameter("config_reference_yaw", yaw_reference)
        });
    }
}

// ======================
// Publish Filtered Point Cloud
// ======================
void PCA::publishFilteredCloud(
    const std::vector<Eigen::Vector2d>& left_side,
    const std::vector<Eigen::Vector2d>& right_side) {
    
    // Create new point cloud message
    sensor_msgs::msg::PointCloud2 filtered_msg;
    filtered_msg.header.stamp = this->now();
    filtered_msg.header.frame_id = "lidar_link";
    filtered_msg.height = 1;
    filtered_msg.width = left_side.size() + right_side.size();
    filtered_msg.is_dense = false;
    filtered_msg.is_bigendian = false;
    
    // Define fields (x, y, z)
    sensor_msgs::PointCloud2Modifier modifier(filtered_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(filtered_msg.width);
    
    // Fill in the filtered points
    sensor_msgs::PointCloud2Iterator<float> iter_x(filtered_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(filtered_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(filtered_msg, "z");
    
    // Add left side points
    for (const auto& pt : left_side) {
        *iter_x = static_cast<float>(pt.x());
        *iter_y = static_cast<float>(pt.y());
        *iter_z = 0.0f;
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
    
    // Add right side points
    for (const auto& pt : right_side) {
        *iter_x = static_cast<float>(pt.x());
        *iter_y = static_cast<float>(pt.y());
        *iter_z = 0.0f;
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
    
    filtered_cloud_pub_->publish(filtered_msg);
}

// ======================
// PCA Computation
// ======================
PCAResult PCA::computePCA(const std::vector<Eigen::Vector2d>& pts) {
    PCAResult result;

    if (pts.empty()) {
        return result;
    }

    // Robust centroid
    double mean_x = 0.0;
    std::vector<double> ys;
    ys.reserve(pts.size());

    for (const auto& p : pts) {
        mean_x += p.x();
        ys.push_back(p.y());
    }
    mean_x /= pts.size();

    std::nth_element(
        ys.begin(),
        ys.begin() + ys.size() / 2,
        ys.end()
    );
    double median_y = ys[ys.size() / 2];

    Eigen::Vector2d centroid(mean_x, median_y);

    // Covariance
    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (const auto& p : pts) {
        Eigen::Vector2d q = p - centroid;
        cov += q * q.transpose();
    }
    cov /= pts.size();

    // PCA 
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
    Eigen::Vector2d eigenvalues = solver.eigenvalues();
    Eigen::Matrix2d eigenvectors = solver.eigenvectors();

    int max_idx = (eigenvalues(1) > eigenvalues(0)) ? 1 : 0;
    Eigen::Vector2d direction = eigenvectors.col(max_idx).normalized();
    if (direction.x() < 0) direction = -direction;

    result.centroid = centroid;
    result.direction = direction;
    result.eigenvalues = eigenvalues;
    result.confidence =
        eigenvalues(max_idx) / (eigenvalues(1 - max_idx) + 1e-6);
    result.angle = std::atan2(direction.y(), direction.x());

    return result;
}

double PCA::normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

// ======================
// main()
// ======================
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCA>());
    rclcpp::shutdown();
    return 0;
}