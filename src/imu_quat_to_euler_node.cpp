//============================================================================
// Name        : imu_quat_to_euler_node.cpp
// Author      : Omar Elnahas
// Description : 
//============================================================================
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>


class ImuSubscriber : public rclcpp::Node
{
    public:
    ImuSubscriber():Node("imu_quat_to_euler_node")
    {
        // Imu data subscription
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&ImuSubscriber::imuCallback, this, std::placeholders::_1)
        );

        // Publishers
        yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("/gazebo/yaw", 10);
        roll_pub_ = this->create_publisher<std_msgs::msg::Float64>("/gazebo/roll", 10);
        pitch_pub_ = this->create_publisher<std_msgs::msg::Float64>("/gazebo/pitch", 10);
        
    }

private:

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) 
    {
        double roll, pitch, yaw;
        
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;

        quaternion_to_euler(qx, qy, qz, qw, roll, pitch, yaw);

        auto roll_msg = std_msgs::msg::Float64();
        roll_msg.data = roll;
        roll_pub_->publish(roll_msg);

        auto pitch_msg = std_msgs::msg::Float64();
        pitch_msg.data = pitch;
        pitch_pub_->publish(pitch_msg);

        auto yaw_msg = std_msgs::msg::Float64();
        yaw_msg.data = yaw;
        yaw_pub_->publish(yaw_msg);
    }

    void quaternion_to_euler(double qx, double qy, double qz, double qw, double &roll, double &pitch, double &yaw) {
        // Roll (x-axis rotation)
        double sinr_cosp = 2 * (qw * qx + qy * qz);
        double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        double sinp = 2 * (qw * qy - qz * qx);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = std::asin(sinp);

        // Yaw (z-axis rotation)
        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_pub_;    
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuSubscriber>());
    rclcpp::shutdown();
    return 0;
};
        
