#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <random>  // Include the random library for generating noise

class IMUPublisherNode : public rclcpp::Node
{
public:
    IMUPublisherNode() : Node("imu_publisher")
    {
        // Create a publisher for IMU data
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("fake_imu", 10);

        // Set up a timer to publish fake IMU data at a rate of 200 Hz (5 ms period)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),  // 5 ms period (200 Hz)
            std::bind(&IMUPublisherNode::publishFakeIMUData, this));

        // Initialize random number generator for noise
        std::random_device rd;
        random_generator_ = std::mt19937(rd());
        linear_acc_distribution_ = std::normal_distribution<>(0.0, 0.01); // Mean = 0, Standard deviation = 0.01
        angular_vel_distribution_ = std::normal_distribution<>(0.0, 0.01);
    }

private:
    void publishFakeIMUData()
    {
        // Create an IMU message
        auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

        // Add random noise to linear acceleration and angular velocity
        imu_msg->linear_acceleration.x = 0.0 + linear_acc_distribution_(random_generator_);
        imu_msg->linear_acceleration.y = 0.0 + linear_acc_distribution_(random_generator_);
        imu_msg->linear_acceleration.z = 9.81 + linear_acc_distribution_(random_generator_);
        imu_msg->angular_velocity.x = 0.0 + angular_vel_distribution_(random_generator_);
        imu_msg->angular_velocity.y = 0.0 + angular_vel_distribution_(random_generator_);
        imu_msg->angular_velocity.z = 0.0 + angular_vel_distribution_(random_generator_);

        // Set the timestamp
        imu_msg->header.stamp = this->get_clock()->now();

        // Set the frame ID
        imu_msg->header.frame_id = "imu_frame";  // Change "imu_frame" to your desired frame ID

        // Publish the IMU message
        imu_publisher_->publish(std::move(imu_msg));
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mt19937 random_generator_;
    std::normal_distribution<> linear_acc_distribution_;
    std::normal_distribution<> angular_vel_distribution_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
