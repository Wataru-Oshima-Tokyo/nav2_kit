#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class CmdVelToOdomNode : public rclcpp::Node
{
public:
    CmdVelToOdomNode() : Node("cmd_vel_to_odom"), tf_broadcaster_(this)
    {
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&CmdVelToOdomNode::cmdVelCallback, this, std::placeholders::_1));
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        x_ = 0.0;
        y_ = 0.0;
        th_ = 0.0;
        last_time_ = this->now();
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Compute odometry
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        double dx = msg->linear.x * dt;
        double dy = msg->linear.y * dt;
        double dth = msg->angular.z * dt;

        x_ += dx * cos(th_) - dy * sin(th_);
        y_ += dx * sin(th_) + dy * cos(th_);
        th_ += dth;

        // Create and publish odometry message
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, th_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x = msg->linear.x;
        odom.twist.twist.linear.y = msg->linear.y;
        odom.twist.twist.angular.z = msg->angular.z;
        odom_publisher_->publish(odom);

        // Publish TF transformation
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = current_time;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        tf_broadcaster_.sendTransform(transform);
    }


    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    double x_, y_, th_;
    rclcpp::Time last_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToOdomNode>());
    rclcpp::shutdown();
    return 0;
}





