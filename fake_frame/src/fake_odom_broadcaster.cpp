#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>

class MyTFBroadcaster : public rclcpp::Node
{
public:
    MyTFBroadcaster() : Node("my_tf2_broadcaster"), tfBuffer_(this->get_clock()), tfListener_(tfBuffer_)
    {
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&MyTFBroadcaster::rebroadcast_tf, this));
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/imu", 10, std::bind(&MyTFBroadcaster::odometry_callback, this, std::placeholders::_1));

        last_z_ = 0.0;
        position_z = 0.0;
        initialized_ = false;
    }

private:
    void rebroadcast_tf()
    {
        try
        {
            auto trans = tfBuffer_.lookupTransform("odom", "velodyne", rclcpp::Time());
            trans.child_frame_id = "base_link";
            if(initialized_){
                std::lock_guard<std::mutex> lock(mutex_);   
                trans.transform.translation.z = position_z;
            }




            broadcaster_->sendTransform(trans);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);   
        if (!initialized_) {
            last_z_ = msg->pose.pose.position.z;
            initialized_ = true;
            return;
        }

        double current_z = msg->pose.pose.position.z;
        double delta_z = current_z - last_z_;

        RCLCPP_INFO(this->get_logger(), "Delta Z: %f", delta_z);
        position_z +=delta_z;
        last_z_ = current_z;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    geometry_msgs::msg::PoseStamped current_pose;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    double last_z_;
    double position_z;
    bool initialized_;
    std::mutex mutex_;
    // bool  get_pose() {
    //     try {
    //         // Replace 'now' with 'tf2::TimePointZero' if you want to get the latest available transform
    //         geometry_msgs::msg::TransformStamped transformStamped = tfBuffer_.lookupTransform("map", "base_link", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0)); 
    //         current_pose.header.stamp = transformStamped.header.stamp;
    //         current_pose.header.frame_id = "map";
    //         current_pose.pose.position.x = transformStamped.transform.translation.x;
    //         current_pose.pose.position.y = transformStamped.transform.translation.y;
    //         current_pose.pose.position.z = transformStamped.transform.translation.z;
    //         current_pose.pose.orientation = transformStamped.transform.rotation;

    //         // Handle the pose as needed
    //         RCLCPP_INFO(this->get_logger(), "Pose: [%f, %f, %f]", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    //         return true;
    //     } catch (tf2::TransformException &ex) {
    //         RCLCPP_WARN(this->get_logger(), "Could not transform 'base_link' to 'map': %s", ex.what());
    //         return false;
    //     }
    // }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyTFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
