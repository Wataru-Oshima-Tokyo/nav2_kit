#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>


class FakeDynamicFrameBroadcaster : public rclcpp::Node
{
public:
    FakeDynamicFrameBroadcaster() : Node("fake_dynamic_frame_broadcaster")
    {

        this->get_parameter("use_sim_time", use_sim_time_);
        if (use_sim_time_)
        {
            this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        }
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&FakeDynamicFrameBroadcaster::broadcast_fake_frame, this));
        this->declare_parameter<std::string>("parent_link", "map");
        this->get_parameter("parent_link", parent_link);
        this->declare_parameter<std::string>("child_link", "odom");
        this->get_parameter("child_link", child_link);
    
    }

private:
    void broadcast_fake_frame()
    {
        auto t = std::make_unique<geometry_msgs::msg::TransformStamped>();

        t->header.stamp = this->now();
        t->header.frame_id = parent_link;
        t->child_frame_id = child_link;
        t->transform.translation.x = 0.0;
        t->transform.translation.y = 0.0;
        t->transform.translation.z = 0.0;
        t->transform.rotation.x = 0.0;
        t->transform.rotation.y = 0.0;
        t->transform.rotation.z = 0.0;
        t->transform.rotation.w = 1.0;

        broadcaster_->sendTransform(*t);
    }


    bool use_sim_time_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    std::string parent_link;
    std::string child_link;
    std::mutex mutex_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FakeDynamicFrameBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
