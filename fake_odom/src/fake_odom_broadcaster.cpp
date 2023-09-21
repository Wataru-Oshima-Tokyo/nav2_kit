#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class MyTFBroadcaster : public rclcpp::Node
{
public:
    MyTFBroadcaster() : Node("my_tf2_broadcaster"), tfBuffer_(this->get_clock()), tfListener_(tfBuffer_)
    {
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&MyTFBroadcaster::rebroadcast_tf, this));
    }

private:
    void rebroadcast_tf()
    {
        try
        {
            auto trans = tfBuffer_.lookupTransform("map", "velodyne", rclcpp::Time());
            trans.child_frame_id = "base_link";
            broadcaster_->sendTransform(trans);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyTFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
