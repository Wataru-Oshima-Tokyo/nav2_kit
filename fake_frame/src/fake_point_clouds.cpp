#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class FakeFrame : public rclcpp::Node
{
public:
    FakeFrame()
        : Node("fake_frame")
    {


        // Get the parameter
        this->declare_parameter<std::string>("fake_frame_id", "fake_velodyne_link");
        this->get_parameter("fake_frame_id", fake_frame_id_);
        this->declare_parameter<std::string>("target_topic", "points_raw");
        this->get_parameter("target_topic", target_topic);
        std::string fake_topic = "/fake/" + target_topic;
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(fake_topic, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            target_topic, 10,
            std::bind(&FakeFrame::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        msg->header.frame_id = fake_frame_id_;
        // msg->header.stamp = this->now();  // Update the timestamp to current time
        publisher_->publish(*msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::string fake_frame_id_;
    std::string target_topic;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<FakeFrame>());
    rclcpp::shutdown();
    return 0;
}
