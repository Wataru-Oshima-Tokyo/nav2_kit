#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "std_srvs/srv/set_bool.hpp"

class FakeScan : public rclcpp::Node
{
public:
    FakeScan()
        : Node("fake_scan"), tfBuffer_(this->get_clock()), tfListener_(tfBuffer_)
    {


        // Get the parameter
        this->declare_parameter<std::string>("target_topic", "scan");
        this->get_parameter("target_topic", target_topic);
        std::string fake_topic = "/fake/" + target_topic;
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(fake_topic, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            target_topic, 
            rclcpp::QoS(10).best_effort(),  // Set history depth and QoS to Best Effort
            std::bind(&FakeScan::callback, this, std::placeholders::_1));
        start_scanning_service_ = this->create_service<std_srvs::srv::SetBool>(
            "toggle_scanning",
            std::bind(&FakeScan::handle_scan_toggle, this, std::placeholders::_1, std::placeholders::_2));
        fake_frame_id_ = "fake_laser";
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&FakeScan::rebroadcast_tf, this));
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // RCLCPP_WARN(this->get_logger(), "Received scan topic");
        if (start_scanning)
            last_scan_ = *msg; // Store the incoming scan data
        else{
            RCLCPP_WARN(this->get_logger(), "not started scanning yet...");
            last_scan_ = *msg; 
            std::fill(last_scan_.ranges.begin(), last_scan_.ranges.end(), 0);
        }

    }
        // Service callback to toggle collision detection
    void handle_scan_toggle(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        start_scanning = request->data;
        response->success = true;
        response->message = "Start scannning " + std::string(start_scanning ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_scanning_service_;
    std::string fake_frame_id_;
    std::string target_topic;

    void rebroadcast_tf()
    {
        try
        {
            auto trans = tfBuffer_.lookupTransform("map", "base_link", rclcpp::Time());
            rclcpp::Time trans_time = trans.header.stamp;

            // Add a small delay, e.g., 100 milliseconds
            rclcpp::Duration delay(0, 100000000); // 100 milliseconds
            rclcpp::Time delayed_time = trans_time + delay;
            // Update the timestamp of the last scan
            last_scan_.header.frame_id = fake_frame_id_;
            last_scan_.header.stamp = trans_time;

            // Republish the scan with the updated time
            publisher_->publish(last_scan_);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    bool start_scanning = false;  // Initial state of collision detection
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    sensor_msgs::msg::LaserScan last_scan_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<FakeScan>());
    rclcpp::shutdown();
    return 0;
}
