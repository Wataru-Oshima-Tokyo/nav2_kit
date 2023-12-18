#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp" // Include odometry message
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "std_srvs/srv/set_bool.hpp"

class FakeScan : public rclcpp::Node
{
public:
    FakeScan()
        : Node("fake_scan"), tfBuffer_(this->get_clock()), tfListener_(tfBuffer_), last_time_pitch_exceeded(this->get_clock()->now())
    {


        // Get the parameter
        this->declare_parameter<std::string>("target_topic", "scan");
        this->get_parameter("target_topic", target_topic);
        std::string fake_topic = "/fake/" + target_topic;
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(fake_topic, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            target_topic, 
            rclcpp::QoS(10).best_effort(),  // Set history depth and QoS to Best Effort
            std::bind(&FakeScan::scan_callback, this, std::placeholders::_1));
        // Subscribe to /dlio/odom_node/odom to catch the pitch
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/dlio/odom_node/odom", 
            rclcpp::QoS(10).best_effort(),  // Set history depth and QoS to Best Effort
            std::bind(&FakeScan::odom_callback, this, std::placeholders::_1));
        std::string service_name = "/toggle_scanning/" + target_topic;
        start_scanning_service_ = this->create_service<std_srvs::srv::SetBool>(
            service_name,
            std::bind(&FakeScan::handle_scan_toggle, this, std::placeholders::_1, std::placeholders::_2));
        fake_frame_id_ = "fake_laser";
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&FakeScan::rebroadcast_tf, this));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // RCLCPP_WARN(this->get_logger(), "Received scan topic");
        if (start_scanning){
            last_scan_ = *msg; // Store the incoming scan data
            last_time_pitch_exceeded = this->get_clock()->now();
        }else{
            RCLCPP_WARN(this->get_logger(), "not started scanning yet...");
            last_scan_ = *msg; 
            std::fill(last_scan_.ranges.begin(), last_scan_.ranges.end(), 0);
        }

    }
    // scan_callback to handle odometry data
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract the pitch from the odometry message
        double pitch = msg->pose.pose.orientation.x;
        // Log the pitch for debugging
        
        
        // Check if the pitch is more than 0.2
        if (!start_scanning){
            if (fabs(pitch) > 0.15) {
                // If the pitch is more than 0.2 for 2 seconds, set start_scanning to true
                if (this->get_clock()->now() - last_time_pitch_exceeded > rclcpp::Duration(2, 0)) {
                    RCLCPP_INFO(this->get_logger(), "\033[1;32mNow it is confirmed that the robot is climbing\033[0m");
                    start_scanning = true;
                }
            } else {
                // Reset the time when the pitch exceeded 0.2
                // RCLCPP_INFO(this->get_logger(), "Received pitch: %f", pitch);
                last_time_pitch_exceeded = this->get_clock()->now();
            }
        }
    }
        // Service scan_callback to toggle collision detection
    void handle_scan_toggle(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        start_scanning = request->data;
        response->success = true;
        response->message = "Start scannning " + std::string(start_scanning ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // Odometry subscription
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
    rclcpp::Time last_time_pitch_exceeded;  // Add this line

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
