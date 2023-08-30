#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class WallFollower : public rclcpp::Node
{
public:
    WallFollower()
        : Node("wall_follower")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&WallFollower::listener_callback, this, std::placeholders::_1));

    }

private:
    void listener_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto ranges = msg->ranges;
        auto twist = geometry_msgs::msg::Twist();
        // assumed the scan is already filtered here from -30 to 30 (-math.pi/6 < theta < math.pi/6)
        bool published = false;
        for (size_t i = 0; i < ranges.size(); i++)
        {   
            if (ranges[i] < 0.3){
                twist.linear.x = 0;
                RCLCPP_WARN(this->get_logger(), "Make it stop");
                published = true;
                break;
            }
            else if (ranges[i] < 1.0){
                twist.linear.x = 0.05;

                RCLCPP_WARN(this->get_logger(), "Slow down");
                published = true;
                break;
            }
            else if (ranges[i] < 2.0){
                twist.linear.x = 0.1;
                RCLCPP_WARN(this->get_logger(), "Slow down a little bit");
                published = true;
                break;
            }
        }
        if (published)
            publisher_->publish(twist);
    }



    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollower>());
    rclcpp::shutdown();
    return 0;
}
