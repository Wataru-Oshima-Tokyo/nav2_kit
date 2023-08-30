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
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&WallFollower::scan_callback, this, std::placeholders::_1));
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/nav_cmd_vel", rclcpp::SensorDataQoS(), std::bind(&WallFollower::cmd_vel_callback, this, std::placeholders::_1));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto ranges = msg->ranges;
        // assumed the scan is already filtered here from -30 to 30 (-math.pi/6 < theta < math.pi/6)
        bool scan_checker = false;

        for (int i = index_min; i < index_max; i++)
        {   
            if (ranges[i] < 0.5){
                twist.linear.x = 0;
                RCLCPP_WARN(this->get_logger(), "Do not go forward!!");
                scan_checker = true;
                break;
            }
            else if (ranges[i] < 1.0){
                twist.linear.x = 0.1;

                RCLCPP_WARN(this->get_logger(), "Slow down");
                scan_checker = true;
                break;
            }
            else if (ranges[i] < 2.0){
                twist.linear.x = 0.2;
                RCLCPP_WARN(this->get_logger(), "Slow down a little bit");
                scan_checker = true;
                break;
            }
        }
        if (scan_checker) {
            warning = true; 
        } else {
            warning = false;
        }
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        twist.angular.z = msg->angular.z; 
        if(!warning || msg->linear.x < twist.linear.x)
            twist.linear.x = msg->linear.x; 
        publisher_->publish(twist);
    }



    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    geometry_msgs::msg::Twist twist;
    int angle_min_deg = -30;  // minimum angle in degrees
    int angle_max_deg = 30;   // maximum angle in degrees
    double angle_increment = 0.0087;
    // calculate the indices in the ranges list that correspond to the angles
    int index_min = (angle_min_deg - (-180)) / (angle_increment * 180 / M_PI);
    int index_max = (angle_max_deg - (-180)) / (angle_increment * 180 / M_PI);
    bool warning = false;
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollower>());
    rclcpp::shutdown();
    return 0;
}
