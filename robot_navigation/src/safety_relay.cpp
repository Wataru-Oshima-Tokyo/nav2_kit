#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <random>
class SafetyRelay : public rclcpp::Node
{
public:
    SafetyRelay()
        : Node("saftey_relay")
    {
        cmd_vel_publisher_  = this->create_publisher<geometry_msgs::msg::Twist>("/diff_cont/cmd_vel_unstamped", 10);

        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_for_move", rclcpp::SensorDataQoS(), std::bind(&SafetyRelay::scan_callback, this, std::placeholders::_1));
        
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::SensorDataQoS(), std::bind(&SafetyRelay::cmd_vel_callback, this, std::placeholders::_1));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto ranges = msg->ranges;
        // assumed the scan is already filtered here from -30 to 30 (-math.pi/6 < theta < math.pi/6)
        bool scan_checker = false;
        
        for (int i = index_min; i < index_max; i++)
        {   
            if ((ranges[i] < 1.0 && i >= index_min_slowdown && i <= index_max_slowdown) || 
                (backup && ranges[i] < 3.0 )){
                twist.linear.x = - 0.2;
                RCLCPP_WARN(this->get_logger(), "Back up ");
                scan_checker = true;
                backup = true;
                break;
            }
            else if (ranges[i] < 1.0){
                twist.linear.x = 0.0;
                RCLCPP_WARN(this->get_logger(), "Stop");
                scan_checker = true;
                backup = true;
                break;
            }
            else if (ranges[i] < 2.0 && i >= index_min_slowdown && i <= index_max_slowdown){
                twist.linear.x = 0.2;
                RCLCPP_WARN(this->get_logger(), "Slow down a little bit");
                scan_checker = true;
                break;
            }else if (ranges[i] < 3.0 && i >= index_min_slowdown && i <= index_max_slowdown){
                twist.linear.x = 0.3;
                RCLCPP_WARN(this->get_logger(), "Jutst be careful");
                scan_checker = true;
                backup = false;
                break;
            } else if (ranges[i] < 4.0 && i >= index_min_slowdown && i <= index_max_slowdown){
                twist.linear.x = 0.4;
                RCLCPP_WARN(this->get_logger(), "obstacle detection speed");
                scan_checker = true;
                backup = false;
                break;
            }
        }
        if (scan_checker) {
            warning = true; 
        } else {
            warning = false;
            backup = false;
        }
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {   

        twist.angular.z = msg->angular.z;
        if(!warning || fabs(msg->linear.x) < fabs(twist.linear.x))
            twist.linear.x = msg->linear.x; 



        if (fabs(twist.angular.z) < 0.2 && fabs(twist.linear.x) <= 0.01)
            if (!(fabs(twist.angular.z) < 0.1 ))
                twist.angular.z *= 3;
            else
                twist.angular.z = 0;
        cmd_vel_publisher_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    geometry_msgs::msg::Twist twist;
    int angle_min_deg = -30;  // minimum angle in degrees
    int angle_max_deg = 30;   // maximum angle in degrees
    double angle_increment = 0.0087;
    int angle_range = 60;
    int angle_min_slowdown_deg = angle_min_deg/3;  // minimum angle for slowdown in degrees
    int angle_max_slowdown_deg = angle_max_deg/3;   // maximum angle for slowdown in degrees
    // calculate the indices in the ranges list that correspond to the slowdown angles
    int index_min_slowdown = (angle_min_slowdown_deg  + angle_range) / (angle_increment * 180 / M_PI);
    int index_max_slowdown = (angle_max_slowdown_deg + angle_range) / (angle_increment * 180 / M_PI);

    // calculate the indices in the ranges list that correspond to the angles
    int index_min = (angle_min_deg  + angle_range) / (angle_increment * 180 / M_PI);
    int index_max = (angle_max_deg + angle_range) / (angle_increment * 180 / M_PI);
    bool backup = false; 
    bool warning = false;
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyRelay>());
    rclcpp::shutdown();
    return 0;
}
