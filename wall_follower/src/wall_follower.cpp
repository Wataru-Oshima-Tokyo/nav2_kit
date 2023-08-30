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
            "/scan_for_move", rclcpp::SensorDataQoS(), std::bind(&WallFollower::listener_callback, this, std::placeholders::_1));

    }

private:
    void listener_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto ranges = msg->ranges;
        auto twist = geometry_msgs::msg::Twist();
        // assumed the scan is already filtered here from -30 to 30 (-math.pi/6 < theta < math.pi/6)
        for (size_t i = 0; i < ranges.size(); i++)
        {
            if (ranges[i] < 1.0){
                twist.linear.x = 0;
                twist.angular.z = 0.5;
                RCLCPP_INFO(this->get_logger(), "Turning left");
                break;
            }
            else if (ranges[i] < 2.0){
                twist.linear.x = 0.5;
                twist.angular.z = 0.4;
                RCLCPP_INFO(this->get_logger(), "Turning left and a slightly foward");
                break;
            }
            else if (ranges[i] < 3.0){
                twist.linear.x = 1;
                twist.angular.z = 0.3;
                RCLCPP_INFO(this->get_logger(), "Turning a little left and slow donw ");
                break;
            }
            else if (ranges[i] < 4.0){
                twist.linear.x = 1.5;
                twist.angular.z = 0.2;
                RCLCPP_INFO(this->get_logger(), "Turning a little left and slow donw a little bit");
                break;
            }else{
                twist.linear.x = 3;
            }
        }

        if (twist.angular.z < 0.001){
            RCLCPP_INFO(this->get_logger(), "Moving forward");
        }
    
        // if (straight_ahead > 2.0)
        // {
        //     // Move forward
        //     twist.linear.x = 3;
        //     RCLCPP_INFO(this->get_logger(), "Moving forward");
        // }
        // else if (ahead_right > ahead_left)
        // {
        //     // Turn right
        //     twist.angular.z = -0.5;
        //     RCLCPP_INFO(this->get_logger(), "Turning right");
        // }
        // else
        // {
        //     // Turn left
        //     twist.angular.z = 0.5;
        //     RCLCPP_INFO(this->get_logger(), "Turning left");
        // }

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
