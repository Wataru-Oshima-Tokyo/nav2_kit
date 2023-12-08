#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <random>
#include "std_srvs/srv/set_bool.hpp"

class SafetyRelay : public rclcpp::Node
{
public:
    SafetyRelay()
        : Node("saftey_relay")
    {
        // 1. Declare the parameter
        this->declare_parameter<std::string>("cmd_vel_topic", "/diff_cont/cmd_vel_unstamped");
        this->declare_parameter<int>("linear_coefficient", 3);
        this->declare_parameter<int>("angular_coefficient", 5);
        
        // 2. Retrieve the parameter value
        std::string cmd_vel_topic;
        this->get_parameter("cmd_vel_topic", cmd_vel_topic);
        this->get_parameter("linear_coefficient", linear_coefficient);
        this->get_parameter("angular_coefficient", angular_coefficient);
        collision_detection_service_ = this->create_service<std_srvs::srv::SetBool>(
            "toggle_collision_detection",
            std::bind(&SafetyRelay::handle_collision_detection_toggle, this, std::placeholders::_1, std::placeholders::_2));
        // 3. Initialize the publisher using the retrieved topic name
        cmd_vel_publisher_  = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
        

        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fake/scan_for_move", rclcpp::SensorDataQoS(), std::bind(&SafetyRelay::scan_callback, this, std::placeholders::_1));
        
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::SensorDataQoS(), std::bind(&SafetyRelay::cmd_vel_callback, this, std::placeholders::_1));
        
        this->get_parameter("use_sim_time", use_sim_time_);
        if (use_sim_time_)
        {
            this->set_parameter(rclcpp::Parameter("use_sim_time", true));
            max_vel = 1.5 * linear_coefficient;
        }else{
            max_vel = 0.8 * linear_coefficient;
        }
        
    }

private:

    // Service callback to toggle collision detection
    void handle_collision_detection_toggle(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        collision_detection_enabled_ = request->data;
        warning = request->data;
        response->success = true;
        response->message = "Collision detection " + std::string(collision_detection_enabled_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto ranges = msg->ranges;
        // assumed the scan is already filtered here from -30 to 30 (-math.pi/6 < theta < math.pi/6)
        int obstacle_count[4] = {0,0,0,0};
        // obstacle_count[0] : counter for "Stop"
        // obstacle_count[1] : counter for "Slow down a little bit"
        // obstacle_count[2] : counter for "Just be careful"
        // obstacle_count[3] : counter for "obstacle detection speed"
       


        for (int i = index_min; i < index_max; i++)
        {   
            if (ranges[i] < 1.0 && i >= index_min_slowdown && i <= index_max_slowdown){
                obstacle_count[0]++;
                if(obstacle_count[0] > count_threshold)
                    break;
            }
            else if (ranges[i] < 2.0 && i >= index_min_slowdown && i <= index_max_slowdown){
                obstacle_count[1]++;
                
            }
            else if (ranges[i] < 3.0 && i >= index_min_slowdown && i <= index_max_slowdown){
                obstacle_count[2]++;
            }
            else if (ranges[i] < 4.0 && i >= index_min_slowdown && i <= index_max_slowdown){
                obstacle_count[3]++;
            }
        }
        if (collision_detection_enabled_){
            if(obstacle_count[0] > count_threshold){
                twist.linear.x = 0.0;
                RCLCPP_WARN(this->get_logger(), "Stop");
                warning = true;
                
            }
            else if(obstacle_count[1] > count_threshold){
                twist.linear.x = 0.2;
                RCLCPP_WARN(this->get_logger(), "Slow down a little bit");
                warning = true;
            }
            else if(obstacle_count[2] > count_threshold){
                twist.linear.x = 0.3;
                RCLCPP_WARN(this->get_logger(), "Just be careful");
                warning = true;
            }
            else if(obstacle_count[3] > count_threshold){
                twist.linear.x = 0.4;
                RCLCPP_WARN(this->get_logger(), "obstacle detection speed");
                warning = true;  
            }
            else{
                warning = false;
            }
        }


    }

    void clamp_velocity_to_max(double &velocity, float max_value, double &ang_vel) {
        if (velocity > max_value) {
            velocity = max_value;
        }


    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {   
        RCLCPP_INFO(this->get_logger(), "\033[34mReceived a command\033[0m");
        twist.angular.z = msg->angular.z;
        if (!collision_detection_enabled_){
            accel = 1.0;
         if( fabs(msg->linear.x) <0.11 || msg->linear.x < 0){
                twist.linear.x = msg->linear.x; 
            }else{
              twist.linear.x = 0.11;
            }
        }else{
            if(!warning || fabs(msg->linear.x) < fabs(twist.linear.x) ||  msg->linear.x < 0){
                twist.linear.x = msg->linear.x; 
                accel += 0.001;
            }else if (warning){
                accel = 1.0;
            }
        }



        if (use_sim_time_){
            if (fabs(msg->angular.z) < 0.2 && fabs(msg->linear.x) <= 0.01){
                if (!(fabs(msg->angular.z) < 0.1001 )){
                    twist.angular.z *= angular_coefficient*3;
                }
                else{
                    twist.linear.x = 0;
                    twist.angular.z = 0;
                }
            }else if (!warning && fabs(msg->linear.x) >= 0.1){
                twist.linear.x *= linear_coefficient * accel;
                twist.angular.z  *= angular_coefficient;
            }else{
                twist.linear.x *= (linear_coefficient/2) < 1 ? 1 : linear_coefficient/2;
                twist.angular.z *= angular_coefficient; //(angular_coefficient/2) < 1 ? 1 : angular_coefficient/2;
            }
        }else{
            if (fabs(twist.angular.z) < 0.2 && fabs(twist.linear.x) <= 0.01){
                if (!(fabs(twist.angular.z) < 0.1001 ))
                    twist.angular.z *= 2;
                else
                    twist.angular.z = 0;
            }
        }

        clamp_velocity_to_max(twist.linear.x, max_vel, twist.angular.z);
        cmd_vel_publisher_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr collision_detection_service_;
    geometry_msgs::msg::Twist twist;
    bool collision_detection_enabled_ = true;  // Initial state of collision detection
    int angle_min_deg = -30;  // minimum angle in degrees
    int angle_max_deg = 30;   // maximum angle in degrees
    double angle_increment = 0.0087;
    int angle_range = 60;
    int linear_coefficient, angular_coefficient;
    int angle_min_slowdown_deg = angle_min_deg/3;  // minimum angle for slowdown in degrees
    int angle_max_slowdown_deg = angle_max_deg/3;   // maximum angle for slowdown in degrees
    // calculate the indices in the ranges list that correspond to the slowdown angles
    int index_min_slowdown = (angle_min_slowdown_deg  + angle_range) / (angle_increment * 180 / M_PI);
    int index_max_slowdown = (angle_max_slowdown_deg + angle_range) / (angle_increment * 180 / M_PI);

    // calculate the indices in the ranges list that correspond to the angles
    int index_min = (angle_min_deg  + angle_range) / (angle_increment * 180 / M_PI);
    int index_max = (angle_max_deg + angle_range) / (angle_increment * 180 / M_PI); 
    bool warning = false;
    bool use_sim_time_ = false;
    const  int count_threshold = 3;
    float accel = 1.0;
    float max_vel;
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyRelay>());
    rclcpp::shutdown();
    return 0;
}
