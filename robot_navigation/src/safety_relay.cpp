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
        safety_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_for_safety", rclcpp::SensorDataQoS(), std::bind(&SafetyRelay::safety_callback, this, std::placeholders::_1));        
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::SensorDataQoS(), std::bind(&SafetyRelay::cmd_vel_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/dlio/odom_node/odom", 
            rclcpp::QoS(10).best_effort(),  // Set history depth and QoS to Best Effort
            std::bind(&SafetyRelay::odom_callback, this, std::placeholders::_1));
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

    double calculateLidarRange(double &lidar_height, double &distance_ahead, double &slope_angle) {
        // Calculate the height difference
        double height_diff = lidar_height * sin(slope_angle);

        // Calculate the actual distance traveled
        double actual_distance = sqrt(pow(distance_ahead, 2) + pow(height_diff, 2));

        // Calculate the distance seen at the top
        double distance_top = distance_ahead + height_diff;

        return distance_top;
    }
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

    void safety_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        static int safety_threshold = 10;
        auto ranges = msg->ranges;
                // Calculate index_min based on message parameters
        int safety_stop_count = 0;
        static double safety_angle_increment = msg->angle_increment;

        // calculate the indices in the ranges list that correspond to the angles
        static int safety_index_max = std::round(msg->angle_max / safety_angle_increment); 
        // Reduce the range to 1/3 of the original range for more focused checking
        static double safety_angle_min_slowdown_rad =  msg->angle_min/3;  // minimum angle for slowdown in radians
        static double safety_angle_max_slowdown_rad = msg->angle_max/3;   // maximum angle for slowdown in radians
        // Set the input values
        static double lidar_height = 0.8;  // meters
        static double distance_ahead = 1.0;  // meters
        static double offset_ = 0.2;
        // double abs_pitch = fabs(pitch);
        // Calculate the distance seen at the top
        // double check_distance = calculateLidarRange(lidar_height, distance_ahead, abs_pitch);
        if (fabs(pitch) <0.01){
            for (int i = 0; i < safety_index_max; i++)
            {   if (ranges[i] >= INFINITY)
                    continue;;
                if (ranges[i] > (1.5) ){
                    safety_stop_count++;
                    if(safety_stop_count > safety_threshold)
                        break;
                }
            }
        }

        if(safety_stop_count > safety_threshold){
            RCLCPP_WARN(this->get_logger(), "\033[1;31mSafety_stop\033[0m");
            safety_stop = true;
        }else{
            safety_stop = false;
        }

    }

        // scan_callback to handle odometry data
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract the pitch from the odometry message
         pitch = msg->pose.pose.orientation.x;

    }


    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        
        auto ranges = msg->ranges;
                // Calculate index_min based on message parameters
        int obstacle_count[4] = {0,0,0,0};
        double scan_angle_increment = msg->angle_increment;

        // calculate the indices in the ranges list that correspond to the angles
        int scan_index_min = 0;
        int scan_index_max = std::round(msg->angle_max / scan_angle_increment); 
        // Reduce the range to 1/3 of the original range for more focused checking
        double scan_angle_min_slowdown_rad =  msg->angle_min/3;  // minimum angle for slowdown in radians
        double scan_angle_max_slowdown_rad = msg->angle_max/3;   // maximum angle for slowdown in radians
        // calculate the indices in the ranges list that correspond to the reduced range
        int scan_index_min_slowdown = std::round((scan_angle_min_slowdown_rad  + msg->angle_max) / scan_angle_increment);
        int scan_index_max_slowdown = std::round((scan_angle_max_slowdown_rad + msg->angle_max) / scan_angle_increment);

        for (int i = scan_index_min_slowdown; i < scan_index_max_slowdown; i++)
        {   
            RCLCPP_INFO(this->get_logger(), "\033[ranges[%d] %lf\033[0m", i, ranges[i]);
            if (ranges[i] < 1.0){
                obstacle_count[0]++;
                if(obstacle_count[0] > count_threshold)
                    break;
            }
            else if (ranges[i] < 2.0){
                obstacle_count[1]++;
                
            }
            else if (ranges[i] < 3.0){
                obstacle_count[2]++;
            }
            else if (ranges[i] < 4.0){
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
        
        twist.angular.z = msg->angular.z;
        if (!collision_detection_enabled_){
            accel = 1.0;
            if( fabs(msg->linear.x) <0.15 || msg->linear.x < 0){
                twist.linear.x = msg->linear.x; 
            }else{
                twist.linear.x = 0.15;
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
                twist.linear.x *= linear_coefficient; //(linear_coefficient/2) < 1 ? 1 : linear_coefficient/2;
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
        if (safety_stop)
            twist.linear.x = 0.0;
        cmd_vel_publisher_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr safety_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr collision_detection_service_;
    geometry_msgs::msg::Twist twist;
    bool collision_detection_enabled_ = true;  // Initial state of collision detection
    int linear_coefficient, angular_coefficient;
    bool safety_stop = false;
    bool warning = false;
    bool use_sim_time_ = false;
    const int count_threshold = 3;
    float accel = 1.0;
    float max_vel;
    double pitch = 0.0;
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyRelay>());
    rclcpp::shutdown();
    return 0;
}
