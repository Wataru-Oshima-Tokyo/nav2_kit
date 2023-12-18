#include <memory>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"

class ScanToCostmap : public rclcpp::Node
{
public:
    ScanToCostmap() : Node("scan_to_costmap"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        this->get_parameter("use_sim_time", use_sim_time_);
        if (1)
        {
            this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        }
        this->declare_parameter<std::string>("target_topic", "fake/scan_for_move");
        this->get_parameter("target_topic", target_topic);
        // Initialize publisher for costmap
        costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("global_costmap/costmap", rclcpp::SystemDefaultsQoS());
        costmap_raw_pub_ = this->create_publisher<nav2_msgs::msg::Costmap>("/global_costmap/costmap_raw", rclcpp::SystemDefaultsQoS());
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(target_topic, rclcpp::SystemDefaultsQoS(), std::bind(&ScanToCostmap::scanCallback, this, std::placeholders::_1));
        costmap_client_ = this->create_client<nav2_msgs::srv::GetCostmap>("/global_costmap/get_costmap");
        costmap_update_pub_ = this->create_publisher<map_msgs::msg::OccupancyGridUpdate>("/global_costmap/costmap_updates", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
        // costmap_raw_ = nav2_msgs::msg::Costmap>();
        prepareCostmap();
        // Initialize the occupancy grid
        // initializeGrid();
                // Initialize a timers

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Adjust the duration as needed
            std::bind(&ScanToCostmap::run, this));
    }

private:

    void run()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Your timer code here
        // This code will run periodically in a separate thread
        // Example: Print a message or perform some periodic checks        // Change the color to blue
        if (!header_set_)
            return;
        if (get_pose()){
            std::vector<int8_t> data;
            for (int i=0; i<200*200; i++){
                data.push_back(100);
            }
            // publishCostmapUpdate(current_pose.pose.position.x, current_pose.pose.position.y, 200, 200, data);
        }

        // for (size_t i = 0; i < scan->ranges.size(); ++i) {
        //     float range = scan->ranges[i];
        //     if (range >= scan->range_min && range <= scan->range_max) {
        //         float angle = scan->angle_min + i * scan->angle_increment;
        //         tf2::Transform scan_to_map;
        //         // Get transform from scan frame to map frame
        //         try {
        //             auto transform = tf_buffer_.lookupTransform("map", scan->header.frame_id, rclcpp::Time(0));
        //             tf2::fromMsg(transform.transform, scan_to_map);
        //         } catch (tf2::TransformException & ex) {
        //             RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
        //                         scan->header.frame_id.c_str(), "map", ex.what());
        //             continue;
        //         }
        //         // Transform the point
        //         tf2::Vector3 point(range * cos(angle), range * sin(angle), 0.0);
        //         tf2::Vector3 point_in_map = scan_to_map * point;
        //         // Inside the scanCallback, after transforming the point
        //         int mx, my;
        //         // if (costmap_->worldToMap(mx, my, point_in_map.x(), point_in_map.y())) {
        //         //     costmap_->setCost(mx, my, 100);
        //         // }
        //     }
        // }
        // prepareCostmap();

        // if (!costmap_raw_) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to get costmap_raw_");
        //     prepareCostmap();
        //     return;
        // }
       
        costmap_raw_pub_->publish(costmap_raw_);
    }


    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Clear previous data
        std::lock_guard<std::mutex> lock(mutex_);
        if (!header_set_) {
            header_set_ = true;
        }
        scan = msg;
    }

    void publishCostmapUpdate(int x, int y, int width, int height, const std::vector<int8_t>& data) {
        auto update_msg = std::make_unique<map_msgs::msg::OccupancyGridUpdate>();
        update_msg->header.stamp = this->now();
        update_msg->header.frame_id = "map";  // Ensure this matches your map frame
        update_msg->x = x;///costmap_raw_->metadata.resolution;
        update_msg->y = y;///costmap_raw_->metadata.resolution;
        update_msg->width = width;
        update_msg->height = height;
        update_msg->data = data;
        RCLCPP_INFO(this->get_logger(), "\033[1;34mUpdating occupancy grid\033[0m");
        RCLCPP_INFO(this->get_logger(), "The update x y  is : (%d, %d)", update_msg->x, update_msg->y);
        costmap_update_pub_->publish(std::move(update_msg));
    }


    bool get_pose() {
        try {
            // Replace 'now' with 'tf2::TimePointZero' if you want to get the latest available transform
            geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_.lookupTransform("map", "base_link", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0)); 
            current_pose.header.stamp = transformStamped.header.stamp;
            current_pose.header.frame_id = "map";
            current_pose.pose.position.x = transformStamped.transform.translation.x;
            current_pose.pose.position.y = transformStamped.transform.translation.y;
            current_pose.pose.position.z = transformStamped.transform.translation.z;
            current_pose.pose.orientation = transformStamped.transform.rotation;

            // Handle the pose as needed
            RCLCPP_INFO(this->get_logger(), "Pose: [%f, %f, %f]", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
            return true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform 'base_link' to 'map': %s", ex.what());
            return false;
        }
    }

    void prepareCostmap()
    {
        auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
        // Set any necessary fields in the request

        while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
            // if (!rclcpp::ok()) {
            //     RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            //     return;
            // }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
        }

        auto result_future = costmap_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
            return;
        }

        auto response = result_future.get();
//   response->map.header.stamp = current_time;
//   response->map.header.frame_id = global_frame_;
//   response->map.metadata.size_x = size_x;
//   response->map.metadata.size_y = size_y;
//   response->map.metadata.resolution = costmap_->getResolution();
//   response->map.metadata.layer = "master";
//   response->map.metadata.map_load_time = current_time;
//   response->map.metadata.update_time = current_time;
//   response->map.metadata.origin.position.x = costmap_->getOriginX();
//   response->map.metadata.origin.position.y = costmap_->getOriginY();
//   response->map.metadata.origin.position.z = 0.0;
//   response->map.metadata.origin.orientation = tf2::toMsg(quaternion);
//   response->map.data.resize(data_length);
//   response->map.data.assign(data, data + data_length);



        RCLCPP_INFO(this->get_logger(), "Preparing the cost map");
        double resolution = response->map.metadata.resolution;
        RCLCPP_INFO(this->get_logger(), "Resolution %f]", resolution);

        

        costmap_raw_.header.frame_id = response->map.header.frame_id;
        costmap_raw_.header.stamp = response->map.header.stamp;

        costmap_raw_.metadata.layer = "master";
        costmap_raw_.metadata.resolution = resolution;

        costmap_raw_.metadata.size_x = response->map.metadata.size_x;
        costmap_raw_.metadata.size_y = response->map.metadata.size_y;
        costmap_raw_.metadata.origin.position.x = response->map.metadata.origin.position.x; 
        costmap_raw_.metadata.origin.position.y = response->map.metadata.origin.position.y; 
        costmap_raw_.metadata.origin.position.z = 0.0;
        costmap_raw_.metadata.origin.orientation = response->map.metadata.origin.orientation;
        costmap_raw_.data = response->map.data;
        RCLCPP_INFO(this->get_logger(), "Size x: %d | Size y %d | Resolution %f]", costmap_raw_.metadata.size_x , costmap_raw_.metadata.size_y, costmap_raw_.metadata.resolution);
        costmap_raw_.data.resize(costmap_raw_.metadata.size_x * costmap_raw_.metadata.size_y);
        // unsigned char * data = costmap_->getCharMap();
        // RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
        // Handle the response
        for (unsigned int i = 0; i < costmap_raw_.data.size(); i++) {
            costmap_raw_.data[i] = 254;
        }
        RCLCPP_INFO(this->get_logger(), "Got costmap of size: %d x %d", response->map.metadata.size_x, response->map.metadata.size_y);
        // costmap_raw_pub_->publish(std::move(costmap_raw_));
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
    rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr costmap_raw_pub_;
    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
    rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_update_pub_;


    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    nav_msgs::msg::OccupancyGrid grid_;
    geometry_msgs::msg::TransformStamped transform_stamped;
    geometry_msgs::msg::PoseStamped current_pose;
    bool use_sim_time_;
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    std::mutex mutex_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string target_topic;
    std_msgs::msg::Header header_for_scan;
    bool header_set_ = false;
    bool  initial = true;
    std::vector<float, std::allocator<float>> ranges;
    float range_min, range_max, angle_min, angle_max;
    nav2_msgs::msg::Costmap costmap_raw_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToCostmap>());
    rclcpp::shutdown();
    return 0;
}
