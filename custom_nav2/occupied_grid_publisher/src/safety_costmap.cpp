#include <memory>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include <tf2/LinearMath/Quaternion.h>

#include <functional>

namespace std {
    template <>
    struct hash<geometry_msgs::msg::Point> {
        std::size_t operator()(const geometry_msgs::msg::Point& p) const {
            std::size_t seed = 0;
            hash_combine(seed, p.x);
            hash_combine(seed, p.y);
            hash_combine(seed, p.z);
            return seed;
        }

        // Helper function to combine hash values
        template <class T>
        static void hash_combine(std::size_t& seed, const T& v) {
            std::hash<T> hasher;
            seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
    };
}

class ScanToCostmap : public rclcpp::Node
{
public:
    ScanToCostmap() : Node("scan_to_costmap"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        this->get_parameter("use_sim_time", use_sim_time_);
        if (use_sim_time_)
        {
            this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        }
        this->declare_parameter<std::string>("target_topic", "fake/scan");
        this->get_parameter("target_topic", target_topic);
        this->declare_parameter<std::string>("base_frame_id_", "base_link");
        this->get_parameter("base_frame_id_", base_frame_id_);
        // Initialize publisher for costmap
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(target_topic, rclcpp::SystemDefaultsQoS(), std::bind(&ScanToCostmap::scanCallback, this, std::placeholders::_1));
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
                "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                std::bind(&ScanToCostmap::receiveMap, this, std::placeholders::_1));
        auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        // costmap_client_ = this->create_client<nav2_msgs::srv::GetCostmap>("/global_costmap/get_costmap");
        costmap_client_ = this->create_client<nav2_msgs::srv::GetCostmap>("/global_costmap/get_costmap");
        costmap_update_pub_ = this->create_publisher<map_msgs::msg::OccupancyGridUpdate>("/global_costmap/costmap_updates", custom_qos);
        // costmap_raw_ = nav2_msgs::msg::Costmap>();
        prepareCostmap();


        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Adjust the duration as needed
            std::bind(&ScanToCostmap::run, this));
        //
    }

private:

    void prepareCostmap()
    {
        auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
        // Set any necessary fields in the request

        while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
            // if (!rclcpp::ok()) {
            //     RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            //     return;
            // }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to appear... and tf tree from map to base_link");
        }

        auto result_future = costmap_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
            return;
        }
        auto response = result_future.get();
        RCLCPP_INFO(this->get_logger(), "\033[1;31m---->Received a costmap\033[0m");
        costmap_.metadata.resolution = response->map.metadata.resolution;
        costmap_.metadata.size_x = response->map.metadata.size_x;
        costmap_.metadata.size_y = response->map.metadata.size_y;
        costmap_.metadata.origin.position.x = response->map.metadata.origin.position.x; 
        costmap_.metadata.origin.position.y = response->map.metadata.origin.position.y; 
        costmap_.data = response->map.data;
        run_ = true;


    }


    void run()
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);

        // Your timer code here
        // This code will run periodically in a separate thread
        // Example: Print a message or perform some periodic checks        // Change the color to blue
        if (!run_)
            return;
		if (!scan_receive_) {
			RCLCPP_WARN(
			  get_logger(),
			  "Not yet received scan. Therefore, safety costmap cannot be initiated.");
              return;
		}
		if (!map_receive_) {
			RCLCPP_WARN(
			  get_logger(),
			  "Not yet received map. Therefore, safety costmap cannot be initiated.");
              return;
		}
        if (get_pose()){
            // updateBounds(current_pose.pose.position.x, current_pose.pose.position.y, robot_yaw, &min_x, &min_y, &max_x, &max_y);
            updateCostmapWithPoints(transformScanToMap(scan));
            RCLCPP_INFO(this->get_logger(), "transformScanToMap is Done");
        }



        // costmap_update_pub_->publish(costmap_raw_);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        if (run_) {
            scan_receive_ = true;
            scan = msg;
        }
    }



    std::vector<geometry_msgs::msg::Point> transformScanToMap(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        std::vector<geometry_msgs::msg::Point> points_in_map;

        // Get the transformation from the laser frame to the map frame
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform("map", scan->header.frame_id, scan->header.stamp, rclcpp::Duration::from_seconds(1.0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to 'map': %s", scan->header.frame_id.c_str(), ex.what());
            return points_in_map;
        }

        // Transform each scan point to the map frame
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float range = scan->ranges[i];
            if (range < scan->range_min || range > scan->range_max) {
                continue;  // Invalid range
            }

            // Calculate angle of the scan point
            double angle = scan->angle_min + i * scan->angle_increment;

            // Convert polar coordinates (range, angle) to Cartesian coordinates (x, y)
            double laser_x = range * cos(angle);
            double laser_y = range * sin(angle);

            // Transform point from laser frame to map frame
            tf2::Vector3 point_laser(laser_x, laser_y, 0.0);
            tf2::Transform transform;
            tf2::fromMsg(transform_stamped.transform, transform);
            tf2::Vector3 point_map = transform * point_laser;

            // Add point to the list
            geometry_msgs::msg::Point point;
            point.x = point_map.x();
            point.y = point_map.y();
            point.z = 0.0;
            points_in_map.push_back(point);
        }


        return getCurrentPoints(points_in_map);

        // return points_in_map;
    }

    std::vector<geometry_msgs::msg::Point> getCurrentPoints(std::vector<geometry_msgs::msg::Point>& points_in_map){
        for (const auto& point : points_in_map) {
            // Inflate points by creating additional points around the original point
            for (double dx = -0.2; dx <= 0.2; dx += 0.1) { // Adjust step and range for desired inflation
                for (double dy = -0.2; dy <= 0.2; dy += 0.1) {
                    geometry_msgs::msg::Point inflated_point;
                    inflated_point.x = point.x + dx;
                    inflated_point.y = point.y + dy;
                    inflated_point.z = 0.0;

                    // Track the lifespan of the inflated point
                    point_lifespan_map[inflated_point] = std::chrono::steady_clock::now();
                }
            }
        }

        // Remove points that are older than 3 seconds from the lifespan map
        auto now = std::chrono::steady_clock::now();
        for (auto it = point_lifespan_map.begin(); it != point_lifespan_map.end();) {
            if (std::chrono::duration_cast<std::chrono::seconds>(now - it->second).count() > 3) {
                it = point_lifespan_map.erase(it);
            } else {
                ++it;
            }
        }
        
        
        
        std::vector<geometry_msgs::msg::Point> current_points;
        
        for (const auto& item : point_lifespan_map) {
            current_points.push_back(item.first);
        }
        return current_points;
    }


    void updateCostmapWithPoints(const std::vector<geometry_msgs::msg::Point>& points_in_map) {
        // Convert robot's position to grid coordinates
        int robot_grid_x = static_cast<int>(std::floor((current_pose.pose.position.x - map_.info.origin.position.x) / map_.info.resolution));
        int robot_grid_y = static_cast<int>(std::floor((current_pose.pose.position.y - map_.info.origin.position.y) / map_.info.resolution));

        // Calculate bounds for a 10-meter radius around the robot
        int radius_in_cells = static_cast<int>(10 / map_.info.resolution); // 10 meters to cells
        int min_x = std::max(0, robot_grid_x - radius_in_cells);
        int max_x = std::min(static_cast<int>(map_.info.width), robot_grid_x + radius_in_cells);
        int min_y = std::max(0, robot_grid_y - radius_in_cells);
        int max_y = std::min(static_cast<int>(map_.info.height), robot_grid_y + radius_in_cells);

        // Initialize data for costmap update
        int width = max_x - min_x + 1;
        int height = max_y - min_y + 1;
        std::vector<int8_t> data(width * height, -1); // Initialize with unknown (-1)

        // Update the costmap within the bounds
        for (const auto& point : points_in_map) {
            int grid_x = static_cast<int>((point.x - map_.info.origin.position.x) / map_.info.resolution);
            int grid_y = static_cast<int>((point.y - map_.info.origin.position.y) / map_.info.resolution);

            if (grid_x >= min_x && grid_x <= max_x && grid_y >= min_y && grid_y <= max_y) {
                int index = (grid_x - min_x) + (grid_y - min_y) * width;
                // costmap_.data[index] = 100; // Mark as occupied
                data[index] = 100;
            }
        }

        // Publish the costmap update
        publishCostmapUpdate(min_x, min_y, width, height, data);
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
        // update_msg->data.resize(update_msg->width * update_msg->height);
        RCLCPP_INFO(this->get_logger(), "\033[1;34mUpdating occupancy grid\033[0m");
        RCLCPP_INFO(this->get_logger(), "The update x y  is : (%d, %d)", update_msg->x, update_msg->y);
        RCLCPP_INFO(this->get_logger(), "The update width height  is : (%d, %d)", update_msg->width, update_msg->height);
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
            // Convert quaternion to yaw
            tf2::Quaternion q(
                current_pose.pose.orientation.x,
                current_pose.pose.orientation.y,
                current_pose.pose.orientation.z,
                current_pose.pose.orientation.w
            );
            robot_yaw = tf2::getYaw(q);
            // Handle the pose as needed
            RCLCPP_INFO(this->get_logger(), "Pose: [%f, %f, %f]", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
            return true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform 'base_link' to 'map': %s", ex.what());
            return false;
        }
    }

    void receiveMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
    {
        map_ = *msg;
        map_receive_ = true;
        RCLCPP_INFO(get_logger(), "Received map.");
        
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
    rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_update_pub_;
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;


    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::TransformStamped transform_stamped;
    geometry_msgs::msg::PoseStamped current_pose;
    bool use_sim_time_;
    // rclcpp::Time scan_time_stamp_;
    // std::string scan_frame_id_;
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    std::mutex scan_mutex_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string target_topic;
    std::string base_frame_id_;
    bool run_ = false;
    bool scan_receive_ = false;
    bool map_receive_ = false;
    double robot_yaw;
    std::unordered_map<geometry_msgs::msg::Point, std::chrono::steady_clock::time_point> point_lifespan_map;

    // float range_min, range_max, angle_min, angle_max;
    nav2_msgs::msg::Costmap costmap_;
    nav_msgs::msg::OccupancyGrid map_;
    unsigned int x0_, xn_, y0_, yn_;
    unsigned int bx0_, bxn_, by0_, byn_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToCostmap>());
    rclcpp::shutdown();
    return 0;
}