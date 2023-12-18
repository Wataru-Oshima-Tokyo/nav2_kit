#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "techshare_ros_pkg2/msg/point_array.hpp"
#include <unordered_set>
#include <sstream>
#include <cmath>
#include "std_srvs/srv/set_bool.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class OccupiedGridPublisher : public rclcpp::Node {
public:
  OccupiedGridPublisher()
  : Node("occupied_grid_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
      this->get_parameter("use_sim_time", use_sim_time_);
      if (use_sim_time_)
      {
          this->set_parameter(rclcpp::Parameter("use_sim_time", true));
      }
    occupancy_grid_update_subscriber_ = this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      "global_costmap/costmap_updates", 10, std::bind(&OccupiedGridPublisher::occupancyGridUpdateCallback, this, std::placeholders::_1));
    scan_for_move_client_ = this->create_client<std_srvs::srv::SetBool>("/toggle_scanning/scan_for_move");
    scan_client_ = this->create_client<std_srvs::srv::SetBool>("/toggle_scanning/scan");
    costmap_client_ = this->create_client<nav2_msgs::srv::GetCostmap>("/global_costmap/get_costmap");
    request_ = std::make_shared<std_srvs::srv::SetBool::Request>();
    // Publisher for the occupied cells topic.
    occupied_cells_publisher_ = this->create_publisher<techshare_ros_pkg2::msg::PointArray>("occupied_cells", 10);
    
    send_request(true);
    prepareCostmap();
    // Timer for resetting the hash map.
    // reset_timer_ = this->create_wall_timer(
    //   std::chrono::seconds(5), std::bind(&OccupiedGridPublisher::resetHashMap, this));


    
  }

private:


    void prepareCostmap()
    {
        auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
        // Set any necessary fields in the request

        while (!costmap_client_->wait_for_service(std::chrono::seconds(1)) || !get_pose()) {
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
        
        initial_costmap_flag = false;
    }





  void occupancyGridUpdateCallback(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
    if (initial_costmap_flag) return;
    // std::vector<geometry_msgs::msg::PointStamped> points_to_publish;
    std::vector<geometry_msgs::msg::Point> points_to_publish;
    for (unsigned int i = 0; i < msg->data.size(); ++i) {
      if (msg->data[i] == 100) {  // Occupied cell
        // Convert index to 2D grid coordinates.
        unsigned int mx = (i % msg->width) + msg->x;
        unsigned int my = (i / msg->width) + msg->y;

        // Convert grid coordinates to world coordinates.
        double wx = costmap_.metadata.origin.position.x + (mx + 0.5) * costmap_.metadata.resolution;
        double wy = costmap_.metadata.origin.position.y + (my + 0.5) * costmap_.metadata.resolution;

        geometry_msgs::msg::Point occupied_cell;
        occupied_cell.x = wx;
        occupied_cell.y = wy;
        points_to_publish.push_back(occupied_cell);
      }
    }
    if (!points_to_publish.empty()) {
      point_array_msg.points = points_to_publish;
      occupied_cells_publisher_->publish(point_array_msg);
      RCLCPP_INFO(this->get_logger(), "Published %zu occupied points", point_array_msg.points.size());
    }
  }


  
  void send_request(bool enable) {
    request_->data = enable;
    std::array<rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr, 2> services = {scan_for_move_client_, scan_client_};
    for (const auto& service : services){
      while (!service->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
      }

      auto result = service->async_send_request(request_,
        std::bind(&OccupiedGridPublisher::handle_response, this, std::placeholders::_1));
    }

  }

  
  void handle_response(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "Response: %s", response->message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to toggle collision detection");
    }
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
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr occupancy_grid_update_subscriber_;
  rclcpp::Publisher<techshare_ros_pkg2::msg::PointArray>::SharedPtr occupied_cells_publisher_;
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
  geometry_msgs::msg::PoseStamped current_pose;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  techshare_ros_pkg2::msg::PointArray point_array_msg;
  bool initial_costmap_flag = true;
  bool use_sim_time_;
  // double threshold_ = 0.15;  // Define a threshold, for example 10 cm.
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr scan_for_move_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr scan_client_;
  std_srvs::srv::SetBool::Request::SharedPtr request_;
  nav2_msgs::msg::Costmap costmap_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OccupiedGridPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}