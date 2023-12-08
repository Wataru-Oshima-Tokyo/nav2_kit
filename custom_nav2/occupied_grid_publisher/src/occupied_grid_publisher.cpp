#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "techshare_ros_pkg2/msg/point_array.hpp"
#include <unordered_set>
#include <sstream>
#include <cmath>
#include "std_srvs/srv/set_bool.hpp"

class OccupiedGridPublisher : public rclcpp::Node {
public:
  OccupiedGridPublisher()
  : Node("occupied_grid_publisher") {
    // Subscriber for the occupancy grid topic.
    occupancy_grid_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "global_costmap/costmap", 10, std::bind(&OccupiedGridPublisher::occupancyGridCallback, this, std::placeholders::_1));
    client_ = this->create_client<std_srvs::srv::SetBool>("toggle_scanning");
    request_ = std::make_shared<std_srvs::srv::SetBool::Request>();
    // Publisher for the occupied cells topic.
    occupied_cells_publisher_ = this->create_publisher<techshare_ros_pkg2::msg::PointArray>("occupied_cells", 10);
    // Timer for resetting the hash map.
    reset_timer_ = this->create_wall_timer(
      std::chrono::seconds(5), std::bind(&OccupiedGridPublisher::resetHashMap, this));


    
  }

private:


  std::string createKey(double x, double y) {
    int x_key = std::floor(x / threshold_);
    int y_key = std::floor(y / threshold_);
    std::stringstream ss;
    ss << x_key << "," << y_key;
    return ss.str();
  }

  void resetHashMap() {
    occupied_cells_publisher_->publish(point_array_msg);
    // Reset the hash map but retain the keys from the first costmap.
    std::unordered_set<std::string> temp = first_costmap_points_;
    published_points_.swap(temp);
    RCLCPP_INFO(this->get_logger(), "Hash map reset, retaining first costmap points.");
  }


  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Iterate through the occupancy grid and check for occupied cells.
    size_t initial_size = published_points_.size();
    
    std::vector<geometry_msgs::msg::Point> points_to_publish;
    for (unsigned int i = 0; i < msg->data.size(); ++i) {
      
      if (msg->data[i] == 100) {  // Occupied cell
        // Convert the index to 2D grid coordinates.
        unsigned int mx = i % msg->info.width;
        unsigned int my = i / msg->info.width;

        // Convert grid coordinates to world coordinates.
        double wx = msg->info.origin.position.x + (mx + 0.5) * msg->info.resolution;
        double wy = msg->info.origin.position.y + (my + 0.5) * msg->info.resolution;

        std::string key = createKey(wx, wy);
        
        if (first_costmap_points_.find(key) == first_costmap_points_.end()) {
          // Publish the occupied cell point.
          published_points_.insert(key);
          geometry_msgs::msg::Point occupied_cell;
          occupied_cell.x = wx;
          occupied_cell.y = wy;
          occupied_cell.z = 0.0;  // Assuming 2D, the z-coordinate is set to zero.
          points_to_publish.push_back(occupied_cell);
        }
      }
    }
    point_array_msg.points = points_to_publish;
    if (initial_costmap_flag){
      first_costmap_points_ = published_points_;
      send_request(true);
      initial_costmap_flag = false;
    }
    size_t new_points = published_points_.size() - initial_size;
    if(new_points > 0) {
      RCLCPP_INFO(this->get_logger(), "\033[1;32m---->New unique points published: %ld\033[0m", new_points);
    }else{
      RCLCPP_INFO(this->get_logger(), "\033[1;31m No new points\033[0m");
    }
  }
  
  void send_request(bool enable) {
    request_->data = enable;
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
    }

    auto result = client_->async_send_request(request_,
      std::bind(&OccupiedGridPublisher::handle_response, this, std::placeholders::_1));
  }
  void handle_response(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "Response: %s", response->message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to toggle collision detection");
    }
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
  rclcpp::Publisher<techshare_ros_pkg2::msg::PointArray>::SharedPtr occupied_cells_publisher_;
  techshare_ros_pkg2::msg::PointArray point_array_msg;
  rclcpp::TimerBase::SharedPtr reset_timer_;
  std::unordered_set<std::string> published_points_;
  std::unordered_set<std::string> first_costmap_points_;
  bool initial_costmap_flag = true;
  double threshold_ = 0.15;  // Define a threshold, for example 10 cm.
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  std_srvs::srv::SetBool::Request::SharedPtr request_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OccupiedGridPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}