#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "techshare_ros_pkg2/msg/point_array.hpp"
#include <unordered_set>
#include <sstream>
#include <cmath>


class OccupiedGridPublisher : public rclcpp::Node {
public:
  OccupiedGridPublisher()
  : Node("occupied_grid_publisher") {
    // Subscriber for the occupancy grid topic.
    occupancy_grid_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "global_costmap/costmap", 10, std::bind(&OccupiedGridPublisher::occupancyGridCallback, this, std::placeholders::_1));

    // Publisher for the occupied cells topic.
    occupied_cells_publisher_ = this->create_publisher<techshare_ros_pkg2::msg::PointArray>("occupied_cells", 10);
  }

private:


  std::string createKey(double x, double y) {
    int x_key = std::floor(x / threshold_);
    int y_key = std::floor(y / threshold_);
    std::stringstream ss;
    ss << x_key << "," << y_key;
    return ss.str();
  }

  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Iterate through the occupancy grid and check for occupied cells.
    size_t initial_size = published_points_.size();
    techshare_ros_pkg2::msg::PointArray point_array_msg;
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
        
        if (published_points_.find(key) == published_points_.end()) {
          // Publish the occupied cell point.
          published_points_.insert(key);
          geometry_msgs::msg::Point occupied_cell;
          occupied_cell.x = wx;
          occupied_cell.y = wy;
          occupied_cell.z = 0.0;  // Assuming 2D, the z-coordinate is set to zero.
          points_to_publish.push_back(occupied_cell);
          // occupied_cells_publisher_->publish(occupied_cell);
        }
      }
    }
    point_array_msg.points = points_to_publish;
    occupied_cells_publisher_->publish(point_array_msg);
    size_t new_points = published_points_.size() - initial_size;
    if(new_points > 0) {
      RCLCPP_INFO(this->get_logger(), "\033[1;32m---->New unique points published: %ld\033[0m", new_points);
    }else{
      RCLCPP_INFO(this->get_logger(), "\033[1;31m No new points\033[0m");
    }
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
  rclcpp::Publisher<techshare_ros_pkg2::msg::PointArray>::SharedPtr occupied_cells_publisher_;
  std::unordered_set<std::string> published_points_;
  double threshold_ = 0.1;  // Define a threshold, for example 10 cm.
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OccupiedGridPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}