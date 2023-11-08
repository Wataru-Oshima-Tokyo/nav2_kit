#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"

class OccupiedGridPublisher : public rclcpp::Node {
public:
  OccupiedGridPublisher()
  : Node("occupied_grid_publisher") {
    // Subscriber for the occupancy grid topic.
    occupancy_grid_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "global_costmap/costmap", 10, std::bind(&OccupiedGridPublisher::occupancyGridCallback, this, std::placeholders::_1));

    // Publisher for the occupied cells topic.
    occupied_cells_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("occupied_cells", 10);
  }

private:
  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Iterate through the occupancy grid and check for occupied cells.
    for (unsigned int i = 0; i < msg->data.size(); ++i) {
      if (msg->data[i] == 100) {  // Occupied cell
        // Convert the index to 2D grid coordinates.
        unsigned int mx = i % msg->info.width;
        unsigned int my = i / msg->info.width;

        // Convert grid coordinates to world coordinates.
        double wx = msg->info.origin.position.x + (mx + 0.5) * msg->info.resolution;
        double wy = msg->info.origin.position.y + (my + 0.5) * msg->info.resolution;

        // Publish the occupied cell point.
        geometry_msgs::msg::Point occupied_cell;
        occupied_cell.x = wx;
        occupied_cell.y = wy;
        occupied_cell.z = 0.0;  // Assuming 2D, the z-coordinate is set to zero.

        occupied_cells_publisher_->publish(occupied_cell);
      }
    }
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr occupied_cells_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OccupiedGridPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}