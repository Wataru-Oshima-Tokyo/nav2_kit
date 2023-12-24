// #include "rclcpp/rclcpp.hpp"
// #include "nav2_costmap_2d/costmap_2d.hpp"
// #include <random>
// #include "nav2_costmap_2d/cost_values.hpp"
// class RandomCostmapUpdater : public rclcpp::Node {
// public:
//     RandomCostmapUpdater() : Node("random_costmap_updater") {
//         // Initialize costmap (for simplicity, let's create a 10x10 costmap with a resolution of 1 meter)
//         costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(500, 500, 1.0, 0.0, 0.0);

//         // Set a timer to update the costmap
//         timer_ = this->create_wall_timer(
//             std::chrono::seconds(1),
//             std::bind(&RandomCostmapUpdater::updateRandomCells, this));
//     }

// private:
//     void updateRandomCells() {
//         std::random_device rd;
//         std::mt19937 gen(rd());
//         // Adjust the range to fit the 10x10 block within the 500x500 costmap
//         std::uniform_int_distribution<> distribX(0, costmap_->getSizeInCellsX() - 10);
//         std::uniform_int_distribution<> distribY(0, costmap_->getSizeInCellsY() - 10);

//         // Randomly select the top-left corner of the 10x10 block
//         unsigned int start_x = distribX(gen);
//         unsigned int start_y = distribY(gen);

//         // Update the cost for each cell in the 10x10 block
//         for (unsigned int x = start_x; x < start_x + 10; x++) {
//             for (unsigned int y = start_y; y < start_y + 10; y++) {
//                 costmap_->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
//             }
//         }

//         RCLCPP_INFO(this->get_logger(), "Updated 10x10 block starting at (%u, %u)", start_x, start_y);
//     }

//     std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<RandomCostmapUpdater>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class CostmapUpdater : public rclcpp::Node {
public:
    CostmapUpdater() : Node("costmap_updater") {
        // Create a client to get the global costmap
        costmap_client_ = this->create_client<nav2_msgs::srv::GetCostmap>("/global_costmap/get_costmap");
        // Create a publisher to update the global costmap
        auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();

        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap", custom_qos);

        // Request the costmap
        request_costmap();
    }

private:
    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    void request_costmap() {
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
        process_costmap(response->map);
    }

    nav_msgs::msg::OccupancyGrid convertToOccupancyGrid(const nav2_msgs::msg::Costmap& costmap) {
        nav_msgs::msg::OccupancyGrid occupancy_grid;

        // Set basic information
        occupancy_grid.info.width = costmap.metadata.size_x;
        occupancy_grid.info.height = costmap.metadata.size_y;
        occupancy_grid.info.resolution = costmap.metadata.resolution;
        occupancy_grid.info.origin = costmap.metadata.origin;

        // Convert costmap data to occupancy data
        // This is an example. You'll need to map costmap values to occupancy values as needed
        for (const auto& value : costmap.data) {
            int occupancy_value = value; // Convert costmap value to occupancy value
            occupancy_grid.data.push_back(occupancy_value);
        }

        return occupancy_grid;
    }

    void process_costmap(const nav2_msgs::msg::Costmap& costmap) {
        // Modify the costmap here based on your requirements
        nav_msgs::msg::OccupancyGrid new_costmap = convertToOccupancyGrid(costmap);

        // Example: Set a 10x10 block of cells to a high cost
        int size_x = new_costmap.info.width;
        int size_y = new_costmap.info.height;
        for (int x = 0; x < std::min(10, size_x); ++x) {
            for (int y = 0; y < std::min(10, size_y); ++y) {
                int index = x + y * size_x;
                new_costmap.data[index] = 100; // Set to a high cost
            }
        }

        // Publish the updated costmap
        RCLCPP_INFO(this->get_logger(), "Publishing the costmap");
        costmap_pub_->publish(new_costmap);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CostmapUpdater>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

