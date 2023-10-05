#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>


class PointCloudTrimmer : public rclcpp::Node
{
public:
    PointCloudTrimmer() : Node("pointcloud_trimmer")
    {
            // Declare parameters with default values
        this->declare_parameter<double>("min_height", 0.0);
        this->declare_parameter<double>("max_height", 1.5);
        this->declare_parameter<double>("angle_min", -60.0);
        this->declare_parameter<double>("angle_max", 60.0);
        this->declare_parameter<double>("range_min", 0.1);
        this->declare_parameter<double>("range_max", 1.5);
        this->declare_parameter<std::string>("cloud_in", "points_raw");
        this->declare_parameter<std::string>("cloud_out", "trimmed_points");

        // Get the parameters
        this->get_parameter("min_height", min_height_);
        this->get_parameter("max_height", max_height_);
        this->get_parameter("angle_min", min_angle_);
        this->get_parameter("angle_max", max_angle_);
        this->get_parameter("range_min", min_length_);
        this->get_parameter("range_max", max_length_);
        this->get_parameter("cloud_in", cloud_in);
        this->get_parameter("cloud_out", cloud_out);
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_in, 10, std::bind(&PointCloudTrimmer::callback, this, std::placeholders::_1));
                // Create the service client
        clear_costmap_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("global_costmap/clear_entirely_global_costmap");

        // Setup the timer to call the service client every 2 seconds
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&PointCloudTrimmer::clearCostmapCallback, this));
    }

private:

        void clearCostmapCallback()
        {
            // Call the service to clear the global costmap
            auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
            auto future = clear_costmap_client_->async_send_request(request);
            // RCLCPP_INFO(this->get_logger(), "Clear cost map");

        }

        void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr trimmed_cloud(new pcl::PointCloud<pcl::PointXYZ>());

            pcl::fromROSMsg(*msg, *cloud);


            for (const auto &point : cloud->points)
            {
                if (point.z >= min_height_ && point.z <= max_height_)
                {
                    double distance = std::hypot(point.x, point.y);
                    double angle = std::atan2(point.y, point.x) * 180.0 / M_PI;

                    if (angle >= min_angle_ && angle <= max_angle_ && distance >= min_length_ && distance <= max_length_)
                    {
                        trimmed_cloud->points.push_back(point);
                    }
                }
            }


            // Segment the ground
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.01);  // This threshold determines how close a point must be to the model in order to be considered an inlier.

            seg.setInputCloud(trimmed_cloud);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() == 0)
            {
                RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
                return;
            }

            // Extract non-ground points
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(trimmed_cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*trimmed_cloud);

            // for (const auto &point : trimmed_cloud->points)
            // {
            //     if (point.z >= min_height_ && point.z <= max_height_)
            //     {
            //         double distance = std::hypot(point.x, point.y);
            //         double angle = std::atan2(point.y, point.x) * 180.0 / M_PI;

            //         if (angle >= min_angle_ && angle <= max_angle_ && distance >= min_length_ && distance <= max_length_)
            //         {
            //             trimmed_cloud->points.push_back(point);
            //         }
            //     }
            // }

            // trimmed_cloud->width = trimmed_cloud->points.size();
            // trimmed_cloud->height = 1;

            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*trimmed_cloud, output);
            output.header = msg->header;

            publisher_->publish(output);
        }


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_costmap_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    private:
        double min_height_;
        double max_height_;
        double min_angle_;
        double max_angle_;
        double max_length_;
        double min_length_;
        std::string cloud_in;
        std::string cloud_out;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudTrimmer>());
    rclcpp::shutdown();
    return 0;
}
