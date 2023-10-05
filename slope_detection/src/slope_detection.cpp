#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include <pcl/conversions/from_pcl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

class SlopeDetectionNode : public rclcpp::Node
{
public:
    SlopeDetectionNode()
        : Node("slope_detection_node"), 
          tf_listener_(tf_buffer_, this)
    {
        this->get_parameter("use_sim_time", use_sim_time_);
        if (use_sim_time_)
        {
            this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        }



        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points_raw", 10,
            std::bind(&SlopeDetectionNode::lidarCallback, this, std::placeholders::_1));
    }

private:
        void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {               
            // Convert ROS PointCloud2 to PCL PointCloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud);
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.01); // Adjust this threshold depending on the accuracy of your data

            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Could not estimate a planar model for the given dataset.");
                slope_detected_ = false;
                return;
            }

            // Here, coefficients->values have the coefficients of the detected plane ax+by+cz+d=0
            // You can compute the normal to the plane and determine the slope

            // Find centroid of the inliers (plane points)
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud, *inliers, centroid);
            
            // The centroid variable now has the x, y, z coordinates of the center of the plane
            tf2::Vector3 detected_slope_position(centroid[0], centroid[1], centroid[2]);
            
            // Update slope_position_ and visualize in RViz
            slope_position_ = detected_slope_position;
            slope_detected_ = true;
            publishSlopeTF();
        }

            void publishSlopeTF()
            {
                if (!slope_detected_)
                    return;

                // Create a transform for the detected slope in the "velodyne" frame
                tf2::Transform tfSlopeInVelodyne;
                tfSlopeInVelodyne.setIdentity();
                tfSlopeInVelodyne.setOrigin(slope_position_);

                geometry_msgs::msg::TransformStamped tfStamped;
                tfStamped.header.stamp = this->get_clock()->now();
                tfStamped.header.frame_id = "velodyne"; // parent frame
                tfStamped.child_frame_id = "detected_slope"; // child frame
                tfStamped.transform = tf2::toMsg(tfSlopeInVelodyne);

                tf_broadcaster_->sendTransform(tfStamped);
                RCLCPP_INFO(this->get_logger(), "Publish tfStamped");

            }
        


    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf2_ros::Buffer tf_buffer_{this->get_clock()};
    tf2_ros::TransformListener tf_listener_;
    float threshold_ = 0.2;  
    bool slope_detected_ = false;
    bool use_sim_time_;
    tf2::Vector3 slope_position_ = tf2::Vector3(0, 0, 0);

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlopeDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
