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

class SlopeDetectionNode : public rclcpp::Node
{
public:
    SlopeDetectionNode()
        : Node("slope_detection_node"), 
          tf_listener_(tf_buffer_, this)
    {
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points_raw", 10,
            std::bind(&SlopeDetectionNode::lidarCallback, this, std::placeholders::_1));
    }

private:
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
        
        float previous_z = *iter_z;
        ++iter_z;

        std::vector<tf2::Vector3> slope_points;

        for (; iter_z != iter_z.end(); ++iter_z, ++iter_x, ++iter_y)
        {
            float current_z = *iter_z;
            
            float gradient = current_z - previous_z;
            
            if (gradient > threshold_)
            {
                tf2::Vector3 point(*iter_x, *iter_y, *iter_z);
                slope_points.push_back(point);
            }
                
            previous_z = current_z;
        }

        // Find the average position
        tf2::Vector3 avg_point(0, 0, 0);
        for (const auto& point : slope_points)
        {
            avg_point += point;
        }
        avg_point /= slope_points.size();

        slope_detected_ = true;
        slope_position_ = avg_point;
        publishSlopeTF();
    }

        void publishSlopeTF()
        {
            if (!slope_detected_)
                return;

            geometry_msgs::msg::TransformStamped tfMapToVelodyne;
            try 
            {
                tfMapToVelodyne = tf_buffer_.lookupTransform("map", "velodyne", rclcpp::Time(0));
            }
            catch (tf2::TransformException &ex) 
            {
                RCLCPP_WARN(this->get_logger(), "%s", ex.what());
                return;
            }

            tf2::Transform tfMapToVelodyneTF;
            tf2::fromMsg(tfMapToVelodyne.transform, tfMapToVelodyneTF);

            // Convert slope position to a tf2::Transform
            tf2::Transform tfSlopeInVelodyne;
            tfSlopeInVelodyne.setIdentity();
            tfSlopeInVelodyne.setOrigin(slope_position_);

            tf2::Transform tfSlopeInMap = tfMapToVelodyneTF * tfSlopeInVelodyne;

            geometry_msgs::msg::TransformStamped tfStamped;
            tfStamped.header.stamp = this->get_clock()->now();
            tfStamped.header.frame_id = "map";
            tfStamped.child_frame_id = "detected_slope";


            tfStamped.transform = tf2::toMsg(tfSlopeInMap);
            tfStamped.transform.translation.x = 51.5;
            tfStamped.transform.translation.y = 48.0;
            tfStamped.transform.translation.z = 0.0;

            tf_broadcaster_->sendTransform(tfStamped);
        }


    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf2_ros::Buffer tf_buffer_{this->get_clock()};
    tf2_ros::TransformListener tf_listener_;
    float threshold_ = 0.2;  
    bool slope_detected_ = false;
    tf2::Vector3 slope_position_ = tf2::Vector3(0, 0, 0);

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlopeDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
