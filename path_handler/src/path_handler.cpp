#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <std_srvs/srv/empty.hpp>
#include <fstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <chrono>
#include <thread>
using namespace std::placeholders;

class PathRecorder : public rclcpp::Node
{
public:
    PathRecorder() : Node("path_recorder"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {        
        this->get_parameter("use_sim_time", use_sim_time_);
        if (use_sim_time_)
        {
            this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        }
        this->declare_parameter<std::string>("trajectory_file_path", "./");
        this->get_parameter("trajectory_file_path", file_path);
        trajectory_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            "/lio_sam/mapping/path",
            10,
            std::bind(&PathRecorder::trajectoryCallback, this, std::placeholders::_1));

        record_service_ = this->create_service<std_srvs::srv::Empty>("trajectory_record_start", 
            std::bind(&PathRecorder::record, this, _1, _2));

        reset_service_ = this->create_service<std_srvs::srv::Empty>("trajectory_record_reset", 
            std::bind(&PathRecorder::reset, this, _1, _2));

        finish_service_ = this->create_service<std_srvs::srv::Empty>("trajectory_record_finish", 
            std::bind(&PathRecorder::finish, this, _1, _2));

        start_service_ = this->create_service<std_srvs::srv::Empty>("generate_trajectory", 
            std::bind(&PathRecorder::start, this, _1, _2));

        trajectory_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/replay_trajectory", 10);

    }

private:
    void trajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (is_recording_ && !msg->poses.empty()) {
            
            
            bool should_append = true;
            
            if (!recorded_trajectory_.poses.empty()) {
                // Retrieve the last pose in recorded_trajectory_
                auto& last_pose = recorded_trajectory_.poses.back().pose;
                
                // Retrieve the latest pose from msg
                auto& new_pose = msg->poses.back().pose;

                // Compute the position difference
                double dx = last_pose.position.x - new_pose.position.x;
                double dy = last_pose.position.y - new_pose.position.y;
                double dz = last_pose.position.z - new_pose.position.z;

                double distance = sqrt(dx*dx + dy*dy + dz*dz);

                double threshold = 0.05; // e.g., 5 cm
                if (distance < threshold) {
                    should_append = false;
                }
            }

            if (should_append) {
                recorded_trajectory_.header.frame_id = msg->header.frame_id;
                recorded_trajectory_.poses.push_back(msg->poses.back());  // Appending the latest pose
                RCLCPP_INFO(this->get_logger(), "\033[34mAppending to the trajectory\033[0m");
            }
        }
    }

    void record(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        is_recording_ = true;
        // recorded_trajectory_ = nav_msgs::msg::Path();
        recorded_trajectory_.header.stamp = this->now(); // Setting the start timestamp
    }

    void reset(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
               std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        recorded_trajectory_ = nav_msgs::msg::Path();
        is_recording_ = false;
    }

    void finish(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        is_recording_ = false;
        std::string file_name = file_path + "/recorded_trajectory.txt";
        while(!get_pose()){
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        recorded_trajectory_.poses.push_back(current_pose);
        if (!recorded_trajectory_.poses.empty())
            saveToFile(file_name);

    }

    void start(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
            std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        std::string file_name = file_path + "/recorded_trajectory.txt";
        loadFromFile(file_name);
        if (!recorded_trajectory_.poses.empty()) {
            recorded_trajectory_.header.stamp = this->now(); // Updating the timestamp to the end of recording
            size_t midpoint = recorded_trajectory_.poses.size() / 2;

            // Create two new paths for the two halves
            nav_msgs::msg::Path first_half, second_half;
            first_half.header = recorded_trajectory_.header;
            second_half.header = recorded_trajectory_.header;

            for (size_t i = 0; i < midpoint; ++i) {
                first_half.poses.push_back(recorded_trajectory_.poses[i]);
            }
            
            for (size_t i = midpoint; i < recorded_trajectory_.poses.size(); ++i) {
                second_half.poses.push_back(recorded_trajectory_.poses[i]);
            }

            // Calculate the cumulative distance of the first half
            double distance_first_half = computePathDistance(first_half);
            double max_speed = 1.0; // Set this to your robot's average speed in m/s
            double estimated_time_seconds = distance_first_half / max_speed;

            // Publish the two halves with a delay based on the computed time
            trajectory_publisher_->publish(first_half);
            std::chrono::nanoseconds estimated_time_nanoseconds(static_cast<int64_t>(estimated_time_seconds * 1e9));
            RCLCPP_INFO(this->get_logger(), "Will wait for %lf seconds", estimated_time_seconds);
            rclcpp::sleep_for(estimated_time_nanoseconds);
            trajectory_publisher_->publish(second_half);
        }
    }

    double computePathDistance(const nav_msgs::msg::Path& path) {
        double total_distance = 0.0;
        for (size_t i = 1; i < path.poses.size(); ++i) {
            auto& pose_a = path.poses[i - 1].pose.position;
            auto& pose_b = path.poses[i].pose.position;
            double segment_distance = std::sqrt(
                std::pow(pose_b.x - pose_a.x, 2) +
                std::pow(pose_b.y - pose_a.y, 2) +
                std::pow(pose_b.z - pose_a.z, 2)
            );
            total_distance += segment_distance;
        }
        return total_distance;
    }

    void saveToFile(const std::string& filename) {
        std::ofstream out_file(filename);
        if (!out_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the file: %s", filename.c_str());
            return;
        }


        // Save the frame_id
        out_file << "#FRAME_ID:" << recorded_trajectory_.header.frame_id << std::endl;


        for (const auto& poseStamped : recorded_trajectory_.poses) {
            const auto& position = poseStamped.pose.position;
            out_file << position.x << "," << position.y << "," << position.z << std::endl;
        }

        out_file.close();
        RCLCPP_INFO(this->get_logger(), "Saved trajectory to: %s", filename.c_str());
    }

    bool  get_pose() {
        try {
            // Replace 'now' with 'tf2::TimePointZero' if you want to get the latest available transform
            geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_.lookupTransform("odom", "base_link", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0)); 
            current_pose.header.stamp = transformStamped.header.stamp;
            current_pose.header.frame_id = "odom";
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

    void loadFromFile(const std::string& filename) {
        std::ifstream in_file(filename);
        if (!in_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the file: %s", filename.c_str());
            return;
        }

        std::string line;

        // Read the frame_id
        std::getline(in_file, line);
        if (line.find("#FRAME_ID:") == 0) {
            recorded_trajectory_.header.frame_id = line.substr(10);  // Extract the frame_id after "#FRAME_ID:"
        }

        // Read the trajectory poses
        recorded_trajectory_.poses.clear();
        while (std::getline(in_file, line)) {
            if (line.find("#") == 0) continue;  // Skip any comment lines

            std::istringstream ss(line);
            double x, y, z;
            char comma;
            if (!(ss >> x >> comma >> y >> comma >> z)) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse a line in the file: %s", line.c_str());
                continue;
            }

            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.pose.position.x = x;
            pose_stamped.pose.position.y = y;
            pose_stamped.pose.position.z = z;

            recorded_trajectory_.poses.push_back(pose_stamped);
        }

        in_file.close();
        RCLCPP_INFO(this->get_logger(), "Loaded trajectory from: %s", filename.c_str());
    }



    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_publisher_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr record_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr finish_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_service_;

    nav_msgs::msg::Path recorded_trajectory_;
    std::string file_path;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::PoseStamped current_pose;
    bool is_recording_ = false;
    bool use_sim_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathRecorder>());
    rclcpp::shutdown();
    return 0;
}
