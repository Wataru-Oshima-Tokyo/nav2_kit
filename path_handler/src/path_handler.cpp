#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/empty.hpp>

using namespace std::placeholders;

class PathRecorder : public rclcpp::Node
{
public:
    PathRecorder() : Node("path_recorder")
    {
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
            RCLCPP_INFO(this->get_logger(), "Appending to the trajectory");
            recorded_trajectory_.header.frame_id = msg->header.frame_id;
            recorded_trajectory_.poses.push_back(msg->poses.back());  // Appending the latest pose
        }
    }

    void record(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        is_recording_ = true;
        recorded_trajectory_ = nav_msgs::msg::Path();
        recorded_trajectory_.header.stamp = this->now(); // Setting the start timestamp
    }

    void reset(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
               std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        recorded_trajectory_ = nav_msgs::msg::Path();
    }

    void finish(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        is_recording_ = false;
        recorded_trajectory_.header.stamp = this->now(); // Updating the timestamp to the end of recording
    }

    // void start(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
    //            std::shared_ptr<std_srvs::srv::Empty::Response> res)
    // {
    //     if (!recorded_trajectory_.poses.empty()) {
    //         trajectory_publisher_->publish(recorded_trajectory_);
    //         // recorded_trajectory_ = nav_msgs::msg::Path();
    //     }
    // }

    void start(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
            std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        if (!recorded_trajectory_.poses.empty()) {
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
            double max_speed = 0.5; // Set this to your robot's average speed in m/s
            double estimated_time_seconds = distance_first_half / max_speed;

            // Publish the two halves with a delay based on the computed time
            trajectory_publisher_->publish(first_half);
            std::chrono::nanoseconds estimated_time_nanoseconds(static_cast<int64_t>(15.0 * 1e9));
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
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_publisher_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr record_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr finish_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_service_;

    nav_msgs::msg::Path recorded_trajectory_;
    bool is_recording_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathRecorder>());
    rclcpp::shutdown();
    return 0;
}
