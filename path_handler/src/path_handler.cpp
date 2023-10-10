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

    void start(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
               std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        if (!recorded_trajectory_.poses.empty()) {
            trajectory_publisher_->publish(recorded_trajectory_);
            // recorded_trajectory_ = nav_msgs::msg::Path();
        }
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
