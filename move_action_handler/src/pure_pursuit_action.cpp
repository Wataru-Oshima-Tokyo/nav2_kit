#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <move_action_handler/action/follow_path.hpp>


class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

        action_server_ = rclcpp_action::create_server<move_action_handler::action::FollowPath>(
            this,
            "follow_path",
            std::bind(&PurePursuitNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PurePursuitNode::handleCancel, this, std::placeholders::_1),
            std::bind(&PurePursuitNode::handleAccepted, this, std::placeholders::_1));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp_action::Server<move_action_handler::action::FollowPath>::SharedPtr action_server_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::PoseStamped current_pose_;
    nav_msgs::msg::Path current_path_;
    double lookahead_distance_ = 1.0;

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        current_path_ = *msg;
    }

    geometry_msgs::msg::PoseStamped getRobotPose()
    {
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_.lookupTransform("map", "base_link", rclcpp::Time(0));
            geometry_msgs::msg::PoseStamped robot_pose;
            robot_pose.pose.position.x = transform.transform.translation.x;
            robot_pose.pose.position.y = transform.transform.translation.y;
            robot_pose.pose.position.z = transform.transform.translation.z;
            robot_pose.pose.orientation = transform.transform.rotation;
            return robot_pose;
        }
        catch (tf2::TransformException &e)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", e.what());
            return geometry_msgs::msg::PoseStamped();  // Return an empty pose by default
        }
    }

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const move_action_handler::action::FollowPath::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received follow path request");
        (void)uuid;
        // Let's accept the request and execute it.
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<move_action_handler::action::FollowPath>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        stopRobot();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void stopRobot()
    {
        auto stop_msg = std::make_shared<geometry_msgs::msg::Twist>();
        stop_msg->linear.x = 0.0;
        stop_msg->angular.z = 0.0;
        vel_pub_->publish(*stop_msg);
    }

    void handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<move_action_handler::action::FollowPath>> goal_handle)
    {
        current_path_ = goal_handle->get_goal()->path;
        while (!isPathCompleted()) {
            computeAndPublish();
            rclcpp::sleep_for(std::chrono::milliseconds(100));  // Adjust as needed
        }
        auto result = std::make_shared<move_action_handler::action::FollowPath::Result>();
        result->success = true;
        result->message = "Destination reached";
        goal_handle->succeed(result);
    }

    bool isPathCompleted()
    {
        // Implement your logic to determine if the robot has completed the path
        return current_path_.poses.empty();
    }

    void computeAndPublish()
    {
        current_pose_ = getRobotPose();
        auto goal_pose = findGoalPoint(current_pose_);
        auto steering_angle = computeSteeringAngle(current_pose_, goal_pose);
        auto vel_msg = computeVelocity(steering_angle);
        vel_pub_->publish(vel_msg);
    }

    geometry_msgs::msg::PoseStamped findGoalPoint(const geometry_msgs::msg::PoseStamped& robot_pose)
    {
        // Implement your logic to find the next goal point based on the lookahead distance
        // This is a placeholder; you'd iterate over `current_path_.poses` and select the appropriate waypoint
        return current_path_.poses.front();
    }

    double computeSteeringAngle(const geometry_msgs::msg::PoseStamped& robot_pose, const geometry_msgs::msg::PoseStamped& goal_pose)
    {
        // Implement your logic to compute the steering angle based on robot and goal poses
        // This is a placeholder; you'd use trigonometry based on the robot's pose and the selected waypoint
        return 0.0;  // placeholder
    }

    geometry_msgs::msg::Twist computeVelocity(double steering_angle)
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.5;  // Adjust your forward speed as needed
        msg.angular.z = steering_angle;
        return msg;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
    