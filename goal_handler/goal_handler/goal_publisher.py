import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration 
from techshare_ros_pkg2.action import TriggerGoalSequence
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
import yaml
import time

class GoalActionServer(Node):

    def __init__(self):
        super().__init__('nav_goal_handler')
        self.declare_parameter('goals_file', '/')
        self.output_file = self.get_parameter('goals_file').get_parameter_value().string_value + "goals.yaml"
        # Load the goals from the YAML file

        
        self.goal_pose_server = ActionServer(
            self,
            TriggerGoalSequence,
            'go_to_pose_sender',
            self.goal_pose_execute_callback)
        self.through_goal_poses_server = ActionServer(
            self,
            TriggerGoalSequence,
            'go_through_poses_sender',
            self.goal_poses_execute_callback)
        

    def goal_pose_execute_callback(self, goal_handle):
        with open(self.output_file, 'r') as file:
            self.goals_data = yaml.safe_load(file)
        self.get_logger().info('Executing goal...')
        # feedback_msg = TriggerGoalSequence.Feedback()

        # for idx, goal_data in enumerate(self.goals_data['goals']):
        #     goal_pose = PoseStamped()
        #     goal_pose.header.frame_id = 'map'
        #     goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        #     goal_pose.pose.position.x = goal_data['position']['x']
        #     goal_pose.pose.position.y = goal_data['position']['y']
        #     goal_pose.pose.position.z = goal_data['position']['z']

        #     goal_pose.pose.orientation.x = goal_data['orientation']['x']
        #     goal_pose.pose.orientation.y = goal_data['orientation']['y']
        #     goal_pose.pose.orientation.z = goal_data['orientation']['z']
        #     goal_pose.pose.orientation.w = goal_data['orientation']['w']

        #     # self.publisher_.publish(goal_pose)
        #     self.nav.lifecycleStartup()
        #     self.nav.waitUntilNav2Active()
        #     self.nav.goToPose(goal_pose)
        #     i = 0
        #     nav_start = self.nav.get_clock().now()
        #     nav_result = True
        #     while not self.nav.isTaskComplete():
        #         feedback = self.nav.getFeedback()

        #         if feedback and i % 5 == 0:
        #                 self.get_logger().info('Distance remaining: ' + '{:.2f}'.format(
        #                 feedback.distance_remaining) + ' meters.')
                
        #         now = self.nav.get_clock().now()
        #         # Some navigation timeout to demo cancellation
        #         if now - nav_start > Duration(seconds=300.0):
        #             self.get_logger().info('MOVE BASE Goal canceled due to timeout')
        #             self.nav.cancelTask()
        #         time.sleep(1)
        #     nav_result = self.nav.getResult()
        #     if nav_result == TaskResult.SUCCEEDED:
        #         self.get_logger().info('Goal was succeeded!')
        #         self.get_logger().info('Go to the next pose')
        #     elif nav_result == TaskResult.CANCELED:
        #         self.get_logger().info('Goal was canceled!')
        #         goal_handle.canceled()
        #         break
        #     elif nav_result == TaskResult.FAILED:
        #         self.get_logger().info('Goal failed!')
        #         goal_handle.abort()
        #         break
        #     else:
        #         nav_result.get_logger().info('Goal has an invalid return status!')    
        #         goal_handle.canceled()  
        #         break
        #     # Here, you would wait for feedback that the robot has reached its destination.
        #     # For simplicity, this is omitted, but you'd likely set up a subscriber or some other mechanism.
            
        #     feedback_msg.current_goal_index = idx + 1
        #     goal_handle.succeed()
        #     self.get_logger().info(f"Published goal {idx+1}")
        #     goal_handle.publish_feedback(feedback_msg)
        #     if not nav_result:
        #         break
        #     # Simulated wait for simplicity
        #     time.sleep(1)
        # if nav_result:
        #     result = TriggerGoalSequence.Result()
        #     result.success = True
        #     result.message = "All goals have been published!"
        #     return result
        # else:
        #     result = TriggerGoalSequence.Result()
        #     result.success = False
        #     result.message = "All goals have been published!"
        #     return result



    def goal_poses_execute_callback(self, goal_handle):
        self.nav = BasicNavigator()
        with open(self.output_file, 'r') as file:
            self.goals_data = yaml.safe_load(file)
        self.get_logger().info('Executing goal...')
        feedback_msg = TriggerGoalSequence.Feedback()
        goal_poses = []
        for idx, goal_data in enumerate(self.goals_data['goals']):
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
            goal_pose.pose.position.x = goal_data['position']['x']
            goal_pose.pose.position.y = goal_data['position']['y']
            goal_pose.pose.position.z = goal_data['position']['z']

            goal_pose.pose.orientation.x = goal_data['orientation']['x']
            goal_pose.pose.orientation.y = goal_data['orientation']['y']
            goal_pose.pose.orientation.z = goal_data['orientation']['z']
            goal_pose.pose.orientation.w = goal_data['orientation']['w']
            goal_poses.append(goal_pose)
            # self.publisher_.publish(goal_pose)

        
        # nav_result = self.nav.waitUntilNav2Active(localizer="bt_navigator")
        self.nav.goThroughPoses(goal_poses)
        i = 0
        nav_start = self.nav.get_clock().now()
        nav_result = True
        prev_id = 0
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            current_pose_id = len(goal_poses) - feedback.number_of_poses_remaining
            if prev_id != current_pose_id:
                    self.get_logger().info('goal id : ' + '{:d}'.format(current_pose_id) + ' is reached!')
                    prev_id = current_pose_id
                    nav_start = self.nav.get_clock().now()
            now = self.nav.get_clock().now()
            time_difference_secs = int((now.nanoseconds - nav_start.nanoseconds) / 1e9)
            if feedback and i % 5 == 0:
                    self.get_logger().info('Current pose id: ' + '{:d}'.format(
                    current_pose_id))
                    self.get_logger().info('Distance remaining: ' + '{:.2f}'.format(
                    feedback.distance_remaining) + ' meters.')
                    self.get_logger().info('Poses remaining: ' + '{:d}'.format(
                    feedback.number_of_poses_remaining) + ' poses')
                    self.get_logger().info('Time elapsed from the previous goal: ' + '{:d}'.format(
                    time_difference_secs) + ' secs')
                    self.get_logger().info('Time elapsed: ' + '{:d}'.format(
                    feedback.navigation_time.sec) + ' secs')
                     

            
            
            # Some navigation timeout to demo cancellation
            if time_difference_secs > 300.0:
                self.get_logger().info('MOVE BASE Goal canceled due to timeout')
                self.nav.cancelTask()
                break
            time.sleep(1)
            i+=1
        nav_result = self.nav.getResult()
        if nav_result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal was succeeded!')
            goal_handle.succeed()
        elif nav_result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
            goal_handle.canceled()
        elif nav_result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
            goal_handle.abort()
        else:
            self.get_logger().info('Goal has an invalid return status!')    
            goal_handle.canceled()  
            # Here, you would wait for feedback that the robot has reached its destination.
            # For simplicity, this is omitted, but you'd likely set up a subscriber or some other mechanism.
            

            # Simulated wait for simplicity
        if nav_result:
            result = TriggerGoalSequence.Result()
            result.success = True
            result.message = "All goals have been published!"
            return result
        else:
            result = TriggerGoalSequence.Result()
            result.success = False
            result.message = "Failed to reacch the points...."
            return result



def main(args=None):
    rclpy.init(args=args)
    goal_action_server = GoalActionServer()
    rclpy.spin(goal_action_server)
    goal_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
