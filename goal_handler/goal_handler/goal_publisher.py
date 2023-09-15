import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration 
from techshare_ros_pkg2.action import TriggerGoalSequence
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
import yaml
import time
import tf2_py as tf2
from tf2_ros import TransformListener, Buffer
from rclpy.clock import ROSClock
import math
import copy

class GoalActionServer(Node):

    def __init__(self):
        super().__init__('nav_goal_handler')
        self.declare_parameter('goals_file', '/')
        self.output_file = self.get_parameter('goals_file').get_parameter_value().string_value + "goals.yaml"
        # Load the goals from the YAML file
        self.tf_buffer = Buffer(Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)  
        self.through_goal_poses_server = ActionServer(
            self,
            TriggerGoalSequence,
            'go_through_poses_sender',
            self.goal_poses_execute_callback)
        

    def get_pose(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform("world", "velodyne", rclpy.time.Time())
            # self.get_logger().info(
            #     f"Robot's pose in map: x = {transform.transform.translation.x}, "
            #     f"y = {transform.transform.translation.y}, "
            #     f"z = {transform.transform.translation.z}")
            return transform
        except tf2.LookupException as ex:
            self.get_logger().error(f"Exception caught: {ex}")
        except tf2.ExtrapolationException as ex:
            self.get_logger().error(f"Exception caught: {ex}")
        return None

    def distance_to_next(self,target_pose):
        current_position = self.get_pose()
        return math.sqrt(math.pow(target_pose.pose.position.x - current_position.transform.translation.x, 2) + math.pow(target_pose.pose.position.y - current_position.transform.translation.y, 2))

    def goal_poses_execute_callback(self, goal_handle):
        self.nav = BasicNavigator()
        with open(self.output_file, 'r') as file:
            self.goals_data = yaml.safe_load(file)
        self.get_logger().info('Executing goal...')
        feedback_msg = TriggerGoalSequence.Feedback()
        goal_poses = []
        initial_position = None
        while initial_position is None:
            initial_position = self.get_pose()
            time.sleep(0.1)
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
        initial_pose.pose.position.x = initial_position.transform.translation.x
        initial_pose.pose.position.y = initial_position.transform.translation.y
        initial_pose.pose.position.z = initial_position.transform.translation.z

        initial_pose.pose.orientation.x = initial_position.transform.rotation.x
        initial_pose.pose.orientation.y = initial_position.transform.rotation.y
        initial_pose.pose.orientation.z = initial_position.transform.rotation.z
        initial_pose.pose.orientation.w = initial_position.transform.rotation.w
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
        absolute_marker_id = 0
        check_goal_start = False
        reached = False
        for j in range(3):
            if len(goal_poses) != 0 and  j>0:
                self.get_logger().info('-------------------------------------------------')
                self.get_logger().info('STILL has some goals so restart soon!')
                self.get_logger().info('-------------------------------------------------')
                goal_poses = goal_poses[absolute_marker_id+1:] 
                self.nav.goThroughPoses(goal_poses)
                nav_start = self.nav.get_clock().now()
                check_goal_start = False

            while not self.nav.isTaskComplete():
                feedback = self.nav.getFeedback()
                current_target_pose_id = len(goal_poses) - feedback.number_of_poses_remaining
                if prev_id != current_target_pose_id:
                    self.get_logger().info('goal id : ' + '{:d}'.format(absolute_marker_id) + ' is reached!')
                    prev_id = current_target_pose_id
                    nav_start = self.nav.get_clock().now()
                    time_within_threshold = None 
                    absolute_marker_id +=1
                    
                    

                now = self.nav.get_clock().now()
                time_difference_secs = int((now.nanoseconds - nav_start.nanoseconds) / 1e9)
                dis_to_waypoint = self.distance_to_next(goal_poses[current_target_pose_id])
                if feedback and i % 5 == 0:
                    self.get_logger().info('-------------------------------------------------')
                    self.get_logger().info('Current pose id: ' + '{:d}'.format(
                    absolute_marker_id))
                    self.get_logger().info('Distance remaining to the end goal: ' + '{:.2f}'.format(
                    feedback.distance_remaining) + ' meters.')
                    self.get_logger().info('Distance remaining to the next waypoint: ' + '{:.2f}'.format(
                    dis_to_waypoint) + ' meters.')
                    self.get_logger().info('Poses remaining: ' + '{:d}'.format(
                    feedback.number_of_poses_remaining) + ' poses')
                    self.get_logger().info('Time elapsed from the previous goal: ' + '{:d}'.format(
                    time_difference_secs) + ' secs')
                    self.get_logger().info('Time elapsed: ' + '{:d}'.format(
                    feedback.navigation_time.sec) + ' secs')
                    self.get_logger().info('-------------------------------------------------')

                        
                """
                here I would like to check if the remaining distance is less than one 
                and it takes 10 seconds to reach the goal then erase(skip) the goal and go to the next pose.

                """
                if dis_to_waypoint < 2.0:  # If distance is less than one meter
                    check_goal_start = True

                # Check the remaining distance
                if check_goal_start and prev_id == current_target_pose_id:
                    if time_within_threshold is None:  # If we haven't already started the timer
                        time_within_threshold = self.nav.get_clock().now()  # Start the timer
                    else:
                        elapsed_time = int((nav_start.nanoseconds - time_within_threshold.nanoseconds) / 1e9)
                        if elapsed_time > 5:  # If more than 10 seconds have passed
                            self.get_logger().info("Skipping current goal due to being within distance threshold for more than 3 seconds.")
                            time_within_threshold = None  # Reset the timer
                            
                            # Skip to the next pose
                            remaining_goals = goal_poses[current_target_pose_id+1:]  # Slice the list to start from the next pose
                            if len(remaining_goals) > 0:  # If there are remaining goals
                                goal_poses = copy.deepcopy(remaining_goals)
                                self.nav.goThroughPoses(goal_poses)  # Send the robot to the next set of poses
                                nav_start = self.nav.get_clock().now()
                                prev_id = 0
                                check_goal_start = False
                            else:
                                self.get_logger().info("No more goals left!")
                                break  # Exit the loop if there are no more goals

                else:
                    time_within_threshold = None  # Reset the timer if robot moves outside the threshold
                    check_goal_start = False

                
                # Some navigation timeout to demo cancellation
                if time_difference_secs > 300.0:
                    self.get_logger().error('MOVE BASE Goal canceled due to timeout')
                    self.nav.cancelTask()
                    break
                time.sleep(1)
                i+=1
            nav_result = self.nav.getResult()
            if nav_result == TaskResult.SUCCEEDED:
                self.get_logger().info('Goal was succeeded!')
                if not reached:
                    reached = True
                    self.nav.goToPose(initial_pose)
                    nav_start = self.nav.get_clock().now()
                    check_goal_start = False
                    continue
                else:
                    break
                # goal_handle.succeed()
            elif nav_result == TaskResult.CANCELED:
                self.get_logger().info('Goal was canceled!')
                break
                # goal_handle.canceled()
            elif nav_result == TaskResult.FAILED:
                self.get_logger().info('Goal failed!')
                continue
                # goal_handle.abort()
            else:
                self.get_logger().info('Goal has an invalid return status!')  
                break  
                # goal_handle.canceled()  
                # Here, you would wait for feedback that the robot has reached its destination.
                # For simplicity, this is omitted, but you'd likely set up a subscriber or some other mechanism.
                

            # Simulated wait for simplicity
        if nav_result == TaskResult.SUCCEEDED:
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
