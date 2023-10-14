import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import PoseStamped, Twist
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
import os
import datetime
import random
from nav_msgs.msg import Path

class GoalActionServer(Node):

    def __init__(self):
        super().__init__('nav_goal_handler')
        self.declare_parameter('goals_file', '/')
        self.declare_parameter('goal_name', 'goals.yaml')
        self.declare_parameter('cmd_vel', 'cmd_vel')
        self.output_file = (self.get_parameter('goals_file').get_parameter_value().string_value 
                            + self.get_parameter('goal_name').get_parameter_value().string_value )
        # Get the current date and time and format it as a string
        current_datetime = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        base_filename = 'current_pose_{}.yaml'.format(current_datetime)

        # Append it to your base file path
        self.result_file = os.path.join(
            self.get_parameter('goals_file').get_parameter_value().string_value,
            base_filename
        )
        # Load the goals from the YAML file
        self.tf_buffer = Buffer(Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)  
        self.current_speed = 0.0
        self.through_goal_poses_server = ActionServer(
            self,
            TriggerGoalSequence,
            'go_through_poses_sender',
            execute_callback = self.goal_poses_execute_callback,
            goal_callback= self.goal_callback,
            cancel_callback= self.cancel_callback
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel').get_parameter_value().string_value,
            self.cmd_vel_listener_callback,
            10)
        

        self.path_subscription = self.create_subscription(
            Path,
            '/replay_trajectory',
            self.path_callback,
            10)
        
        self.cmd_vel_sub  # prevent unused variable warning
        self.navigation_goal = None
        self.follow_path_nav = BasicNavigator()
        self.first_path = True



    def cmd_vel_listener_callback(self, msg):
        self.current_speed = msg.linear.x

    def transform_to_dict(self, transform):
        return {
            'position': {
                'x': transform.transform.translation.x,
                'y': transform.transform.translation.y,
                'z': transform.transform.translation.z
            },
            'orientation': {
                'x': transform.transform.rotation.x,
                'y': transform.transform.rotation.y,
                'z': transform.transform.rotation.z,
                'w': transform.transform.rotation.w
            }
        }
    def writeTheResult(self, pose, pose_str):
        pose_dict = self.transform_to_dict(pose)
        # If the result file exists, load its content
        if os.path.exists(self.result_file):
            with open(self.result_file, 'r') as file:
                data = yaml.safe_load(file)
                if data is None:
                    data = {}
        else:
            data = {}

        # Check if 'pose' key exists and if not, initialize it as a list
        if pose_str not in data:
            data[pose_str] = []

        # Ensure pose_str is a list and then append the new pose data
        if isinstance(data[pose_str], list):
            data[pose_str].append(pose_dict)
        elif pose_str in data and not isinstance(data[pose_str], list):
            self.print_in_red(f"Data Type of current_pose: {type(data[pose_str])}")
            self.print_in_red("Removing invalid current_pose from data.")
            del data[pose_str]
            return                  
        else:
            # Handle unexpected cases where pose_str is not a list
            self.print_in_yellow("Error: Unexpected data format in YAML file.")
            return

        # Write the updated dictionary back to the YAML file
        with open(self.result_file, 'w') as file:
            yaml.safe_dump(data, file)
        self.print_in_brown(f"Write the {pose_str}")


    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z  # in radians

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert euler angles (roll, pitch, yaw) to quaternion (x, y, z, w)
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        return [x, y, z, w]


    def print_in_orange(self, msg):
        orange_start = "\033[38;5;208m"  # Orange color in 256-color mode.
        color_end = "\033[0m"
        self.get_logger().info(f"{orange_start}{msg}{color_end}")

    def print_in_blue(self, msg):
        blue_start = "\033[94m"
        color_end = "\033[0m"
        self.get_logger().info(f"{blue_start}{msg}{color_end}")

    def print_in_green(self, msg):
        green_start = "\033[92m"
        color_end = "\033[0m"
        self.get_logger().info(f"{green_start}{msg}{color_end}")

    def print_in_brown(self, msg):
        brown_start = "\033[38;5;130m"  # Brown color in 256-color mode.
        color_end = "\033[0m"
        self.get_logger().info(f"{brown_start}{msg}{color_end}")

    def print_in_purple(self, msg):
        purple_start = "\033[95m"  # Light magenta, which might look like purple in most terminals.
        color_end = "\033[0m"
        self.get_logger().info(f"{purple_start}{msg}{color_end}")

    def print_in_pink(self, msg):
        pink_start = "\033[38;5;213m"  # Alternative pink color in 256-color mode.
        color_end = "\033[0m"
        self.get_logger().info(f"{pink_start}{msg}{color_end}")

    def print_in_red(self, msg):
        red_start = "\033[31m"  # Red color.
        color_end = "\033[0m"
        self.get_logger().info(f"{red_start}{msg}{color_end}")

    def print_in_yellow(self, msg):
        yellow_start = "\033[33m"  # Yellow color.
        color_end = "\033[0m"
        self.get_logger().info(f"{yellow_start}{msg}{color_end}")

    def get_pose(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform("world", "velodyne", rclpy.time.Time())
            return transform
        except tf2.LookupException as ex:
            self.get_logger().error(f"Exception caught: {ex}")
        except tf2.ExtrapolationException as ex:
            self.get_logger().error(f"Exception caught: {ex}")
        return None

    def distance_to_next(self,target_pose):
        current_position = self.get_pose()
        return math.sqrt(math.pow(target_pose.pose.position.x - current_position.transform.translation.x, 2) + math.pow(target_pose.pose.position.y - current_position.transform.translation.y, 2))

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.print_in_green('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.print_in_red('Received cancel request')
        return CancelResponse.ACCEPT


    def path_callback(self, msg):
        self.get_logger().info('Received a goal')
        while not self.first_path and not self.follow_path_nav.isTaskComplete():
            self.print_in_orange('Waiting for the previous goal is finished')
            time.sleep(1)

        if msg.poses:
            self.get_logger().info('Publish the path')
            # self.nav.cancelTask()
            self.first_path = not self.first_path
            self.follow_path_nav.followPath(msg)
            
            # inital_pose = self.get_initial_pose()
            # self.path_checker(msg.poses, inital_pose, 1)
        else:
            self.get_logger().warn('Path is empty')

    def get_initial_pose(self):
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
        self.writeTheResult(initial_position, 'initial_pose')
        return initial_pose

    def path_checker(self, goal_poses, initial_pose, current_attempt):
        i = 0
        nav_start = self.nav.get_clock().now()
        absolute_nav_time = self.nav.get_clock().now()
        nav_result = True
        current_pose_id = 0
        absolute_marker_id = 1
        check_goal_start = False
        reached = False
        # previous_speed = 0
        time_within_threshold = None 
        # self.nav.clearAllCostmaps()
        for j in range(3):
            if len(goal_poses) != 0 and  j >0 and not reached:
                self.print_in_blue('-------------------------------------------------')
                self.print_in_yellow('STILL has some goals so restart soon!')
                self.print_in_blue('-------------------------------------------------')
                remaining_goals = goal_poses[absolute_marker_id-1:] 
                self.nav.cancelTask()
                self.nav.goThroughPoses(remaining_goals)
                nav_start = self.nav.get_clock().now()
                check_goal_start = False
                current_pose_id = len(goal_poses) - len(remaining_goals)

            while not self.nav.isTaskComplete():
                feedback = self.nav.getFeedback()
                target_pose_id = len(goal_poses) - feedback.number_of_poses_remaining

                if current_pose_id != target_pose_id and not reached:
                    self.print_in_pink('goal id : ' + '{:d}'.format(absolute_marker_id) + ' is reached!')
                    current_pose_id = target_pose_id
                    nav_start = self.nav.get_clock().now()
                    time_within_threshold = None 
                    absolute_marker_id +=1
                
                # if previous_speed != self.current_speed:
                #     if self.current_speed == - 0.2:
                #         self.print_in_yellow('Object detection mode: Reset costmap')
                #         self.nav.clearLocalCostmap()
                        
                # previous_speed = self.current_speed 

                now = self.nav.get_clock().now()
                time_difference_secs = int((now.nanoseconds - nav_start.nanoseconds) / 1e9)
                abs_nav_time = int((now.nanoseconds - absolute_nav_time.nanoseconds) / 1e9)
                dis_to_waypoint = self.distance_to_next(goal_poses[target_pose_id])
                if feedback and i % 5 == 0:
                    self.print_in_blue('-------------------------------------------------')
                    self.print_in_orange('Current attempt is ' + '{:d}'.format(
                    current_attempt) + " th")
                    self.print_in_green('Target pose id: ' + '{:d}'.format(
                    absolute_marker_id))
                    self.get_logger().info('Distance remaining to the end goal: ' + '{:.2f}'.format(
                    feedback.distance_remaining) + ' meters.')
                    self.print_in_purple('Distance remaining to the next waypoint: ' + '{:.2f}'.format(
                    dis_to_waypoint) + ' meters.')
                    self.get_logger().info('Poses remaining: ' + '{:d}'.format(
                    feedback.number_of_poses_remaining) + ' poses')
                    self.get_logger().info('Time elapsed from the previous goal: ' + '{:d}'.format(
                    time_difference_secs) + ' secs')
                    self.get_logger().info('Time elapsed: ' + '{:d}'.format(
                    abs_nav_time) + ' secs')
                    self.print_in_blue('-------------------------------------------------')

                        
                """
                here I would like to check if the remaining distance is less than one 
                and it takes 10 seconds to reach the goal then erase(skip) the goal and go to the next pose.

                """
                if dis_to_waypoint < 2:  # If distance is less than one meter
                    check_goal_start = True
                elif dis_to_waypoint > 5:
                    check_goal_start = False
                
                # Check the remaining distance
                if check_goal_start and not reached:
                    if time_within_threshold is None:  # If we haven't already started the timer
                        time_within_threshold = self.nav.get_clock().now()  # Start the timer
                        self.print_in_brown("goal reach time now starts ticking!!")
                    else:
                        elapsed_time = int((now.nanoseconds - time_within_threshold.nanoseconds) / 1e9)
                        if elapsed_time > 3:  # If more than 10 seconds have passed
                            self.print_in_yellow("Skipping current goal due to being within distance threshold for more than 3 seconds.")
                            time_within_threshold = None  # Reset the timer
                            
                            # Skip to the next pose
                            remaining_goals = goal_poses[absolute_marker_id:]  # Slice the list to start from the next pose
                            if len(remaining_goals) > 0:  # If there are remaining goals
                                self.nav.cancelTask()
                                self.nav.goThroughPoses(remaining_goals)  # Send the robot to the next set of poses
                                nav_start = self.nav.get_clock().now()
                                current_pose_id = 0
                                check_goal_start = False
                                # absolute_marker_id +=1
                            else:
                                self.print_in_blue("No more goals left!")
                                reached = True
                                goal_poses =[]
                                self.nav.cancelTask()
                                


                else:
                    time_within_threshold = None  # Reset the timer if robot moves outside the threshold

                
                # Some navigation timeout to demo cancellation
                if time_difference_secs > 300.0:
                    self.print_in_red('MOVE BASE Goal canceled due to timeout')
                    self.nav.cancelTask()
                    break

                time.sleep(1)
                i+=1

            nav_result = self.nav.getResult()
            if reached or nav_result == TaskResult.SUCCEEDED:
                self.print_in_pink('Now going back to the initial position')
                initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
                self.nav.goToPose(initial_pose)
                nav_start = self.nav.get_clock().now()
                check_goal_start = False

                while not self.nav.isTaskComplete():
                    time.sleep(1)


            nav_result = self.nav.getResult()
            if nav_result == TaskResult.SUCCEEDED:
                self.print_in_green('Goal was succeeded!')
                # TODO: here I would like to get the the current pose and write it to a yaml file
                # Fetch the current pose
                current_pose = None
                while current_pose is None:
                    current_pose = self.get_pose()
                    time.sleep(0.1)
                if current_pose:
                    self.writeTheResult(current_pose, 'current_pose')
                break
            elif nav_result == TaskResult.CANCELED:
                self.print_in_yellow('Goal was canceled!')
                break
                # goal_handle.canceled()
            elif nav_result == TaskResult.FAILED:
                self.print_in_red('Goal failed!')
                continue
                # goal_handle.abort()
            else:
                self.print_in_red('Goal has an invalid return status!')  
                loop = False
                break  
                # goal_handle.canceled()  
                # Here, you would wait for feedback that the robot has reached its destination.
                # For simplicity, this is omitted, but you'd likely set up a subscriber or some other mechanism.
        return nav_result

    async def goal_poses_execute_callback(self, goal_handle):
        self.nav = BasicNavigator()
        with open(self.output_file, 'r') as file:
            self.goals_data = yaml.safe_load(file)
        self.get_logger().info('Executing goal...')
        feedback_msg = TriggerGoalSequence.Feedback()
        goal_poses = []
        loop = goal_handle.request.loop
        self.print_in_blue("The loop is " + '{:d}'.format(loop))
        initial_pose = self.get_initial_pose()
        fail_counter = 0
        current_attempt = 1
        while True:
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
            self.nav.cancelTask()
            self.nav.goThroughPoses(goal_poses)
            nav_result = self.path_checker(goal_poses, initial_pose, current_attempt)
            if nav_result != TaskResult.SUCCEEDED:
                fail_counter +=1
            if loop == False or fail_counter > 100:
                break     
            else:
                current_attempt +=1

        # Simulated wait for simplicity
        if nav_result == TaskResult.SUCCEEDED:
            result = TriggerGoalSequence.Result()
            result.success = True
            result.message = "All goals have been published!"
            goal_handle.succeed()
            return result
        else:
            result = TriggerGoalSequence.Result()
            result.success = False
            result.message = "Failed to reacch the points...."
            goal_handle.abort()
            return result



def main(args=None):
    rclpy.init(args=args)
    goal_action_server = GoalActionServer()
    rclpy.spin(goal_action_server)
    goal_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
