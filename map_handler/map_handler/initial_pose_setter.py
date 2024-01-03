import rclpy
import tf2_ros
import tf2_geometry_msgs  # To convert PoseStamped which is often used in marker detections
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from .library.color_print import COLOR_PRINT
from rclpy.qos import QoSProfile
from transforms3d import euler, quaternions
import numpy as np
import time
from std_msgs.msg import Float32
from techshare_ros_pkg2.srv import SendMsg
from std_srvs.srv import Empty

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('InitialPoseSetter')
        self.color_print = COLOR_PRINT(self)
        # self.timer = self.create_timer(0.1, self.publishMapToOdom)  # Check node status every 2 seconds
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.dynamic_broadcaster = tf2_ros.TransformBroadcaster(self)
        qos_profile = QoSProfile(depth=10)
        self.transform = None
        self.srv = self.create_service(Trigger, 'kill_marker_tf', self.kill_nodes_callback)
        self.rcs_send_msg_service = self.create_client(SendMsg, "send_msg")
        self.emcl_tf_publish_set_service = self.create_client(Empty, "emcl_node_finish_")
        self.init_pose_publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'init_pose_set_for_emcl', 10)
        self.msg_req = SendMsg.Request()
        self.emcl_tf_req = Empty.Request()
        self.init_pose_check = False
        self.alpha_array = []
        # self.init_pose_sub = self.create_subscription(PoseWithCovarianceStamped, "init_pose_set", self.init_pose_callback, qos_profile)
        self.init_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'init_pose_set', self.init_pose_callback, qos_profile)
        self.alpha_sub = self.create_subscription(Float32, "alpha", self.alpha_callback, qos_profile)
        self.color_print.print_in_purple("initial pose setter started!")
        self.now = time.time()
        self.initial_publish = True
        self.dlio_wait = True
        self.clear_costmap_flag = True
        self.transform_fetched = False
        # self.nav = BasicNavigator()
        self.timer1 = self.create_timer(1, self.map_to_odom_checker)  # Check node status every 2 seconds
        self.timer2 = self.create_timer(1, self.lookup_transform_for_map_to_marker)  
        # self.timer3 = self.create_timer(2, self.call_clear_costmap)  
        self.timer3 = self.create_timer(0.1, self.lookup_transform_for_odom_to_base_link)  # 0.1 seconds = 10ms = 10Hz




    
    # def call_clear_costmap(self):
    #     if self.dlio_wait:
    #         return    
    #     if self.clear_costmap_flag:
    #         self.nav.clearGlobalCostmap()

    def lookup_transform_for_map_to_marker(self):
        if self.dlio_wait:
            self.get_logger().info('Waiting for dlio started!')
            return
        if not self.transform_fetched:  # Check if transform is not fetched yet
            try:
                # Get the transform from "map" to "marker"
                self.map_to_marker_transform = self.tf_buffer.lookup_transform('map', 'marker', rclpy.time.Time())
                self.get_logger().info('Map to marker Transform obtained successfully!')
                # Set the flag to True after successfully fetching the transform
                self.transform_fetched = True
                msg = PoseWithCovarianceStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"
                msg.pose.pose.position.x = self.map_to_marker_transform.transform.translation.x
                msg.pose.pose.position.y = self.map_to_marker_transform.transform.translation.y
                msg.pose.pose.position.z = 0.0
                msg.pose.pose.orientation.x = self.map_to_marker_transform.transform.rotation.x 
                msg.pose.pose.orientation.y = self.map_to_marker_transform.transform.rotation.y 
                msg.pose.pose.orientation.z = self.map_to_marker_transform.transform.rotation.z 
                msg.pose.pose.orientation.w = 1.0
                self.init_pose_publisher_.publish(msg)
                self.alpha_array = []
                self.color_print.print_in_pink(msg.pose)
                self.init_pose_check = True
                self.msg_req.message = "Now Adjusting the initial pose. Please wait..."
                self.msg_req.error = True
                self.rcs_send_msg_service.call_async(self.msg_req)
                self.now = time.time()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                pass
                #self.get_logger().error('Error looking up transform: %s' % e)     

    def lookup_transform_for_map_to_odom(self):
            try:
                # Get the transform from "map" to "marker"
                self.map_to_odom_transform = self.tf_buffer.lookup_transform('map', 'dlio_odom', rclpy.time.Time())
                self.get_logger().info('Map to dlio_odom Transform obtained successfully!')
                # Set the flag to True after successfully fetching the transform
                return True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                return False
                # self.get_logger().error('Error looking up transform: %s' % e) 
    def lookup_transform_for_odom_to_base_link(self):
        if self.dlio_wait:  # Check if transform is not fetched yet
            try:
                # Get the transform from "map" to "marker"
                odom_base_link_transform = self.tf_buffer.lookup_transform('dlio_odom', 'base_link', rclpy.time.Time())
                self.get_logger().info('Odom to base_link Transform obtained successfully!')
                # Set the flag to True after successfully fetching the transform
                self.dlio_wait = False
                self.now = time.time()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                pass
                # self.get_logger().error('Error looking up transform: %s' % e) 


    def isPassedTime(self, duration):
        if (time.time() - self.now) > duration:
            return True
        else:
            return False

    def map_to_odom_checker(self):
        if self.init_pose_check:
            if len(self.alpha_array) > 100: 
                # alpha_mean = sum(self.alpha_array) / len(self.alpha_array) if self.alpha_array else 0
                alpha_median = np.median(self.alpha_array)
                if self.lookup_transform_for_map_to_odom() and alpha_median > 0.95:
                    self.color_print.print_in_green("Done setting initialpose!!")
                    self.emcl_tf_publish_set_service.call_async(self.emcl_tf_req)
                    self.init_pose_check = False
                    self.msg_req.message = f"Done setting initialpose!! | Alpha median {alpha_median*100}%"
                    self.msg_req.error = False
                    self.rcs_send_msg_service.call_async(self.msg_req)
                    self.clear_costmap_flag = False
                    self.alpha_array = []
                    # time.sleep(2)
                    # self.publishMapToOdom()
                else: 
                    self.msg_req.message = f"Still adjusting the initialpose...| Alpha median {alpha_median*100}%"
                    self.msg_req.error = True
                    self.rcs_send_msg_service.call_async(self.msg_req)
                    self.color_print.print_in_orange(f"Still adjusting the initialpose...| Alpha median {alpha_median}")

    def publishMapToOdom(self):
        # if self.transform is not None and not self.init_pose_check:
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "map"
        # if self.initial_publish:
        #     transform_stamped.child_frame_id = "initial_odom"
        #     self.initial_publish = False
        # else:
        transform_stamped.child_frame_id = "dlio_odom"
        transform_stamped.transform.translation = self.transform.transform.translation
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation = self.transform.transform.rotation

        # Broadcast the static transform
        self.static_broadcaster.sendTransform(transform_stamped)


    def kill_nodes_callback(self, request, response):
        response.success = self.killNodes()
        return response

    def euler_to_quarternion(self, _euler):
        """Convert Euler Angles to Quaternion
        euler: geometry_msgs/Vector3
        quaternion: geometry_msgs/Quaternion
        """
        q = euler.euler2quat(_euler.x, _euler.y, _euler.z, axes='sxyz')
        return q[0], q[1], q[2], q[3]

    def euler_from_quaternion(self, quaternion):
        """Convert Quaternion to Euler Angles
        quaternion: geometry_msgs/Quaternion
        euler: tuple (roll, pitch, yaw)
        """
        e = euler.quat2euler((quaternion.x, quaternion.y, quaternion.z, quaternion.w), axes='sxyz')
        return e[0], e[1], e[2]

    def alpha_callback(self, msg):
        if self.init_pose_check:
            self.alpha_array.append(msg.data)


    def init_pose_callback(self, msg):
        self.init_pose_check = True
        self.alpha_array = []
        self.init_pose_publisher_.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()