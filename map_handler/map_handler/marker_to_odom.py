import sys
import tf2_ros
import tf2_geometry_msgs  # To convert PoseStamped which is often used in marker detections
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration 
import yaml
import time
import math
from apriltag_msgs.msg import AprilTagDetectionArray
from techshare_ros_pkg2.srv import SendMsg
from std_srvs.srv import Empty, SetBool
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

class MarkerLcalization(Node):
    def __init__(self):
        super().__init__('marker_localization')
            # Create a tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.array_301 = []
        self.array_302 = []
        self.known_distance_for_markers = 0.2
        self.use_sim_time_ = self.get_parameter("use_sim_time").get_parameter_value().bool_value 
        self.get_logger().info(f"use_sim_time {self.use_sim_time_}")
        self.init_pose_publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'init_pose_set', 10)
        self.rcs_send_msg_service = self.create_client(SendMsg, "send_msg")
        self.emcl_tf_publish_set_service = self.create_client(Empty, "emcl_node_finish_")
        self.emcl_send_msg_service = self.create_client(SetBool, "send_msg_service")
        self.emcl_tf_req = Empty.Request()
        self.emcl_send_msg_req = SetBool.Request()
        self.req = SendMsg.Request()
        if self.use_sim_time_:
            self.get_logger().info('USE SIM TIME ')
            self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])
        else:
            self.get_logger().info('Do not USE SIM TIME ')



        self.timer1 = self.create_timer(1, self.lookup_transform_for_map_to_marker)  # 0.01 seconds = 10ms = 100Hz
        self.timer2 = self.create_timer(0.05, self.run)  # 0.1 seconds = 10ms = 10Hz
        self.timer3 = self.create_timer(0.1, self.lookup_transform_for_map_to_odom)  # 0.1 seconds = 10ms = 10Hz
        self.timer4 = self.create_timer(0.1, self.lookup_transform_for_odom_to_base_link)  # 0.1 seconds = 10ms = 10Hz
        self.transform_fetched = False
        self.initial_transform = False
        self.map_to_odom_fetched = False
        self.dlio_wait = True
        self.found_marker = False 
        self.run_ = True
        self.now = time.time()


    def lookup_transform_for_map_to_marker(self):
        if self.dlio_wait:
            return
        if not self.transform_fetched:  # Check if transform is not fetched yet
            try:
                # Get the transform from "map" to "marker"
                self.map_to_marker_transform = self.tf_buffer.lookup_transform('map', 'marker', rclpy.time.Time())
                self.get_logger().info('Map to marker Transform obtained successfully!')
                # Set the flag to True after successfully fetching the transform
                self.transform_fetched = True
                self.now = time.time()
                msg = PoseWithCovarianceStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"
                msg.pose.pose.position.x = self.map_to_marker_transform.transform.translation.x #- median_x_301
                msg.pose.pose.position.y = self.map_to_marker_transform.transform.translation.y #- median_y_301
                msg.pose.pose.position.z = 0.0
                msg.pose.pose.orientation.x = self.map_to_marker_transform.transform.rotation.x 
                msg.pose.pose.orientation.y = self.map_to_marker_transform.transform.rotation.y 
                msg.pose.pose.orientation.z = self.map_to_marker_transform.transform.rotation.z 
                msg.pose.pose.orientation.w = 1.0
                self.init_pose_publisher_.publish(msg)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                pass
                #self.get_logger().error('Error looking up transform: %s' % e)     

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



    def run(self):
        if self.run_:
            try:
                # Get the transform from "map" to "marker"
                if self.dlio_wait:
                    return
                transform301 = self.tf_buffer.lookup_transform('base_link', '301', rclpy.time.Time())
                # transform302 = self.tf_buffer.lookup_transform('base_link', '302', rclpy.time.Time())
                self.array_301.append(transform301)
                # self.array_302.append(transform302)
                if len(self.array_301) < 50:
                    self.get_logger().info('\033[94m' + 'The number of 301 array is : %d' %len(self.array_301) + '\033[0m')
                    self.found_marker = True
                    return 
                self.run_ = False
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error('Error looking up transform: %s' % e)
                if self.transform_fetched and not self.initial_transform and self.isPassedTime(2) and not self.isPassedTime(5):
                    self.req.message = "Still looking for the marker..."
                    self.req.error = True
                    self.rcs_send_msg_service.call_async(self.req)
                elif self.transform_fetched and not self.initial_transform and self.isPassedTime(7) and not self.isPassedTime(10):
                    self.req.message = "Cannot find the marker for 10 seconds..."
                    self.req.error = True
                    self.rcs_send_msg_service.call_async(self.req)
                    self.run_ = False

    def lookup_transform_for_map_to_odom(self):
        if not self.map_to_odom_fetched:  # Check if transform is not fetched yet
            try:
                # Get the transform from "map" to "marker"
                self.map_to_odom_transform = self.tf_buffer.lookup_transform('map', 'initial_odom', rclpy.time.Time())
                self.get_logger().info('Map to odom Transform obtained successfully!')
                # Set the flag to True after successfully fetching the transform
                self.map_to_odom_fetched = True
                self.publish_transform()
                self.initial_transform = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                pass
                # self.get_logger().error('Error looking up transform: %s' % e) 


    def publish_transform(self):
        # Create and set the TransformStamped message

        # Extract x, y translations and z rotation (yaw) from pose_array
        x_translations_301 = sorted([pose.transform.translation.x for pose in self.array_301])
        y_translations_301 = sorted([pose.transform.translation.y for pose in self.array_301])
        z_translations_301 = sorted([pose.transform.translation.z for pose in self.array_301])
        # x_translations_302 = sorted([pose.transform.translation.x for pose in self.array_302])
        # y_translations_302 = sorted([pose.transform.translation.y for pose in self.array_302])


        # Compute median values for translation
        half_index = int(len(x_translations_301)/2)
        median_x_301 = (x_translations_301[half_index-1] + x_translations_301[half_index]) / 2
        median_y_301 = (y_translations_301[half_index-1] + y_translations_301[half_index]) / 2
        median_z_301 = (z_translations_301[half_index-1] + z_translations_301[half_index]) / 2

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "dlio_odom"
        if self.found_marker:
            if not self.use_sim_time_:
                transform.transform.translation.x = self.map_to_marker_transform.transform.translation.x - median_z_301#depth
                transform.transform.translation.y = self.map_to_marker_transform.transform.translation.y - median_y_301 #- width
                transform.transform.translation.z = self.map_to_marker_transform.transform.translation.z  # Assuming 2D movement
            else:
                transform.transform.translation.x = self.map_to_marker_transform.transform.translation.x - median_x_301#depth
                transform.transform.translation.y = self.map_to_marker_transform.transform.translation.y #- median_y_301 #- width
                transform.transform.translation.z = self.map_to_marker_transform.transform.translation.z  # A
        else:
                transform.transform.translation.x = self.map_to_odom_transform.transform.translation.x
                transform.transform.translation.y = self.map_to_odom_transform.transform.translation.y #- width            
        # transform.transform.rotation = self.map_to_odom_transform.transform.rotation  # Assuming 2D movement
        self.static_broadcaster.sendTransform(transform)
        self.get_logger().info(f'marker from robot is: ({median_x_301}, {median_y_301}, {median_z_301})')
        self.get_logger().info(f'robot is located: ({transform.transform.translation.x}, {transform.transform.translation.y})')
        self.initial_transform = True
        self.run_ = True
        self.get_logger().info('Transform obtained successfully!')
        self.req.message = "Found the marker! Now you should see the robot on the map"
        self.req.error = False
        self.rcs_send_msg_service.call_async(self.req)
        # self.get_logger().info(f'marker to odom Y for 301: {msg.pose.pose.position.y}')
        # # self.get_logger().info(f'marker to odom X for 301: {median_x_302}')
        # # self.get_logger().info(f'marker to odom Y for 301: {median_y_302}')
        # self.get_logger().info(f'marker to odom Z: {transform.transform.rotation.z}')

        # Publish the transform

    def euler_from_quaternion(self, quaternion):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x, y, z, w = quaternion
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

        return roll_x, pitch_y, yaw_z

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert euler angles to quaternion
        """
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = MarkerLcalization()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
