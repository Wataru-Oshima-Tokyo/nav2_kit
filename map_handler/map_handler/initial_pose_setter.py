import rclpy
import tf2_ros
import tf2_geometry_msgs  # To convert PoseStamped which is often used in marker detections
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
import os
import signal
import subprocess
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from .library.color_print import COLOR_PRINT
from rclpy.qos import QoSProfile
from transforms3d import euler, quaternions
import numpy as np

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('InitialPoseSetter')
        self.color_print = COLOR_PRINT(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        qos_profile = QoSProfile(depth=10)
        self.srv = self.create_service(Trigger, 'kill_marker_tf', self.kill_nodes_callback)
        self.init_pose_sub = self.create_subscription(PoseStamped, "init_pose_set", self.init_pose_callback, qos_profile)
        self.color_print.print_in_purple("initial pose setter started!")

    def killNodes(self):
        try:
            cmd = "ps aux | grep -E 'map_to_marker|marker_localization_node' | grep -v grep | awk '{ print \"kill -9\", $2 }' | sh"
            subprocess.run(cmd, shell=True, check=True)
            self.get_logger().info("'map_to_marker' and 'marker_localization_node client' have been killed.")
            return True
        except subprocess.CalledProcessError:
            return False


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

    def init_pose_callback(self, msg):
        self.killNodes()
        self.color_print.print_in_pink(msg.pose)

        # Fill the TransformStamped message
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "map"
        transform_stamped.child_frame_id = "odom"
        transform_stamped.transform.translation.x = msg.pose.position.x
        transform_stamped.transform.translation.y = msg.pose.position.y
        transform_stamped.transform.translation.z = msg.pose.position.z
        transform_stamped.transform.rotation = msg.pose.orientation

        # Broadcast the static transform
        self.static_broadcaster.sendTransform(transform_stamped)



def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()