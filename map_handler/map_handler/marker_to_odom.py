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

class MarkerLcalization(Node):
    def __init__(self):
        super().__init__('marker_localization')
            # Create a tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        use_sim_time = self.get_parameter("use_sim_time")
        if use_sim_time:
             self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])

        # Subscribe to the marker detection topic
        self.marker_position_on_map = []
        self.marker_position_on_map.append(None)
        self.marker_position_on_map.append(None)

        self.timer1 = self.create_timer(1, self.lookup_transform)  # 0.01 seconds = 10ms = 100Hz
        self.timer2 = self.create_timer(1, self.run)  # 0.01 seconds = 10ms = 100Hz
        self.transform_fetched = False
        self.initial_transform = False


    def lookup_transform(self):
        if not self.transform_fetched:  # Check if transform is not fetched yet
            try:
                # Get the transform from "map" to "marker"
                transform = self.tf_buffer.lookup_transform('map', 'marker', rclpy.time.Time())
                self.get_logger().info('Transform obtained successfully!')
                # Set the flag to True after successfully fetching the transform
                self.transform_fetched = True

                # Use the transform as required
                # ... [your code here] ...
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error('Error looking up transform: %s' % e)

    def run(self):
        try:
            # Get the transform from "map" to "marker"
            transform = self.tf_buffer.lookup_transform('base_link', '301', rclpy.time.Time())
            # Set the flag to True after successfully fetching the transform
            if self.transform_fetched and not self.initial_transform:
                # robot_position = self.compute_robot_position_on_map(transform.transform)
                self.publish_transform(transform.transform)
                self.initial_transform = True
                self.get_logger().info('Transform obtained successfully!')
            # Use the transform as required
            # ... [your code here] ...
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error('Error looking up transform: %s' % e)



    def compute_robot_position_on_map(self, x,y):
        # Compute the robot's position based on the marker's position on the map and relative_position
        # This is a simplification; adjust as needed
        
        robot_x = -x
        robot_y = -y
        self.get_logger().info(f'camera to marker X: {x}')
        self.get_logger().info(f'camera to marker Y: {y}')
        return [robot_x, robot_y]

    def publish_transform(self, robot_position):
        # Create and set the TransformStamped message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "marker"
        transform.child_frame_id = "odom"
        transform.transform.translation.x = - robot_position.translation.x
        transform.transform.translation.y = - robot_position.translation.y
        transform.transform.translation.z = 0.0  # Assuming 2D movement
        # transform.transform.rotation.x = - robot_position.rotation.x  # Set rotation if needed
        # transform.transform.rotation.y = - robot_position.rotation.y  
        # transform.transform.rotation.z = - robot_position.rotation.z 
        # transform.transform.rotation.w = 1.0
        self.get_logger().info(f'marker to odom X: {transform.transform.translation.x}')
        self.get_logger().info(f'marker to odom Y: {transform.transform.translation.y}')
        # Publish the transform
        self.static_broadcaster.sendTransform(transform)  


def main(args=None):
    rclpy.init(args=args)
    node = MarkerLcalization()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()