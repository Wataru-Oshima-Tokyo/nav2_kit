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
        self.pose_array = []
        use_sim_time_ = self.get_parameter("use_sim_time").get_parameter_value().bool_value 
        self.get_logger().info(f"use_sim_time {use_sim_time_}")
        if use_sim_time_:
            self.get_logger().info('USE SIM TIME ')
            self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])
        else:
            self.get_logger().info('Do not USE SIM TIME ')

        # Subscribe to the marker detection topic
        self.marker_position_on_map = []
        self.marker_position_on_map.append(None)
        self.marker_position_on_map.append(None)

        self.timer1 = self.create_timer(1, self.lookup_transform)  # 0.01 seconds = 10ms = 100Hz
        self.timer2 = self.create_timer(0.1, self.run)  # 0.1 seconds = 10ms = 10Hz
        self.transform_fetched = False
        self.initial_transform = False


    def lookup_transform(self):
        if not self.transform_fetched:  # Check if transform is not fetched yet
            try:
                # Get the transform from "map" to "marker"
                transform = self.tf_buffer.lookup_transform('map', 'marker', rclpy.time.Time())
                self.get_logger().info('Map to marker Transform obtained successfully!')
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
            self.pose_array.append(transform)
            if len(self.pose_array) < 50:
                self.get_logger().info('The numbe of pose array is : %d' %len(self.pose_array) )

            if self.transform_fetched and not self.initial_transform and len(self.pose_array) > 50:
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

        # Extract x and y translations from pose_array
        x_translations = sorted([pose.transform.translation.x for pose in self.pose_array])
        y_translations = sorted([pose.transform.translation.y for pose in self.pose_array])


        # Compute median values
        median_x = (x_translations[24] + x_translations[25]) / 2
        median_y = (y_translations[24] + y_translations[25]) / 2

        # Create and set the TransformStamped message using median values
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "marker"
        transform.child_frame_id = "odom"
        transform.transform.translation.x = - median_x
        transform.transform.translation.y = - median_y
        transform.transform.translation.z = 0.0  # Assuming 2D movement
        # Set rotation if needed
        # transform.transform.rotation.x = ...
        # transform.transform.rotation.y = ...
        # transform.transform.rotation.z = ...
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
