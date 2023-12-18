import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.costmap_2d import PyCostmap2D
import tf2_ros
import numpy as np

class ScanToCostmapNode(Node):
    def __init__(self):
        super().__init__('scan_to_costmap_node')
        
        # Parameters
        costmap = PyCostmap2D()
        self.frame_id = "map" # Change as needed
        self.width = costmap.getSizeInCellsX()       # Width of the costmap in cells
        self.height = costmap.getSizeInCellsY()      # Height of the costmap in cells
        self.resolution = costmap.getResolution()  # Resolution of the costmap (m/cell)

        # Publisher for the costmap
        self.costmap_pub = self.create_publisher(OccupancyGrid, 'costmap', 10)

        # Subscriber to the laser scan topic
        self.scan_sub = self.create_subscription(LaserScan, '/fake/scan_for_move', self.scan_callback, 10)

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize the costmap
        self.costmap = OccupancyGrid()
        self.initialize_costmap()

    def initialize_costmap(self):
        self.costmap.header.frame_id = self.frame_id
        self.costmap.info.resolution = self.resolution
        self.costmap.info.width = self.width
        self.costmap.info.height = self.height
        self.costmap.info.origin.position.x = -self.width * self.resolution / 2
        self.costmap.info.origin.position.y = -self.height * self.resolution / 2
        self.costmap.info.origin.position.z = 0
        self.costmap.info.origin.orientation.w = 1
        self.costmap.data = [-1] * (self.width * self.height)

    def scan_callback(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform(self.frame_id, msg.header.frame_id, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().error('TF2 transform failed')
            return

        for angle, distance in enumerate(msg.ranges):
            if msg.range_min < distance < msg.range_max:
                # Calculate the map coordinates
                theta = msg.angle_min + angle * msg.angle_increment
                wx = trans.transform.translation.x + distance * np.cos(theta)
                wy = trans.transform.translation.y + distance * np.sin(theta)

                mx, my = self.world_to_map(wx, wy)
                if mx is not None and my is not None:
                    index = self.get_index(mx, my)
                    self.costmap.data[index] = 100  # Mark as occupied

        self.costmap.header.stamp = self.get_clock().now().to_msg()
        self.costmap_pub.publish(self.costmap)

    def world_to_map(self, wx, wy):
        mx = int((wx - self.costmap.info.origin.position.x) / self.resolution)
        my = int((wy - self.costmap.info.origin.position.y) / self.resolution)
        if 0 <= mx < self.width and 0 <= my < self.height:
            return mx, my
        return None, None

    def get_index(self, mx, my):
        return my * self.width + mx

def main(args=None):
    rclpy.init(args=args)
    node = ScanToCostmapNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
