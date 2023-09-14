import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from rclpy.duration import Duration 
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
import yaml
import time
import math

class MapOriginSetter(Node):

    def __init__(self):
        super().__init__('map_origin_setter')
        
        # Subscribe to initialpose
        self.subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.pose_callback,
            10)
        
        # Client for the map service
        self.declare_parameter('map_file_path', '/map.yaml')
        self.output_file = self.get_parameter('map_file_path').get_parameter_value().string_value      


    def euler_from_quaternion(self, quaternion):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x, y, z, w = quaternion

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


    def pose_callback(self, msg):
        # Read the YAML file
        with open(self.output_file, 'r') as file:
            self.map_yaml = yaml.safe_load(file)

        # Extract the pose from the message and set it to the 'origin' field in the YAML data
        # The pose's position gives the x and y, and the orientation (quaternion) gives the z-angle
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Convert the quaternion orientation to Euler angles to get the yaw (z-angle)
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, yaw = self.euler_from_quaternion(quaternion)  # Assuming you have a function for this conversion

        # Update the origin in the map data
        self.map_yaml['origin'] = [x, y, yaw]

        # Overwrite the YAML file with updated data
        with open(self.output_file, 'w') as file:
            yaml.safe_dump(self.map_yaml, file)

        nav = BasicNavigator()
        nav.changeMap(map_filepath=self.output_file)

def main(args=None):
    rclpy.init(args=args)
    node = MapOriginSetter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
