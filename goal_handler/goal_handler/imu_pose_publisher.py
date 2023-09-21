import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

class IMUToPoseNode(Node):
    def __init__(self):
        super().__init__('imu_to_pose')
        
        # Publisher
        self.posestamped_pub = self.create_publisher(PoseStamped, 'imu_pose', 10)
        
        # Subscriber
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.last_time = self.get_clock().now().seconds_nanoseconds()
        self.velocity = [0.0, 0.0, 0.0]  # velocity in x, y, z
        self.position = [0.0, 0.0, 0.0]  # position in x, y, z

    def imu_callback(self, imu_msg):
        current_time = self.get_clock().now().seconds_nanoseconds()
        dt = (current_time[0] + current_time[1]*1e-9) - (self.last_time[0] + self.last_time[1]*1e-9)
        
        # Integrate acceleration to get velocity
        self.velocity[0] += (imu_msg.linear_acceleration.x) * dt
        self.velocity[1] += (imu_msg.linear_acceleration.y) * dt
        self.velocity[2] += (imu_msg.linear_acceleration.z-9.85) * dt

        # Integrate velocity to get position
        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt
        self.position[2] += self.velocity[2] * dt

        # Update the PoseStamped message
        ps_msg = PoseStamped()
        ps_msg.header = imu_msg.header
        ps_msg.pose.orientation = imu_msg.orientation
        ps_msg.pose.position.x = self.position[0]
        ps_msg.pose.position.y = self.position[1]
        ps_msg.pose.position.z = self.position[2]
        self.posestamped_pub.publish(ps_msg)

        # Update last time
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)

    imu_to_pose = IMUToPoseNode()

    rclpy.spin(imu_to_pose)

    # Cleanup
    imu_to_pose.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
