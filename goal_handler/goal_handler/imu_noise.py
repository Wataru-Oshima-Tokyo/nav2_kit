import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class IMUNoiseAndMedianCalculator(Node):
    def __init__(self):
        super().__init__('imu_noise_and_median_calculator')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.z_values_accel = []
        self.z_values_gyro = []
        self.window_size = 50  # Adjust as needed

    def listener_callback(self, msg):
        # Linear acceleration
        z_accel = msg.linear_acceleration.z
        self.z_values_accel.append(z_accel)

        # Angular velocity
        z_gyro = msg.angular_velocity.z
        self.z_values_gyro.append(z_gyro)

        # Keep only the latest 'window_size' elements for both sensors
        if len(self.z_values_accel) > self.window_size:
            self.z_values_accel.pop(0)

        if len(self.z_values_gyro) > self.window_size:
            self.z_values_gyro.pop(0)

        # Calculate and print noise (standard deviation) for both sensors
        if len(self.z_values_accel) == self.window_size and len(self.z_values_gyro) == self.window_size:
            noise_accel = np.std(self.z_values_accel)
            noise_gyro = np.std(self.z_values_gyro)
            self.get_logger().info(f'Noise (Std Dev) for z-value of linear acceleration: {noise_accel}')
            self.get_logger().info(f'Noise (Std Dev) for z-value of angular velocity: {noise_gyro}')


def main(args=None):
    rclpy.init(args=args)
    imu_noise_and_median_calculator = IMUNoiseAndMedianCalculator()
    rclpy.spin(imu_noise_and_median_calculator)
    imu_noise_and_median_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
