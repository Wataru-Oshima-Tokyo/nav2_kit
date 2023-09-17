import rclpy
from rclpy.node import Node
import time
class LoggingTestNode(Node):
    def __init__(self):
        super().__init__('logging_test')
        
        self.print_in_blue("This is blue text")
        self.print_in_green("This is green text")
        self.get_logger().info("dsad")
        self.get_logger().warn("dsad")
        self.get_logger().error("dsad")
        self.print_start()


    def print_in_blue(self, msg):
        blue_start = "\033[94m"
        color_end = "\033[0m"
        self.get_logger().info(f"{blue_start}{msg}{color_end}")

    def print_in_green(self, msg):
        green_start = "\033[92m"
        color_end = "\033[0m"
        self.get_logger().info(f"{green_start}{msg}{color_end}")

    def print_in_orange(self, msg):
        orange_start = "\033[93m"  # This is technically light yellow, but it might appear orange-ish in some terminals.
        color_end = "\033[0m"
        self.get_logger().error(f"{orange_start}{msg}{color_end}")

    def print_in_purple(self, msg):
        purple_start = "\033[95m"  # Light magenta, which might look like purple in most terminals.
        color_end = "\033[0m"
        self.get_logger().info(f"{purple_start}{msg}{color_end}")

    def print_start(self):
        for i in range(10):
            if i%3 == 0:
                self.print_in_blue('goal id : ' + '{:d}'.format(i) + ' is reached!')
            elif i%3 == 1:
                self.print_in_green('goal id : ' + '{:d}'.format(i) + ' is reached!')
            else:
                self.print_in_purple('goal id : ' + '{:d}'.format(i) + ' is reached!')

            time.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    node = LoggingTestNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
