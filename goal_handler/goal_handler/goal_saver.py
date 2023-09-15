import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
import yaml
import os
import datetime

class GoalSaverNode(Node):
    def __init__(self):
        super().__init__('goal_saver_node')
        self.goals = []

        self.waipoint_subscription = self.create_subscription(
            MarkerArray,
            '/waypoints',
            self.goal_callback,
            10
        )

        self.declare_parameter('output_file', '/')
            # Write the goals to a YAML file
        current_date_time = datetime.datetime.now().strftime('%Y-%m-%d')
        base_filename = self.get_parameter('output_file').get_parameter_value().string_value
        self.output_file = f"{base_filename}{current_date_time}_goals.yaml"
        directory = os.path.dirname(self.output_file)
        if not os.path.exists(directory):
            os.makedirs(directory)
        self.id = 0

    def goal_callback(self, msg: MarkerArray):
        # self.get_logger().info(f'Received goal: {msg.pose.position.x}, {msg.pose.position.y}')
        
        # # Append the goal to the goals list
        # self.goals.append({
        #     'position': {
        #         'x': msg.pose.position.x,
        #         'y': msg.pose.position.y,
        #         'z': msg.pose.position.z
        #     },
        #     'orientation': {
        #         'x': msg.pose.orientation.x,
        #         'y': msg.pose.orientation.y,
        #         'z': msg.pose.orientation.z,
        #         'w': msg.pose.orientation.w
        #     }
        # })
        # for marker in msg.markers:
        pose = msg.markers[len(msg.markers)-1].pose
        if pose.position.x != 0 and pose.position.y != 0 and pose.position.z != 0:
            self.get_logger().info('Received marker pose: position [x: %f, y: %f, z: %f], orientation [x: %f, y: %f, z: %f, w: %f]' % (
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

            # You can store the poses in a list or any other data structure
            # Here I'm appending to a list as an example:
            self.goals.append({
                'id': {
                    'id': self.id
                },
                'position': {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z
                },
                'orientation': {
                    'x': pose.orientation.x,
                    'y': pose.orientation.y,
                    'z': pose.orientation.z,
                    'w': pose.orientation.w
                }
            })


            # Write the goals to a YAML file
            with open(self.output_file, 'w') as outfile:
                yaml.dump({'goals': self.goals}, outfile)
            self.id +=1
        else:
            self.get_logger().info("no coordinate available")

def main(args=None):
    rclpy.init(args=args)
    node = GoalSaverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
