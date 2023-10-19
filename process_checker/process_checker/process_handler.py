import os
import sys
import signal
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess
import time
from .library.color_print import COLOR_PRINT

class NodeMonitor(Node):
    def __init__(self):
        super().__init__('node_monitor')
        self.timer = self.create_timer(2.0, self.check_node_status)  # Check node status every 2 seconds
        self.killall_service = self.create_service(Trigger, 'killall', self.handle_killall)  # Create the killall service
        self.start_all_service = self.create_service(Trigger, 'restart', self.handle_startall)  # Create the restart service
        self.catmux_dict = os.getenv("WORK_SPACE")
        self.catmux_command = os.getenv("CATMUX_COMMAND")
        self.declare_parameter('docker', False)
        self.declare_parameter('work_space', None)
        self.declare_parameter('catmux_command', "diffbot_in_ts_1st")
        self.docker = self.get_parameter("docker").get_parameter_value().bool_value 
        self.get_logger().info(f"docker {self.docker}")
        if self.catmux_dict is None:
            self.catmux_dict = self.get_parameter('work_space').value
        if self.catmux_command is None:
            self.catmux_command = self.get_parameter('catmux_command').value
        self.color_print = COLOR_PRINT(self)
        self.color_print.print_in_blue(f"work_space {self.catmux_dict}")
        self.color_print.print_in_blue(f"catmux_command {self.catmux_command}")

        valid_dict = True
        if self.catmux_dict is None:
            self.color_print.print_in_red('Please specify the working space as WORK_SPACE') 
            valid_dict = False
        elif not (os.path.exists(self.catmux_dict) and os.path.isdir(self.catmux_dict)):
            self.color_print.print_in_red('Please put a valid working space') 
            valid_dict = False
        if not valid_dict:
            self.destroy_node()
            sys.exit()
        # self.catmux_dict = self.catmux_dict +"/src/nav2_kit/catmux/"
        if os.path.exists(self.catmux_dict) and os.path.isdir(self.catmux_dict):
            self.color_print.print_in_green("YOU CAN HANDLE CATMUX COMMAND")
        self.started = False



            
    def discover_nodes(self):
        node_names_and_namespaces = self.get_node_names_and_namespaces()
        self.get_logger().info('Discovered nodes: %s' % node_names_and_namespaces)
        return node_names_and_namespaces




    def check_node_status(self):
        # This is a simplified check. In reality, you'd want to check if each node is actually responsive.
        node_names_and_namespaces = self.discover_nodes()
        self.get_logger().info('All nodes are running')

    def kill_node(self, pid):
        try:
            os.kill(pid, signal.SIGTERM)
            self.get_logger().info(f'Successfully killed node with PID: {pid}')
        except ProcessLookupError:
            self.get_logger().error(f'Failed to kill node with PID: {pid}')

    
    def startAll(self):
            """
            Open a new gnome-terminal, change to the specified directory, and run the command.

            Args:
            - directory (str): The directory to navigate to.
            - session_name (str): The name for the catmux session.
            """
            # if self.docker:
            #     command = f"gnome-terminal -- bash -c \"docker exec -it humble /bin/bash -c 'cd {self.catmux_dict} && catmux_create_session {self.catmux_command}.yaml;'\""
            # else:
            command = f'gnome-terminal -- bash -c "cd {self.catmux_dict} && catmux_create_session {self.catmux_command}.yaml;"'
            subprocess.run(command, shell=True)


    def handle_startall(self, request, response):
        try:
            if self.started:
                self.killAll()
                time.sleep(2)
            self.startAll()
            response.success = True
            self.get_logger().info("Start All the nodes")
            self.started = True
        except subprocess.CalledProcessError:
            response.success = False
            response.message = "Failed to kill all ROS2 nodes."
        return response

    def killAll(self):
        try:
            self.color_print.print_in_brown("First kill")
            cmd = "ps aux | grep ros |grep -v grep | grep -v process_handler |grep -v rcs_client_node | awk '{ print \"kill -9\", $2 }' | sh"
            subprocess.run(cmd, shell=True, check=True)
            self.color_print.print_in_brown("Second kill")
            cmd = "ps aux | grep catmux | grep -v grep | grep -v process_handler | awk '{ print \"kill -9\", $2 }' | sh"
            time.sleep(2)
            subprocess.run(cmd, shell=True, check=True)
            self.get_logger().info("All nodes except for 'process_handler' and 'rcs client' have been killed.")
        except subprocess.CalledProcessError:
            self.color_print.print_in_yellow("failed to execute it")
            

    def handle_killall(self, request, response):
        if self.killAll():
            response.success = True
        else:
            response.success = False
            response.message = "Failed to kill all ROS2 nodes."
        return response

    def get_pid_of_node(self, node_name):
        cmd = ['pgrep',  f'{node_name}']
        result = subprocess.run(cmd, stdout=subprocess.PIPE)
        try:
            return int(result.stdout.decode('utf-8').strip())
        except ValueError:
            return None


def main():
    rclpy.init()
    node_monitor = NodeMonitor()
    rclpy.spin(node_monitor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
