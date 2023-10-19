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
        self.work_space = os.getenv("WORK_SPACE")
        self.catmux_cpmmand = os.getenv("CATMUX_COMMAND")
        self.declare_parameter('docker', False)
        self.docker = self.get_parameter("docker").get_parameter_value().bool_value 
        self.get_logger().info(f"docker {self.docker}")
        # if use_sim_time_:
        #     self.get_logger().info('USE SIM TIME ')
        #     self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])
        # else:
        #     self.get_logger().info('Do not USE SIM TIME ')

        self.color_print = COLOR_PRINT(self)
        valid_dict = True
        if self.work_space is None:
            self.color_print.print_in_red('Please specify the working space as WORK_SPACE') 
            valid_dict = False
        elif not (os.path.exists(self.work_space) and os.path.isdir(self.work_space)):
            self.color_print.print_in_red('Please put a valid working space') 
            valid_dict = False
        if not valid_dict:
            self.destroy_node()
            sys.exit()
        self.catmux_dict = self.work_space +"/src/nav2_kit/catmux/"
        if os.path.exists(self.catmux_dict) and os.path.isdir(self.catmux_dict):
            self.color_print.print_in_green("YOu CAN HANDLE CATMUX COMMAND")
        else:
            self.color_print.print_in_yellow("CANNOT FIND THE CATMUX DICT. Please make sure if nav2_kit is directory under <your workspace>/src")



            
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
            if self.docker:
                command = f'gnome-terminal -- bash -c "docker exec -it humble /bin/bash; cd {self.catmux_dict} && catmux_create_session {self.catmux_cpmmand}.yaml;"'
            else:
                command = f'gnome-terminal -- bash -c "cd {self.catmux_dict} && catmux_create_session {self.catmux_cpmmand}.yaml;"'
            subprocess.run(command, shell=True)


    def handle_startall(self, request, response):
        try:
            self.killAll()
            time.sleep(2)
            self.startAll()
            response.success = True
            self.get_logger().info("Start All the nodes")
        except subprocess.CalledProcessError:
            response.success = False
            response.message = "Failed to kill all ROS2 nodes."
        return response

    def killAll(self):
        try:
            cmd = "ps aux | grep ros |grep -v grep | grep -v process_checker |grep -v rcs_client_node | awk '{ print \"kill -9\", $2 }' | sh"
            subprocess.run(cmd, shell=True, check=True)
            cmd = "ps aux | grep catmux | grep -v grep | awk '{ print \"kill -9\", $2 }' | sh"
            time.sleep(2)
            subprocess.run(cmd, shell=True, check=True)
            self.get_logger().info("All nodes except for 'process_checker' and 'rcs client' have been killed.")
        except subprocess.CalledProcessError:
            response.success = False
            

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

