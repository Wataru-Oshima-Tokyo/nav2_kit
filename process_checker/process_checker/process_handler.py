import os
import sys
import signal
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess
import time
from .library.color_print import COLOR_PRINT
from techshare_ros_pkg2.srv import SendMsg
from rclpy.executors import SingleThreadedExecutor

class NodeMonitor(Node):
    def __init__(self):
        super().__init__('node_monitor')
        self.timer = self.create_timer(2.0, self.check_node_status)  # Check node status every 2 seconds
        self.killall_service = self.create_service(Trigger, 'killall', self.handle_killall)  # Create the killall service
        self.start_all_service = self.create_service(Trigger, 'restart', self.handle_startall)  # Create the restart service
        self.rcs_send_msg_service = self.create_client(SendMsg, "send_msg")
        self.catmux_dict = os.getenv("WORK_SPACE")
        self.catmux_command = os.getenv("CATMUX_COMMAND")
        self.declare_parameter('docker', False)
        self.declare_parameter('imu_check', True)
        self.declare_parameter('sim', False)
        self.declare_parameter('work_space', None)
        self.declare_parameter('catmux_command', "diffbot_in_ts_1st")
        self.docker = self.get_parameter("docker").get_parameter_value().bool_value 
        self.imu_check = self.get_parameter("imu_check").get_parameter_value().bool_value 
        self.sim = self.get_parameter("sim").get_parameter_value().bool_value 
        self.get_logger().info(f"docker {self.docker}")
        self.get_logger().info(f"imu_check {self.imu_check}")
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
        self.initial_success = False
        self.now = time.time()

    def isPassedTime(self, duration):
        if (time.time() - self.now) > duration:
            return True
        else:
            return False


            
    def discover_nodes(self):
        node_names_and_namespaces = self.get_node_names_and_namespaces()
        # self.get_logger().info('Discovered nodes: %s' % node_names_and_namespaces)
        if self.started:
            checkNodes = self.checkSomeNodes(node_names_and_namespaces)
            if self.isPassedTime(7) and checkNodes and not self.isPassedTime(10) and not self.initial_success:
                req = SendMsg.Request()
                req.message = "Succeded to run all the nodes"
                req.error = False
                self.rcs_send_msg_service.call_async(req)
                self.initial_success = True
            if self.isPassedTime(10) and checkNodes and not self.is_all_nodes_ros_alive(node_names_and_namespaces):
                req = SendMsg.Request()
                req.message = "Failed to find witmotion node. Restarting soon"
                req.error = True
                self.rcs_send_msg_service.call_async(req)
                time.sleep(1)
                self.startAction()  


        return node_names_and_namespaces


    def checkSomeNodes(self, nodes):
        map_server = False
        dlio = False
        emcl2 = False
        for node_name, namespace in nodes:
            if 'emcl2' in node_name:
                emcl2=True
            if 'dlio_map_node' in node_name:  # Adjust this check as per the exact name of your node
                dlio=True
            if 'map_server' in node_name:  # Adjust this check as per the exact name of your node
                map_server=True
        if emcl2 and dlio and map_server:
            return True
        return False


    def is_all_nodes_ros_alive(self, nodes):
        if self.sim or not self.imu_check:
            return True
        witmotion = False
        dlio = False
        for node_name, namespace in nodes:
            if 'witmotion' in node_name:
                witmotion=True
            if 'dlio_map_node' in node_name:  # Adjust this check as per the exact name of your node
                dlio=True
        if dlio and witmotion:
            return True
        return False




    def check_node_status(self):
        # This is a simplified check. In reality, you'd want to check if each node is actually responsive.
        node_names_and_namespaces = self.discover_nodes()
        self.color_print.print_in_blue(node_names_and_namespaces)
        self.get_logger().info('All nodes are running')


    
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
        req = SendMsg.Request()
        req.message = "Starting all the processes"
        req.error = True
        self.rcs_send_msg_service.call_async(req)
        command = f'gnome-terminal -- bash -c "cd {self.catmux_dict} && catmux_create_session {self.catmux_command}.yaml;"'
        subprocess.run(command, shell=True)
        self.started = True
        self.now = time.time()

    def startAction(self):
        if self.started:
            self.killAll()
            time.sleep(5)
        self.startAll()

    def handle_startall(self, request, response):
        try:
            self.startAction()
            response.success = True
            self.get_logger().info("Start All the nodes")
        except subprocess.CalledProcessError:
            response.success = False
            response.message = "Failed to kill all ROS2 nodes."
        return response
    
    def killAll(self):
        req = SendMsg.Request()
        req.message = "Trying to kill all processes"
        req.error = True
        self.rcs_send_msg_service.call_async(req)

        # Function to kill processes based on a command
        def kill_processes(cmd):
            pids = subprocess.check_output(cmd, shell=True).decode().split()
            for pid in pids:
                try:
                    os.kill(int(pid), signal.SIGKILL)
                except OSError as e:
                    self.color_print.print_in_yellow(f"Failed to kill process {pid}: {e}")
        
        def check_nodes(nodes):
            # Flags to check if specific nodes are running
            rcs_client_node_running = False
            node_monitor_running = False

            for node_name, namespace in nodes:
                # Check for specific nodes
                if 'rcs_client_node' in node_name:
                    rcs_client_node_running = True
                elif 'node_monitor' in node_name:
                    node_monitor_running = True
                else:
                    # If any other node is found, return False
                    self.color_print.print_in_yellow(nodes)
                    return False

            # Return True only if both specific nodes are running and no others
            return rcs_client_node_running and node_monitor_running
        # Kill ROS related processes
        cmd_ros = "ps aux | grep ros | grep -v grep | grep -v process_handler | grep -v detect_simple_server | grep -v robot_control | grep -v rcs_client_node | awk '{ print $2 }'"
        kill_processes(cmd_ros)

        # Kill catmux related processes
        cmd_catmux = "ps aux | grep catmux | grep -v grep | grep -v process_handler | awk '{ print $2 }'"
        kill_processes(cmd_catmux)

        # Polling to check if the nodes are still running
        max_attempts = 120
        attempt = 0
        while attempt < max_attempts:
            running_nodes = check_nodes(self.get_node_names_and_namespaces())
            if running_nodes:  # If no nodes are running
                break
            time.sleep(1)  # Wait for 1 second before checking again
            attempt += 1

        if attempt == max_attempts:
            self.color_print.print_in_yellow("Some nodes might still be running after multiple attempts.")
        else:
            self.color_print.print_in_brown("All nodes have been successfully killed.")
            req.message = "Killed all the processes"
            req.error = False
            self.rcs_send_msg_service.call_async(req)
            self.started = False
            self.initial_success = False
    # def killAll(self):
    #     req = SendMsg.Request()
    #     req.message = "Trying to kill all the processes"
    #     req.error = True
    #     self.rcs_send_msg_service.call_async(req)
    #     try:
    #         # Kill ROS related processes
    #         cmd_ros = "ps aux | grep ros | grep -v grep | grep -v process_handler | grep -v rcs_client_node | awk '{ print $2 }'"
    #         ros_pids = subprocess.check_output(cmd_ros, shell=True).decode().split()
    #         for pid in ros_pids:
    #             self.color_print.print_in_blue("Killing processes...")
    #             os.kill(int(pid), signal.SIGKILL)

    #         # Kill catmux related processes
    #         cmd_catmux = "ps aux | grep catmux | grep -v grep | grep -v process_handler | awk '{ print $2 }'"
    #         catmux_pids = subprocess.check_output(cmd_catmux, shell=True).decode().split()
    #         for pid in catmux_pids:
    #             self.color_print.print_in_blue("Killing catmux...")
    #             os.kill(int(pid), signal.SIGKILL)

    #         time.sleep(2)  # Wait for processes to terminate

    #         # Check if all processes are killed
    #         if not ros_pids and not catmux_pids:
    #             self.color_print.print_in_brown("All nodes except for 'process_handler' and 'rcs client' have been killed.")
    #             self.started = False
    #             req.message = "Killed all the processes"
    #             req.error = False
    #             self.rcs_send_msg_service.call_async(req)
    #             self.initial_success = False
    #         else:
    #             self.color_print.print_in_yellow("Some processes might still be running.")
            
    #     except subprocess.CalledProcessError:
    #         self.color_print.print_in_yellow("Failed to execute the process kill command")

            

    def handle_killall(self, request, response):
        if self.killAll():
            response.success = True
        else:
            response.success = False
            response.message = "Failed to kill all ROS2 nodes."
        return response


def main():
    rclpy.init()
    node_monitor = NodeMonitor()
    rclpy.spin(node_monitor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

