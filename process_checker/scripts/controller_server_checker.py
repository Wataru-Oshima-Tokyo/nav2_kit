import subprocess
import re
import time

def get_lifecycle_state(node_name):
    try:
        result = subprocess.run(['ros2', 'lifecycle', 'get', node_name], capture_output=True, text=True, check=True)
        match = re.search(r'\[(\d+)\]', result.stdout)
        if match:
            state_num = match.group(1)  # Extract the number inside the square brackets
            return state_num
        else:
            print("Failed to parse state number.")
            return None
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {e}")
        return None

node_name = "/controller_server"

while True:
    state_num = get_lifecycle_state(node_name)
    if state_num:
        print(f"The state number of {node_name} is: {state_num}")
    else:
        print(f"Failed to get the state number of {node_name}")
    
    time.sleep(1)  # Sleep for 1 second to achieve 1 Hz frequency

