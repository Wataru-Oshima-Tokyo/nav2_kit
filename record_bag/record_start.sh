#!/bin/bash

# Get current timestamp
timestamp=$(date +"%Y%m%d_%H%M%S")

# Define the bag name
bag_name="aksk_$timestamp"

# Run the ROS2 bag record command
ros2 bag record -o $bag_name /tf /tf_static /map

