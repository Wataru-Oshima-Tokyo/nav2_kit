export RCS_SERVER_ADDRESS=wataru-orin
export ROBOT_NAME=diffbot2
export MAP_ID=ts_1st
export WORK_SPACE=$HOME/humble_ws
export CATMUX_COMMAND="diffbot_in_ts_1st"
docker=false

# Launch the rcs_client in one gnome-terminal
gnome-terminal -- bash -c "ros2 run rcs_client rcs_client_node --ros-args --remap camera_name:=camera --ros-args --remap cmd_vel_topic:=/diff_cont/cmd_vel_unstamped"

# Sleep for a brief moment to ensure the first command starts up
sleep 2

# Launch the process_checker in a different gnome-terminal
gnome-terminal -- bash -c "ros2 run process_checker process_handler --ros-args --remap docker:=${docker}; exec bash"