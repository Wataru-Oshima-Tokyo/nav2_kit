export RCS_SERVER_ADDRESS=wataru-orin
export ROBOT_NAME=diffbot2
export MAP_ID=ts_1st
export WORK_SPACE=/external/
export CATMUX_COMMAND="diffbot_in_ts_1st"
docker=true

# Launch the rcs_client in one gnome-terminal
gnome-terminal -- bash -c "cd humble_ws && agilex_humble_here && echo -e \"export RCS_SERVER_ADDRESS=${RCS_SERVER_ADDRESS}\nexport ROBOT_NAME=${ROBOT_NAME}\nexport MAP_ID=${MAP_ID}\nexport WORK_SPACE=${WORK_SPACE}\nexport CATMUX_COMMAND=${CATMUX_COMMAND}\" >> ~/.bashrc"

gnome-terminal -- bash -c "docker exec -it humble /bin/bash -c 'ros2 run rcs_client rcs_client_node --ros-args --remap camera_name:=camera --remap cmd_vel_topic:=/diff_cont/cmd_vel_unstamped'"

# Sleep for a brief moment to ensure the first command starts up
sleep 2

# Launch the process_checker in a different gnome-terminal
gnome-terminal -- bash -c "docker exec -it humble /bin/bash -c 'ros2 run process_checker process_handler docker:=${docker}'"
