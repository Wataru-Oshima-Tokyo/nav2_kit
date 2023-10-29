RCS_SERVER_ADDRESS=192.168.0.74
ROBOT_NAME=scout_mini_actual
MAP_ID=ts_1st
WORK_SPACE=$HOME/humble_ws
CATMUX_SPACE=$WORK_SPACE/src/nav2_kit/catmux/
CATMUX_COMMAND="diffbot_in_ts_1st"
docker=false
sim=true
# Launch the rcs_client in one gnome-terminal
gnome-terminal -- bash -c "    ros2 run rcs_client rcs_client_node --ros-args \
        --param rcs_server_address:=$RCS_SERVER_ADDRESS \
        --param robot_name:=$ROBOT_NAME \
        --param map_id:=$MAP_ID \
        --param camera_name:=camera \
        --remap cmd_vel_topic:=/diff_cont/cmd_vel_unstamped;"

# Sleep for a brief moment to ensure the first command starts up
sleep 2

# Launch the process_checker in a different gnome-terminal
gnome-terminal -- bash -c "     ros2 run process_checker process_handler --ros-args \
        --param work_space:=$CATMUX_SPACE \
        --param catmux_command:=$CATMUX_COMMAND \
        --param docker:=$docker \
	--param sim:=$sim;"

