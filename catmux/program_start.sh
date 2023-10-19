RCS_SERVER_ADDRESS=wataru-orin
ROBOT_NAME=diffbot2
MAP_ID=ts_1st
WORK_SPACE=/humble_ws
CATMUX_SPACE=/humble_ws/src/nav2_kit/catmux/
CATMUX_COMMAND="diffbot_in_ts_1st"
docker=false

# Launch the rcs_client in one gnome-terminal
gnome-terminal -- bash -c "cd $HOME/humble_ws && agilex_humble_here localhost 10"

gnome-terminal -- bash -c "
docker exec -it humble /bin/bash -c '
    source /opt/ros/humble/setup.bash;
    source $WORK_SPACE/install/setup.bash;
    echo $RCS_SERVER_ADDRESS;
    ros2 run rcs_client rcs_client_node --ros-args \
        --param rcs_server_address:=$RCS_SERVER_ADDRESS \
        --param robot_name:=$ROBOT_NAME \
        --param map_id:=$MAP_ID \
        --param camera_name:=camera \
        --remap cmd_vel_topic:=/diff_cont/cmd_vel_unstamped';
"

# Sleep for a brief moment to ensure the first command starts up
sleep 2

# Launch the process_checker in a different gnome-terminal
gnome-terminal -- bash -c "
docker exec -it humble /bin/bash -c '
    source /opt/ros/humble/setup.bash; 
    source $WORK_SPACE/install/setup.bash; 
    ros2 run process_checker process_handler --ros-args \
        --param work_space:=$CATMUX_SPACE \
        --param catmux_command:=$CATMUX_COMMAND \
        --param docker:=$docker';
"
