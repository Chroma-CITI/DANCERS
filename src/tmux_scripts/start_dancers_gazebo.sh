#!/bin/bash

session="sim"

path_to_ros2_ws='~/sim_ws'
[ -n "$1" ] && path_to_ros2_ws="$1"
path_to_ros2_ws="${path_to_ros2_ws/#\~/$HOME}"

config_name='config_2.yaml'
[ -n "$2" ] && config_name="$2"
config_name="${config_name/#\~/$HOME}"

config_name=$(python3 $path_to_ros2_ws/src/config/generate_obstacles.py $config_name)

if [ "$config_name" = "RecursionError" ]; then
    echo "Error in generating obstacles: the obstacles are too constrained and overlap, please lower number or size of obstacles."
    exit 1
fi

tmux new-session -d -s $session "cd $path_to_ros2_ws && ros2 run gazebo_sim gazebo_sim --ros-args -p config_file:=$config_name"
tmux set remain-on-exit on

window=0
tmux rename-window -t $session:$window 'dancers'
tmux split-window -t $session:$window -v -d "cd $path_to_ros2_ws && ros2 run ns-3_sim ns-3_sim_wifi_adhoc --ros-args -p config_file:=$config_name"
tmux split-window -t $session:$window -h "cd $path_to_ros2_ws && ros2 run coordinator coordinator --ros-args -p config_file:=$config_name"
tmux select-pane -t $session:$window.2
tmux split-window -t $session:$window -h -d "gz sim -g"

window=1
tmux set remain-on-exit on
tmux new-window -t $session:$window "PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=x500 $HOME/PX4-Autopilot-1.14/build/px4_sitl_default/bin/px4 -i 1"
tmux split-window -t $session:$window -v "PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,2" PX4_GZ_MODEL=x500 $HOME/PX4-Autopilot-1.14/build/px4_sitl_default/bin/px4 -i 2"

window=2
tmux new-window -t $session:$window "cd $path_to_ros2_ws && ros2 launch multirobot_control N_VAT_neighbors.launch.py config_path:=$config_name" 

tmux attach-session -t $session:1 -d