#!/bin/bash

session="sim"

config_path='config_minidancers_default.yaml'
[ -n "$1" ] && config_path="$1"
config_path="${config_path/#\~/$HOME}" # to extend "~"

path_to_ros2_ws='~/sim_ws'
[ -n "$2" ] && path_to_ros2_ws="$2"
path_to_ros2_ws="${path_to_ros2_ws/#\~/$HOME}"

config_path=$(python3 $path_to_ros2_ws/src/config/generate_obstacles.py $config_path)

if [ "$config_path" = "RecursionError" ]; then
    echo "Error in generating obstacles: the obstacles are too constrained and overlap, please lower number or size of obstacles."
    exit 1
fi

tmux new-session -d -s $session "ros2 run mini_dancers mini_dancers --ros-args -p config_file:=$config_path -p use_sim_time:=true"
# Uncoment this line if you want to see the output of the modules.
tmux set remain-on-exit on

# Add this between 'ros2 run' and 'package-name' to enable gdb debug on a ros2 node
# --prefix 'gdb -ex run --args'

window=0
tmux rename-window -t $session:$window 'dancers'
# Available ns-3 sim modules : [fake_neighborhood , adhoc_chain_flocking]
tmux split-window -t $session:$window -v -d "ros2 run ns-3_sim ns3_sim_pseudo_routing --ros-args -p config_file:=$config_path -p use_sim_time:=true"
tmux split-window -t $session:$window -h "ros2 run coordinator coordinator --ros-args -p config_file:=$config_path -p use_sim_time:=true"
tmux select-pane -t $session:$window.2
tmux split-window -t $session:$window -h "ros2 run flocking_controller flocking_controller_vat --ros-args -p config_file:=$config_path -p use_sim_time:=true"
tmux split-window -t $session:$window -h "ros2 run rviz2 rviz2 -d ~/.rviz2/visualizer_v0.0.rviz --ros-args -p use_sim_time:=true"


tmux attach-session -t $session -d