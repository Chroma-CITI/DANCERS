#!/bin/bash

session="sim"
path_to_viragh='~/flocking-simulator'
[ -n "$1" ] && path_to_viragh="$1"
path_to_viragh="${path_to_viragh/#\~/$HOME}" # (this is for tilde expansion)

path_to_ros2_ws='~/sim_ws'
[ -n "$2" ] && path_to_ros2_ws="$2"
path_to_ros2_ws="${path_to_ros2_ws/#\~/$HOME}"

config_path='config_robotsim_default.yaml'
[ -n "$3" ] && config_path="$3"
config_path="${config_path/#\~/$HOME}"

config_path=$(python3 $path_to_ros2_ws/src/config/generate_obstacles.py $config_path)

if [ "$config_path" = "RecursionError" ]; then
    echo "Error in generating obstacles: the obstacles are too constrained and overlap, please lower number or size of obstacles."
    exit 1
fi

tmux new-session -d -s $session "cd $path_to_viragh &&  ./robotflocksim_main -obst obstacles/cosim_obstacles.default"
tmux set remain-on-exit on

window=0
tmux rename-window -t $session:$window 'dancers'
tmux split-window -t $session:$window -v -d "cd $path_to_ros2_ws && ros2 run ns-3_sim ns3_sim_pseudo_routing --ros-args -p config_file:=$config_path -p use_sim_time:=true"
tmux split-window -t $session:$window -h "cd $path_to_ros2_ws && ros2 run coordinator coordinator --ros-args -p config_file:=$config_path -p use_sim_time:=true"
tmux select-pane -t $session:$window.2
tmux split-window -t $session:$window -h "cd $path_to_ros2_ws && ros2 run robotsim_connector robotsim_connector --ros-args -p config_file:=$config_path -p viragh_path:=$path_to_viragh -p use_sim_time:=true"

tmux attach-session -t $session -d