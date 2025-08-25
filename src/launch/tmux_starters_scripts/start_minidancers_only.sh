#!/bin/bash

session="sim"

config_path='src/config/config_minidancers_default.yaml'
[ -n "$1" ] && config_path="$1"
config_path="${config_path/#\~/$HOME}" # to extend "~"

physics_pkg='flocking_controller'
[ -n "$2" ] && physics_pkg="$2"

physics_node='flocking_controller_vat'
[ -n "$3" ] && physics_node="$3"


# Check if ROS_WS is set (non-empty), otherwise exit
if [ -z "${ROS_WS}" ]; then
    echo "Error:Please set ROS_WS environment variable. It should point to the root of the ROS2 workspace, for example /home/johndoe/sim_ws"
    exit 1
fi

# Auto-generate the obstacles (calls a python script that outputs the path of the configuration file that should be used. 
# Either the original config_path is the obstacles should not be auto-generated, or the path towards the new config path with random obstacles)
config_path=$(python3 $ROS_WS/src/config/generate_obstacles.py $config_path)

if [ "$config_path" = "RecursionError" ]; then
    echo "Error in generating obstacles: the obstacles are too constrained and overlap, please lower number or size of obstacles."
    exit 1
fi


# If there is a rviz configuration file next to the config file and with the same name, we might as well use it.
rviz_to_use="src/config/default_viz.rviz"

rviz_path="${config_path%.yaml}.rviz"
if [ -f "$rviz_path" ]; then
    echo "Found a rviz file that matches the name of the config file, will use it."
    rviz_to_use="$rviz_path"
fi

tmux new-session -d -s $session "ros2 run mini_dancers mini_dancers --ros-args -p config_file:=$config_path -p use_sim_time:=true -p cosim_mode:=false"
# Uncoment this line if you want to see the output of the modules.
tmux set remain-on-exit on

# Add this between 'ros2 run' and 'package-name' to enable gdb debug on a ros2 node
# --prefix 'gdb -ex run --args'

window=0
tmux rename-window -t $session:$window 'mini-dancers'
tmux split-window -t $session:$window -h -d "ros2 run $physics_pkg $physics_node --ros-args -p config_file:=$config_path -p use_sim_time:=true"
tmux split-window -t $session:$window -v "ros2 run rviz2 rviz2 -d $rviz_to_use --ros-args -p use_sim_time:=true"

tmux attach-session -t $session -d