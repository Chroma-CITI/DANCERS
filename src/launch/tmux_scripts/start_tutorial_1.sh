#!/bin/bash

session="sim"

if [[ $# -gt 5 ]]; then
    echo -e "Illegal number of parameters.\nUsage :\n\t \e[35m./src/launch/tmux_scripts/start_tutorial_1.sh <path_to_config_file> <net_sim_pkg_name> <net_sim_node_name> <controller_pkg_name> <controller_node_name>\e[0m\n\nTo change the physics simulator, you have to change the whole script. If ROS2 nodes refuse to launch, verify that you have source'd your ROS2 ws : \"source install/setup.[bash,zsh]\""
    exit 2
fi

config_path='src/config/tutorials/tutorial_1.yaml'
[ -n "$1" ] && config_path="$1"
config_path="${config_path/#\~/$HOME}" # to extend "~"

networks_pkg='ns-3_sim'
[ -n "$2" ] && networks_pkg="$2"

networks_node='standard_wifi_network'
[ -n "$3" ] && networks_node="$3"

physics_pkg='trajectory_controller'
[ -n "$4" ] && physics_pkg="$4"

physics_node='trajectory_controller_node'
[ -n "$5" ] && physics_node="$5"

# Check if ROS_WS is set (non-empty), otherwise exit
if [ -z "${ROS_WS}" ]; then
    echo "Error: Please set ROS_WS environment variable. It should point to the root of the ROS2 workspace, for example /home/johndoe/sim_ws"
    exit 1
fi

# If there is a rviz configuration file next to the config file and with the same name, we might as well use it.
rviz_to_use="src/config/default_viz.rviz"

rviz_path="${config_path%.yaml}.rviz"
if [ -f "$rviz_path" ]; then
    echo "Found a rviz file that matches the name of the config file, will use it."
    rviz_to_use="$rviz_path"
fi

config_path=$(python3 $ROS_WS/src/config/generate_obstacles.py $config_path)

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
tmux split-window -t $session:$window -v -d "ros2 run $networks_pkg $networks_node --ros-args -p config_file:=$config_path -p use_sim_time:=true"
tmux split-window -t $session:$window -h "ros2 run coordinator coordinator --ros-args -p config_file:=$config_path -p use_sim_time:=true"
tmux select-pane -t $session:$window.2
tmux split-window -t $session:$window -h "ros2 run $physics_pkg $physics_node --ros-args -p config_file:=$config_path -p use_sim_time:=true"
tmux split-window -t $session:$window -h "ros2 run rviz2 rviz2 -d $rviz_to_use --ros-args -p use_sim_time:=true"

tmux attach-session -t $session -d