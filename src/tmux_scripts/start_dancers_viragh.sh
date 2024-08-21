#!/bin/bash

session="sim"

tmux new-session -d -s $session 'cd ~/gits/flocking-simulator && ./robotflocksim_main -obst obstacles/cosim_obstacles.default'
tmux set remain-on-exit on

window=0
tmux rename-window -t $session:$window 'dancers'
tmux split-window -t $session:$window -v -d "cd ~/simulation_ws && ros2 run ns-3_sim ns-3_sim_adhoc_no_tap --ros-args -p config_file:=/home/theotime/simulation_ws/src/config/config_flocking_4.yaml"
tmux split-window -t $session:$window -h "cd ~/simulation_ws && ros2 run coordinator coordinator --ros-args -p config_file:=/home/theotime/simulation_ws/src/config/config_flocking_4.yaml"
tmux select-pane -t $session:$window.2
tmux split-window -t $session:$window -h "cd ~/simulation_ws && ros2 run viragh_connector viragh_connector --ros-args -p config_file:=/home/theotime/simulation_ws/src/config/config_flocking_4.yaml"

tmux attach-session -t $session -d