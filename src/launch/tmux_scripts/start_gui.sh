#!/bin/bash

session="viz"

window=0
tmux new-session -d -s $session "ros2 run rviz2 rviz2 -d /home/theotime/.rviz2/visualizer_v0.0.rviz"
tmux split-window -t $session:$window -v -d "gz sim -g"

tmux attach-session -t $session:$window -d


