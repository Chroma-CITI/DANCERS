#!/bin/bash

# FROM https://stackoverflow.com/questions/5014632/how-can-i-parse-a-yaml-file-from-a-linux-shell-script
# I love people on the internet
function parse_yaml {
   local prefix=$2
   local s='[[:space:]]*' w='[a-zA-Z0-9_]*' fs=$(echo @|tr @ '\034')
   sed -ne "s|^\($s\):|\1|" \
        -e "s|^\($s\)\($w\)$s:$s[\"']\(.*\)[\"']$s\$|\1$fs\2$fs\3|p" \
        -e "s|^\($s\)\($w\)$s:$s\(.*\)$s\$|\1$fs\2$fs\3|p"  $1 |
   awk -F$fs '{
      indent = length($1)/2;
      vname[indent] = $2;
      for (i in vname) {if (i > indent) {delete vname[i]}}
      if (length($3) > 0) {
         vn=""; for (i=0; i<indent; i++) {vn=(vn)(vname[i])("_")}
         printf("%s%s%s=\"%s\"\n", "'$prefix'",vn, $2, $3);
      }
   }'
}


config_name='config_2.yaml'
[ -n "$1" ] && config_name="$1"
config_name="${config_name/#\~/$HOME}"

eval $(parse_yaml $config_name "config_")

session="mission"

tmux kill-session -t $session

# Spawn the PX4 Autopilot instances
tmux new-session -d -s $session "$ROS_WS/src/launch/spawn_px4.bash $config_robots_number $config_robots_model && bash"
# tmux set remain-on-exit on

window=0
tmux rename-window -t $session:$window 'mission'
# Launch the ROS2 that controls the robots
tmux split-window -t $session:$window -v -d "cd $ROS_WS && read -p 'Press any key when the drones have spawned' && sleep $(($config_robots_number+3)) && ros2 launch multirobot_control N_VAT_neighbors.launch.py config_path:=$config_name"

tmux attach-session -t $session:$window -d