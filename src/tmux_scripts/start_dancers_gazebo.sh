#!/bin/bash

session="sim"

config_name='config_2.yaml'
[ -n "$1" ] && config_name="$1"
config_name="${config_name/#\~/$HOME}"

path_to_ros2_ws=$ROS_WS

tmux kill-session -t $session

config_name=$(python3 $path_to_ros2_ws/src/config/generate_obstacles.py $config_name)

# Grid generator : from an initial position, number of columns and distance between points, generate a 3D position 
# call it with grid_generator(i) where i is the index of the point in the grid
num_columns=2
distance=2 # meters
x_init=0
y_init=$(echo "-(($num_columns-1) * $distance)/2" | bc -l ) # use bc to handle float-point arithmetics in bash
z_init="0.5"
grid_generator(){
    x=$(( $x_init + $1/$num_columns * $distance ))
    y=$(echo "$y_init + $distance * ($1 % $num_columns)" | bc)
    z=$z_init

    echo "-$x,$y,$z,0,0,0,0"
}


if [ "$config_name" = "RecursionError" ]; then
    echo "Error in generating obstacles: the obstacles are too constrained and overlap, please lower number or size of obstacles."
    exit 1
fi

tmux new-session -d -s $session "cd $path_to_ros2_ws && ros2 run gazebo_sim gazebo_sim --ros-args -p config_file:=$config_name -p cosim_mode:=true"
tmux set remain-on-exit on

window=0
tmux rename-window -t $session:$window 'dancers'
tmux split-window -t $session:$window -v -d "cd $path_to_ros2_ws && ros2 run ns-3_sim adhoc_chain_flocking --ros-args -p config_file:=$config_name -p cosim_mode:=true"
tmux split-window -t $session:$window -h "cd $path_to_ros2_ws && ros2 run coordinator coordinator --ros-args -p config_file:=$config_name"
tmux select-pane -t $session:$window.2
tmux split-window -t $session:$window -h -d "gz sim -g"

gnome-terminal -- ./src/tmux_scripts/start_px4_mission.sh $config_name
pid_term=$!

# window=1
# tmux set remain-on-exit on
# tmux new-window -t $session:$window "PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=x500 $HOME/PX4-Autopilot-1.14/build/px4_sitl_default/bin/px4 -i 1"
# n=2
# while [ $n -lt $((4+1)) ]; do
#     tmux -c "PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="$( grid_generator $(($n-1)) )" PX4_GZ_MODEL=x500 $HOME/PX4-Autopilot-1.14/build/px4_sitl_default/bin/px4 -i $n &"
# done

# window=2
# tmux new-window -t $session:$window "cd $path_to_ros2_ws && ros2 launch multirobot_control N_VAT_neighbors.launch.py config_path:=$config_name" 

tmux attach-session -t $session:0 -d

tmux kill-session -t 'mission'