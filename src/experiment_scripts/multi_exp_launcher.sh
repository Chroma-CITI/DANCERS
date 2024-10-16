#!/bin/bash

if [[ $# -lt 3 ]]; then
    echo -e "Illegal number of parameters.\nUsage :\n\t \e[35m./src/experiment_scripts/multi_exp_launcher.sh <path_to_robotsim> <path_to_ros2_ws> <config_folder>\e[0m\n Where <config_folder> is the name of the folder containing all the configuration files for this experiment, contained in src/config/"
    exit 2
fi

path_to_viragh='~/flocking-simulator'
[ -n "$1" ] && path_to_viragh="$1"
path_to_viragh="${path_to_viragh/#\~/$HOME}" # (this is for tilde expansion)

path_to_ros2_ws='~/sim_ws'
[ -n "$2" ] && path_to_ros2_ws="$2"
path_to_ros2_ws="${path_to_ros2_ws/#\~/$HOME}"

config_folder='exp_scalability'
[ -n "$3" ] && config_folder="$3"
config_folder="${config_folder/#\~/$HOME}"

meta_counter=0
read -sp "Number of experiments to launch: " n_expe
while [ $meta_counter -lt $n_expe ]; do
    files=$(find $config_folder -maxdepth 1 -type f -name "*.yaml" | sort -h)
    counter=1
    while IFS= read -r file; do
        echo Launching $file:

        gnome-terminal --wait -- ./src/tmux_scripts/start_dancers_minidancers.sh $file $path_to_ros2_ws                 # for minidancers
        # gnome-terminal --wait -- ./src/tmux_scripts/start_dancers_viragh.sh $path_to_viragh $path_to_ros2_ws $file    # for robotsim
        # gnome-terminal --wait -- ./src/tmux_scripts/start_dancers_gazebo.sh $file                                     # for gazebo

        echo Finished $file, sleeping for 3 seconds
        sleep 3
        ((counter++))
    done <<< "$files"
    ((meta_counter++))
done
echo Finished all experiments


# find $config_folder -maxdepth 1 -type f -name "*.yaml" -print0 | sort -h | xargs -0 ./src/tmux_scripts/start_dancers_viragh.sh $path_to_viragh $path_to_ros2_ws 

# "Launching {}" \; -exec ./src/tmux_scripts/start_dancers_viragh.sh $path_to_viragh $path_to_ros2_ws {} \; -exec sleep 5 \;