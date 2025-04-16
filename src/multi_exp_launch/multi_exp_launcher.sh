#!/bin/bash

if [[ $# -lt 1 ]]; then
    echo -e "Illegal number of parameters.\nUsage :\n\t \e[35m./src/experiment_scripts/multi_exp_launcher.sh <config_folder>\e[0m\n Where <config_folder> is the name of the folder containing all the configuration files for this experiment, contained in src/config/"
    exit 2
fi

config_folder='exp_scalability'
[ -n "$1" ] && config_folder="$1"
config_folder="${config_folder/#\~/$HOME}"

networks_pkg='ns-3_sim'
networks_node='wifi_flocking'

physics_pkg='flocking_controller'
physics_node='flocking_controller_vat'

meta_counter=0
read -sp "Number of complete runs (a complete run is the execution of all the configuration files in the folder): " n_expe
echo \n
while [ $meta_counter -lt $n_expe ]; do
    files=$(find $config_folder -maxdepth 1 -type f -name "*.yaml" | sort -V)
    counter=1
    while IFS= read -r file; do
        echo Launching $file:

        gnome-terminal --wait -- ./src/launch/tmux_scripts/start_dancers_minidancers.sh $file $networks_pkg $networks_node $physics_pkg $physics_node                  # for minidancers
        # gnome-terminal --wait -- ./src/launch/tmux_scripts/start_dancers_viragh.sh $path_to_viragh $path_to_ros2_ws $file    # for robotsim
        # gnome-terminal --wait -- ./src/launch/tmux_scripts/start_dancers_gazebo.sh $file                                     # for gazebo

        echo Finished $file, sleeping for 3 seconds
        sleep 3
        ((counter++))
    done <<< "$files"
    ((meta_counter++))
done
echo Finished all experiments


# find $config_folder -maxdepth 1 -type f -name "*.yaml" -print0 | sort -h | xargs -0 ./src/launch/start_dancers_viragh.sh $path_to_viragh $path_to_ros2_ws 

# "Launching {}" \; -exec ./src/launch/start_dancers_viragh.sh $path_to_viragh $path_to_ros2_ws {} \; -exec sleep 5 \;