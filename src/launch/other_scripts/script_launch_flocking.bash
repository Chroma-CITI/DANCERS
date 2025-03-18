#!/bin/bash
uav_num=2
[ -n "$1" ] && uav_num="$1"

i=1
while [ $i -lt $(($uav_num+1)) ]; do
    echo "Starting ros2 flocking node for uav $i"
    netns-exec net$(($i-1)) /bin/bash -c "source /opt/ros/humble/setup.bash; source /home/theotime/simulation_ws/install/setup.bash; ros2 run offboard_flocking VAT_flocking --ros-args -p config_file:=/home/theotime/simulation_ws/src/config/example_config.yaml -p robot_id:=$i >out$i.log 2>err$i.log &"
    i=$(($i+1))
done