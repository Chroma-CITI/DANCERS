#!/bin/bash
uav_num=2
[ -n "$1" ] && uav_num="$1"

i=1
while [ $i -lt $(($uav_num+1)) ]; do
    echo "Starting ros2 flocking node for uav $i"
    j=$(($i-1))
    gnome-terminal --tab --title="uav_$i" -- bash -c "source /opt/ros/humble/setup.bash; source $HOME/simulation_ws/install/setup.bash; export CYCLONEDDS_URI=/home/theotime/simulation_ws/src/offboard_flocking/cyclonedds_profiles/config_tun$j.xml; echo $CYCLONEDDS_URI; ros2 launch offboard_flocking control.launch.py robot_id:=$i"
    i=$((i+1))
done

read -p "Press any key to exit"

