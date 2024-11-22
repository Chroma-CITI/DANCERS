#!/bin/bash
# run multiple instances of the 'px4' binary, but w/o starting the simulator.
# It assumes px4 is already built, with 'make px4_sitl_default'

# The simulator is expected to send to TCP port 4560+i for i in [0, N-1]
# For example jmavsim can be run like this:
#./Tools/simulation/jmavsim/jmavsim_run.sh -p 4561 -l

# Adapted from PX4-Autopilot/Tools/simulation/

# default number of drones : 2
sitl_num=2
[ -n "$1" ] && sitl_num="$1"

model=x500
[ -n "$2" ] && model="$2"

airframe=4001
[ -n "$3" ] && airframe="$3"

separated_ros_domains=true # set to false if you want to use the same ROS_DOMAIN_ID for all sitl instances
[ -n "$4" ] && separated_ros_domains="$4"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path=${HOME}/PX4-Autopilot-1.15

build_path=${src_path}/build/px4_sitl_default

echo $SCRIPT_DIR
echo "killing running instances"
pkill -x px4 || true
pkill -x MicroXRCEAgent || true

sleep 2

source $src_path/src/modules/simulation/gz_bridge/gz_env.sh.in


export PX4_GZ_MODEL=$model
export PX4_SYS_AUTOSTART=$airframe
export GZ_SIM_RESOURCE_PATH="${src_path}/Tools/simulation/gz/models/:${src_path}/Tools/simulation/gz/models/"
export PX4_GZ_STANDALONE=1
# export ROS_LOCALHOST_ONLY=1
# unset GZ_IP

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


n=1
while [ $n -lt $(($sitl_num+1)) ]; do

    # export PX4_DDS_DOMAIN=$n
    # export FASTRTPS_DEFAULT_PROFILES_FILE=/home/theotime/simulation_ws/src/launch/fastrtps_profiles/domains.xml
    if [ "$separated_ros_domains" == "true" ]; then
        export XRCE_DOMAIN_ID_OVERRIDE=$((10+$n))
        export ROS_DOMAIN_ID=$((10+$n))
    fi
    export PX4_GZ_MODEL_POSE=$( grid_generator $(($n-1)) )
    # export PX4_GZ_MODEL_POSE="$n,$n,$z_init,0,0,0"
    export GZ_IP=11.0.0.$n
    working_dir="$build_path/instance_$n"
    [ ! -d "$working_dir" ] && mkdir -p "$working_dir"

    pushd "$working_dir" &>/dev/null
    echo "starting instance $n in $(pwd) with netns net$n"
    # >out.log 2>err.log
    netns-exec net$n echo $GZ_IP
    echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
    netns-exec net$n $build_path/bin/px4 -i $n -d $build_path/etc >out.log 2>err.log &
    netns-exec net$n /usr/local/bin/MicroXRCEAgent udp4 -p 8888 >out_uxrce-dds.log 2>err_uxrce-dds.log &
    popd &>/dev/null

	n=$(($n + 1))
done