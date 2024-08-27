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

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path=${HOME}/PX4-Autopilot-1.14

build_path=${src_path}/build/px4_sitl_default

echo "killing running instances"
pkill -x px4 || true
pkill -x MicroXRCEAgent || true

sleep 4

echo "starting model $model with airframe $airframe"
export PX4_GZ_MODEL=$model
export PX4_SYS_AUTOSTART=$airframe

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

/usr/local/bin/MicroXRCEAgent udp4 -p 8888 >out_uxrce-dds.log 2>1 &

n=1
while [ $n -lt $(($sitl_num+1)) ]; do

    export PX4_GZ_MODEL_POSE=$( grid_generator $(($n-1)) )
    # export PX4_GZ_MODEL_POSE="$n,$n,$z_init,0,0,0"
    working_dir="$build_path/instance_$n"
    [ ! -d "$working_dir" ] && mkdir -p "$working_dir"

    pushd "$working_dir" &>/dev/null
    echo "starting instance $n in $(pwd)"
    $build_path/bin/px4 -i $n -d $build_path/etc >out.log 2>err.log &
    popd &>/dev/null

	n=$(($n + 1))
done