#!/bin/bash

# This script removes the network configuration for the co-simulation.

# Must be executed as root (creates network namespaces)
if [ "$USER" != "root" ]
then
    echo "Please run this as root or with sudo"
    exit 2
fi

# Must be given the number of network namespaces to remove as parameter
if [[ $# -ne 1 ]]; then
    echo -e "Illegal number of parameters.\nUsage :\n\t \e[35msudo ./remove_network.bash N\e[0m\n Where N is the number of network namespaces / robots to destroy."
    exit 2
fi

echo "Removing network (netns net[0-N])"

num_netns=2
[ -n "$1" ] && num_netns="$1"

ip link del brm
ip link del tapm

n=1
while [ $n -lt $(($1+1)) ]; do
    ip netns del net$n
    ip link del wifi_br$n
    ip link del wifi_tap$n
    ip link del wifi_vethb$n
    ip link del veths$n
    n=$(($n+1))
done
