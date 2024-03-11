#!/bin/bash

# This script sets up the network configuration for co-simulation.

# /-------------net0--------------\	/-------------net1--------------\
# |  veth0          wifi_veth0	  |	|  veth1	         wifi_veth1	|
# |  11.0.0.1/24	10.0.0.1/9	  |	|  11.0.0.2/24	    10.0.0.4/24	|
# \----||-------------||----------/	\----||---------------||--------/
#      ||		      ||		         ||               ||
# /----||-------------||---------host----||---------------||--------\
# |  veths0	          ||		       veths1			  ||        |
# |    |___________________brm___________|			  wifi_vethb1   |
# |		              ||   11.0.0.100				      |         |
# |		          wifi_vethb1					      wifi_br1      |
# |				  wifi_br1			                      |         |
# |				  wifi_tap1			                  wifi_tap1     |
# \-----------------------------------------------------------------/

# Must be executed as root (creates network namespaces)
if [ "$USER" != "root" ]
then
    echo "Please run this as root or with sudo"
    exit 2
fi

if [[ $# -ne 1 ]]; then
    echo -e "Illegal number of parameters.\nUsage :\n\t \e[35msudo ./setup_network.bash N\e[0m\n Where N is the number of expected network namespaces / robots."
    exit 2
fi

num_netns=2
[ -n "$1" ] && num_netns="$1"


# ------> This part is commented out because we disable direct communication between network namespaces. 
# ------- Each namespace can communicate with the main namespace via its veth pair but MUST use the wifi between each other
ip tuntap add tapm mode tap
ip link set dev tapm up
ip link add brm type bridge

ip link set tapm master brm

ip addr add 11.0.0.100/24 dev brm
ip link set brm up

n=1
while [ $n -lt $(($1+1)) ]; do
    ip netns add net$n

# Create the direct communication link (that will not go through the network simulator)
# Happens on network 11.0.0.0/24
    ip link add veth$n netns net$n type veth peer name veths$n
    ip link set veths$n up
    ip link set veths$n master brm
    netns-exec net$n sudo ip addr add 11.0.0.$n/24 dev veth$n
    netns-exec net$n sudo ip link set veth$n up
    # Add iptables rules to reject traffic from other network namespaces (on the direct link)
    p=1
    while [ $p -lt $(($1+1)) ]; do
        if [ $p -ne $n ]; then
            # netns-exec net$n sudo iptables -A INPUT -s 11.0.0.$(($p+1)) -j REJECT
            netns-exec net$n sudo iptables -A OUTPUT -d 11.0.0.$p -j REJECT
            
            netns-exec net$n sudo iptables -P INPUT DROP
            netns-exec net$n sudo iptables -A INPUT -i wifi_veth$n -m iprange --src-range 10.0.0.1-10.0.0.128 -j ACCEPT
            netns-exec net$n sudo iptables -A INPUT -i veth$n -s 11.0.0.100 -j ACCEPT
            netns-exec net$n sudo iptables -A INPUT -i lo -j ACCEPT


            # ceinture ET bretelles 
            # netns-exec net$n sudo iptables -A INPUT -s 10.0.0.$(($p*3)) -j REJECT
            # netns-exec net$n sudo iptables -A OUTPUT -d 10.0.0.$(($p*3)) -j REJECT
            # netns-exec net$n sudo iptables -A INPUT -s 10.0.0.$(($p*3+2)) -j REJECT
            # netns-exec net$n sudo iptables -A OUTPUT -d 10.0.0.$(($p*3+2)) -j REJECT
            # there should not be trafic from or towards ghost nodes
        fi
        p=$(($p+1))
    done

# Create the simulated wifi interface 
    ip link add wifi_veth$n netns net$n type veth peer name wifi_vethb$n
    ip link add name wifi_br$n type bridge
    ip link set wifi_vethb$n master wifi_br$n
    ip addr add 10.0.0.$(($n+192))/24 dev wifi_br$n
    ip link set wifi_br$n up
    ip link set wifi_vethb$n up
    netns-exec net$n sudo ip link set lo up
    netns-exec net$n sudo ip addr add 10.0.0.$(($n))/24 dev wifi_veth$n
    netns-exec net$n sudo ip link set wifi_veth$n up
    # netns-exec net$n sudo ip route add default via 10.0.0.$(($n+1))

    # Create TAP interface
    ip tuntap add dev wifi_tap$n mode tap
    ip addr flush dev wifi_tap$n
    ip addr add 10.0.0.$(($n+128))/24 dev wifi_tap$n
    ip link set wifi_tap$n master wifi_br$n
    ip link set wifi_tap$n up

    n=$(($n+1))
done