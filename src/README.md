# Robotics and Network Co-Simulator (for FANETS)
 
This project is a joined robotics and network co-simulator for the purpose of research on communication among a fleet of UAVs.


## About
A *co-simulator* uses two or more existing and mature simulators to build a larger simulator, able to get the better of the two worlds.

Our co-simulator is based on the state-of-the-art, widely used simulators [Gazebo](https://gazebosim.org/home) and [NS-3](https://www.nsnam.org/). The first is a complete and powerful 3D robotic simulator, the second is a realistic network simulator able to faithfully recreate network protocols for many technologies.

We built upon the previous co-simulation work proposed in [ROS-NetSim](https://ieeexplore.ieee.org/document/9345354), which gives a reliable synchronization technique and message passing between the two simulators. The fact that ROS-NetSim is open-source made it easy for us to reuse and modify their proposal, we thank the authors for this choice.

## Installation and tutorial

The detailed functioning and documentation of the co-simulator can be found on the GitHub wiki. 

