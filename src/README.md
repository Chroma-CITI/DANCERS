# Robotics and Network Co-Simulator (for FANETS)
 
This project is a joined robotics and network co-simulator for the purpose of research on communication among a fleet of UAVs.


## Project organization

The `src` repository holds the source code of DANCERS. You'll find repertories holdin code of different natures :

### Configuration and Launch files

- **config** : Holds the (`yaml`) configuration files for the experiments made with DANCERS.
- **launch** : Holds the sripts used to launch the co-simulator. You'll find ROS2 launch files and shell scripts in there.
- **multi_exp_launch** : Holds useful scripts to launch multiple simulations automatically.
- **protobuf_msgs** : Holds the definition of the protobuf messages exhanged between the simulators.

### ROS2 packages

Most of the important code of DANCERS is written as C++ ROS2 Nodes. If you are not familiar with ROS2, you can find their documentation [here](https://docs.ros.org/en/humble/index.html) (ROS2 Humble).

- **coordinator** : The code for the coordinator module of DANCERS, that manages the synchronization and message exchange between the simulators.
- **network_simulators/ns-3_sim** : The code for the network part of the simulation, with ns-3.
- **physics_simulators/Gazebo** : The code for the physics part of the simulation, with Gazebo.
- **physics_simulators/MiniDancers** : The code for the physics part of the simulation, with MiniDancers.
- **physics_simulators/RobotSim** : The code for the physics part of the simulation, with RobotSim.
- **dancers_msgs** : A package with the custom ROS2 messages used in DANCERS. 


## Installation and tutorial

The detailed functioning and documentation of the co-simulator can be found on the GitHub [wiki](https://github.com/Chroma-CITI/DANCERS/wiki). 

