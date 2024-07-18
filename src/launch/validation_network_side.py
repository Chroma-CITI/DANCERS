# ROS2 Launch program that starts a co-simulation of Gazebo and NS-3 along with the Gazebo GUI.
# It sets-up a virtual network infrastructure based on the file setup_network.bash and launches PX4-Autopilot instances.

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessStart, OnProcessIO, OnExecutionComplete, OnProcessExit, OnShutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, LocalSubstitution
from launch.conditions import IfCondition

import yaml


def generate_launch_description():
    ws_path = '/home/theotime/simulation_ws'
    config_path = ws_path + '/src/config/evaluation.yaml'

    # copy all the environment variables of the user
    node_env = os.environ.copy()

    # read the config file (Need pyYaml >= 5.1)
    with open(config_path) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    # Boolean parameter to launch the Gazebo GUI or not
    use_gz_gui = LaunchConfiguration('use_gz_gui', default=True)
    use_gz_gui_arg = DeclareLaunchArgument(
        'use_gz_gui',
        default_value=use_gz_gui,
        description='If true, launch the Gazebo GUI')

    # ROS2 Action: run a bash script as sudo to configure the virtual network (require sudo)
    setup_network = ExecuteProcess(
        cmd=['sudo', ws_path + '/src/launch/setup_network.bash', str(config['robots_number'])],
        name='setup_network'
    )

    # Sets the environment variable GZ_IP, needed to communicate with Gazebo server from different namespaces (Gazebo's transport library)
    # [config] bridge_ip: IP of the bridge providing direct communication with the network namespaces
    set_GZ_IP = SetEnvironmentVariable(
        name='GZ_IP',
        value=[str(config['bridge_ip'])]
    )
    
    set_FASTRTPS_DEFAULT_PROFILE_FILE = SetEnvironmentVariable(
        name='FASTRTPS_DEFAULT_PROFILE_FILE',
        value=[ws_path + '/src/config/profile_simulation_coms.xml']
    )
    
    coordinator = Node(
        package='coordinator',
        executable='coordinator',
        name='coord',
        parameters=[
            {"config_file": config_path}
        ]
    )

    # ROS2 Node: Launch the robotics coordinator node.
    # This c++ file is a modified version of the robotics coordinator from the ROS-NetSim framework
    # The Gazebo server (i.e. the robotics simulator) is started by this Node, thus the name "gazebo"
    robotic_coordinator = Node(
            package='robotics_coordinator',
            executable='robotics_coord',
            name='gazebo',
            parameters=[
                {"config_file": config_path},
                {"verbose": False}
            ]
        )

    # ROS2 Node: Launch the network simulator node.
    network_simulator = Node(
            package='ns-3_sim_ros',
            executable='ns-3_sim_adhoc_no_tap',
            name='ns3',
            parameters=[
                {"config_file": config_path},
                {"verbose": False}
            ]
        )

    # ROS2 Action: Runs a command to launch an empty Gazebo GUI. It connects automatically to a running Gazebo Server 
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '>', '/dev/null'],
        name='gazebo_gui',
        condition=IfCondition(LaunchConfiguration('use_gz_gui'))
    )
    
    def gz_clock_bridge(i):
        return ExecuteProcess(
            cmd=['netns-exec', 'net'+str(i), 'bash', '-c', 'source install/setup.bash && unset FASTRTPS_DEFAULT_PROFILES_FILE && ROS_DOMAIN_ID='+ str(10+i) +' ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock --ros-args -r __node:=clock_bridge_'+str(i)],
            name='clock_bridge_'+str(i),
            env=node_env
        )


    # ROS2 Action: Runs a script that destroys the virtual network infrastructure 
    # Require sudo mode
    # Note: we wrap this script in a function in a clumsy way because of a bug in ROS2 launch system: 
    # it appears the OnShutDown event handler doesn't work with any action other than LogInfo() 
    # see https://robotics.stackexchange.com/questions/24908/how-to-execute-a-script-at-shutdown-of-a-launch-process
    def remove_network(event, context):
        os.system('sudo '+ws_path+'/src/launch/remove_network.bash '+str(config['robots_number']))
        os.system('pkill -2 parameter_bridg')
        os.system('pkill -3 robotics_coord')
        return [
            LogInfo(msg=['Launch was asked to shutdown: ', LocalSubstitution('event.reason')])
        ]
    
    # The main ROS2 LaunchDescription object
    ld = LaunchDescription([
        use_gz_gui_arg,
        setup_network,
        set_GZ_IP,
        set_FASTRTPS_DEFAULT_PROFILE_FILE,
        RegisterEventHandler(
            OnProcessExit(
                target_action=setup_network,
                on_exit=[
                    coordinator,
                    network_simulator,
                    robotic_coordinator,
                    gazebo_gui,
                ] 
                # + [gz_clock_bridge(i) for i in range(1, config['robots_number']+1)]
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=remove_network
            )
        )
    ])

    return ld