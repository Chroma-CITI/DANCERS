
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
    # Boolean parameter to launch the Gazebo GUI or not
    use_gz_gui = LaunchConfiguration('use_gz_gui', default=True)
    use_gz_gui_arg = DeclareLaunchArgument(
        'use_gz_gui',
        default_value=use_gz_gui,
        description='If true, launch the Gazebo GUI')
    
    # String parameter to find the config file
    config_path = LaunchConfiguration('config_path')
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='config.yaml',
        description='Path to the config file')
    
    ros_ws = os.environ.get('ROS_WS')
    
    # copy all the environment variables of the user
    node_env = os.environ.copy()

    # read the config file (Need pyYaml >= 5.1)
    # try/catch to handle the case where the file is not found
    try:
        with open(config_path.perform(context)) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
    except FileNotFoundError:
        print("Error: config file not found")


    # ROS2 Action: run a bash script as sudo to configure the virtual network (require sudo)
    setup_network = ExecuteProcess(
        cmd=['sudo', ros_ws + '/src/launch/setup_network.bash', str(config['robots_number'])],
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
        value=[ros_ws + '/src/config/fastrtps_whitelist_profile.xml']
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
            package='gazebo_sim',
            executable='gazebo_sim',
            name='gazebo',
            parameters=[
                {"config_file": config_path},
                {"verbose": False}
            ]
        )

    # ROS2 Node: Launch the network simulator node.
    network_simulator = Node(
            package='ns-3_sim',
            executable='ns-3_sim_adhoc',
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

    # TODO : Instead of spawning the drones statically at launch from a script, write a ROS2 node providing a "spawn drone" service that can spawn or remove uavs during runtime
    # ROS2 Action: Runs a bash script to launch N PX4-Autopilot instances from the ~/PX4-Autopilot directory
    # [config] robots_number: Number of px4_Autopilot to launch
    # [config] robots_model: name of the model that will be spawned in Gazebo (a model with this name must exist in PX4-Autopilot/Tools/simulation/gz/models)
    spawn_px4 = ExecuteProcess(
        cmd=[ros_ws + '/src/launch/spawn_px4_netns.bash', str(config['robots_number']), str(config['robots_model'])],
        name='spawn_px4',
        env=node_env
    )

    def obstacle_detector(i):
        obs = Node(
            package='obstacle_detector',
            executable='obstacle_extractor_node',
            name='obstacle_detector'+str(i),
            output={'both': 'log'},
            parameters=[
                {"active": True},
                {"use_scan": True},
                {"use_pcl": False},
                {"use_sim_time": True},
                {"use_split_and_merge": True},
                {"circles_from_visibles": True},
                {"discard_converted_segments": True},
                {"transform_coordinates": True},
                {"min_group_points": 10},
                {"max_group_distance": 0.1},
                {"distance_proportion": 0.00628},
                {"max_split_distance": 0.2},
                {"max_merge_separation": 0.2},
                {"max_merge_spread": 0.2},
                {"max_circle_radius": 0.6},
                {"radius_enlargement": 0.3},
                {"frame_id": config['robots_model']+'_'+str(i)+'/base_link/gpu_lidar'}
            ],
            namespace=config['robots_model']+'_'+str(i),
        ) # This was a tentative to create a node and export its cmd definition with obs.cmd but it didn't work. Rolled back to a classic cmd launch of the node
        return ExecuteProcess(
            cmd=['netns-exec', 'net'+str(i), 'bash', '-c', 'source install/setup.bash && unset FASTRTPS_DEFAULT_PROFILES_FILE && ROS_DOMAIN_ID='+ str(10+i) +' ros2 run obstacle_detector obstacle_extractor_node --ros-args -p use_pcl:=false -p use_sim_time:=true -p min_group_points:=10 -p radius_enlargement:=0.3 -p frame_id:='+config['robots_model']+'_'+str(i)+'/base_link/gpu_lidar'+' -r __ns:=/'+config['robots_model']+'_'+str(i),],
            name='obstacle_detector'+str(i),
            env=node_env
        )

    # Bridges from Gazebo pub/sub to ROS2 pub/sub messages (lidar)
    def ros_gz_bridge_lidar(i):
        gz_topic_name = '/world/multicopter/model/'+config['robots_model']+'_'+str(i)+'/link/base_link/sensor/gpu_lidar/scan'
        node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge_lidar_'+str(i),
            output={'both': 'log'},
            arguments=[
                gz_topic_name+
                '@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
            ],
            ros_arguments=[
                '--remap', gz_topic_name+':='+config['robots_model']+'_'+str(i)+'/scan'
            ]
        )
        return ExecuteProcess(
            cmd=['netns-exec', 'net'+str(i), 'bash', '-c', 'source install/setup.bash && unset FASTRTPS_DEFAULT_PROFILES_FILE && ROS_DOMAIN_ID='+ str(10+i) +' ros2 run ros_gz_bridge parameter_bridge '+gz_topic_name+'@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan --ros-args --remap '+gz_topic_name+':='+config['robots_model']+'_'+str(i)+'/scan'],
            name='ros_gz_bridge_lidar_'+str(i),
            env=node_env
        )
    
    # The odometry bridge must be executed inside the network namespace of each robot
    def ros_gz_bridge_odom(i):
        return ExecuteProcess(
            cmd=['netns-exec', 'net'+str(i), 'bash', '-c', "source install/setup.bash && ros2 run ros_gz_bridge parameter_bridge /model/px4vision_lidar_"+str(i)+"/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry --ros-args --remap /model/px4vision_lidar_"+str(i)+"/odometry:=/px4vision_lidar_"+str(i)+"/odometry"],
            name='ros_gz_bridge_odom_'+str(i),
            env=node_env
        )
    
    def UAV_odom_publisher(i):
        return ExecuteProcess(
            cmd=['netns-exec', 'net'+str(i), 'bash', '-c', 'source install/setup.bash && unset GZ_IP && export ROS_LOCALHOST_ONLY=1 && ros2 run offboard_flocking gz_odom_publisher --ros-args -p robot_id:='+str(i)+' -p robot_model:=px4vision_lidar -r __node:=UAV_odom_publisher_'+str(i)],
            name='uav_odom_publisher_'+str(i),
            env=node_env
        )

    
    def clock_domain_id_bridge(i):
        return ExecuteProcess(
            cmd=['netns-exec', 'net'+str(i), 'bash', '-c', 'source install/setup.bash && unset FASTRTPS_DEFAULT_PROFILES_FILE && ros2 run domain_bridge domain_bridge --to '+ str(10+i) +' src/domain_bridge/examples/clock_bridge_config.yaml'],
            name='clock_domain_id_bridge_'+str(i),
            env=node_env
        )
    
    def gz_clock_bridge(i):
        return ExecuteProcess(
            cmd=['netns-exec', 'net'+str(i), 'bash', '-c', 'source install/setup.bash && unset FASTRTPS_DEFAULT_PROFILES_FILE && ROS_DOMAIN_ID='+ str(10+i) +' ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock --ros-args -r __node:=clock_bridge_'+str(i)],
            name='clock_bridge_'+str(i),
            env=node_env
        )
        
    # static transform frame for lidar position relative to UAV body
    def lidar_static_tf2_broadcaster(i):
        lid = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf2_broadcaster_'+str(i),
            output={'both': 'log'},
            arguments=[
                '--frame-id', config['robots_model']+'_'+str(i), 
                '--child-frame-id', config['robots_model']+'_'+str(i)+'/base_link/gpu_lidar',
                '--x', '0.085',
                '--y', '0.015',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0'
                ]
            )
        return ExecuteProcess(
            cmd=['netns-exec', 'net'+str(i), 'bash', '-c', 'source install/setup.bash && unset FASTRTPS_DEFAULT_PROFILES_FILE && ROS_DOMAIN_ID='+ str(10+i) +' ros2 run tf2_ros static_transform_publisher'+
                ' --frame-id '+config['robots_model']+'_'+str(i)+
                ' --child-frame-id '+config['robots_model']+'_'+str(i)+'/base_link/gpu_lidar'+
                ' --x 0.085'+
                ' --y 0.015'+
                ' --z 0.0'+
                ' --roll 0.0'+
                ' --pitch 0.0'+
                ' --yaw 0.0'
                ],
            name='lidar_static_tf2_broadcaster_'+str(i),
            env=node_env
        )

    # ROS2 Node: Launch the tf2 broadcaster node
    # [config] robot_model: name of the robot model used in Gazebo
    def tf2_broadcaster(i):
        return ExecuteProcess(
            cmd=['netns-exec', 'net'+str(i), 'bash', '-c', "source install/setup.bash && ros2 run tf2_cpp uav_tf2_broadcaster --ros-args -p robot_name:="+config['robots_model']+'_'+str(i)+" -r __node:=tf2_broadcaster_"+str(i)],

            name='tf2_broadcaster_'+str(i),
        )


    # ROS2 Action: Runs a script that destroys the virtual network infrastructure 
    # Require sudo mode
    # Note: we wrap this script in a function in a clumsy way because of a bug in ROS2 launch system: 
    # it appears the OnShutDown event handler doesn't work with any action other than LogInfo() 
    # see https://robotics.stackexchange.com/questions/24908/how-to-execute-a-script-at-shutdown-of-a-launch-process
    def remove_network(event, context):
        os.system('sudo '+ros_ws+'/src/launch/remove_network.bash '+str(config['robots_number']))
        os.system('pkill -2 parameter_bridg')
        os.system('pkill -3 gazebo_sim')
        return [
            LogInfo(msg=['Launch was asked to shutdown: ', LocalSubstitution('event.reason')])
        ]
    
    # The main ROS2 LaunchDescription object
    ld = LaunchDescription([
        use_gz_gui_arg,
        config_path_arg,
        setup_network,
        set_GZ_IP,
        set_FASTRTPS_DEFAULT_PROFILE_FILE,
        RegisterEventHandler(
            OnProcessExit(
                target_action=setup_network,
                on_exit=[
                    # network_coordinator,
                    coordinator,
                    network_simulator,
                    robotic_coordinator,
                    gazebo_gui,
                    # clock_bridge,
                ] 
                # + [UAV_odom_publisher(i) for i in range(1, config['robots_number']+1)]
                # + [tf2_broadcaster(i) for i in range(1, config['robots_number']+1)]
                + [ros_gz_bridge_lidar(i) for i in range(1, config['robots_number']+1)]
                + [lidar_static_tf2_broadcaster(i) for i in range(1, config['robots_number']+1)]
                + [obstacle_detector(i) for i in range(1, config['robots_number']+1)]
                + [gz_clock_bridge(i) for i in range(1, config['robots_number']+1)]
            )
        ),
        RegisterEventHandler(
            OnProcessIO(
                target_action=network_simulator,
                on_stdout=lambda info: spawn_px4 if(info.text.decode().strip() == 'Network simulator ready') 
                else LogInfo(
                    msg='ns-3 says "{}"'.format(info.text.decode().strip())
                )
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=remove_network
            )
        )
    ])

    # # Add necessary nodes for each UAV to the launch description
    # for i in range(1, config['robots_number']+1):
    #     ld.add_action(lidar_static_tf2_broadcaster(i))
    #     ld.add_action(ros_gz_bridge_lidar(i))
    #     ld.add_action(obstacle_detector(i))

    return ld