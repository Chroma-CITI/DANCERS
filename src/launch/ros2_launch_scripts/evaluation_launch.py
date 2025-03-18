import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessStart, OnProcessIO, OnExecutionComplete, OnProcessExit, OnShutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, LocalSubstitution
from launch.conditions import IfCondition
from pathlib import Path

def generate_launch_description():
    
    robot_id = 1
    base_station_id = 2
    experiment_name = 'ns3_net_side_2'
    Path(os.environ['ROS_WS']+'/data/co-sim_validation/'+experiment_name).mkdir(parents=True, exist_ok=True)

    mission_receiver = Node(
        package='simple_pubsub',
        executable='mission_starter_udp_receiver',
        name='mission_receiver',
        output='screen',
        parameters=[
            {"robot_id": robot_id},
            {"use_sim_time": False}, # True for co-simulator, False for ns-3 only
            {"experiment_name": experiment_name},
        ]
    )
    
    udp_sender = ExecuteProcess(
        cmd=['netns-exec', 'net'+str(base_station_id), 'zsh', '-c', 'source install/setup.zsh && unset ROS_DOMAIN_ID && ros2 run simple_pubsub udp_sender --ros-args \
            -p use_sim_time:=false -p freq:=10 -p experiment_name:='+str(experiment_name)], # True for co-simulator, False for ns-3 only
        name='udp_sender',
        output='screen',
    )

    pose_recorder = Node(
        package='offboard_flocking',
        executable='gz_pos_saver',
        name='pose_recorder',
        output='screen',
        parameters=[
            {"robot_id": robot_id},
            {"robot_model": "px4vision_lidar"},
            {"file_name": "data/co-sim_validation/"+str(experiment_name)+"/position.csv"},
        ]
    )
    
    start_tshark = ExecuteProcess(
        cmd=['tshark', '-i', 'wifi_veth'+str(robot_id), '-w', 'data/co-sim_validation/'+str(experiment_name)+'/capture.pcap'],
        output='screen',
        shell=True,
    )
    
    ld = LaunchDescription([
        mission_receiver,
        udp_sender,
        pose_recorder,
        start_tshark,
    ])
    
    return ld
