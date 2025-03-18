import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessIO, OnExecutionComplete, OnProcessExit, OnShutdown, OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, LocalSubstitution
from launch.conditions import IfCondition
from pathlib import Path

# This file requires the following environment variables:
# ROS_WS: The path to the workspace

# Launch this file from netns 1 !!

def generate_launch_description():
    
    ws_path = '/home/theotime/simulation_ws'
    
    robot_id = 1 # The id of the robot that will execute the carousel
    station_id = 2 # The id of the station that will send the commands
    interpacket_time = 12000
    experiment_name = '_0222_carousel_spectrum_congestion_'+str(interpacket_time)
    Path(os.environ['ROS_WS']+'/data/carousel_sim/'+experiment_name).mkdir(parents=True, exist_ok=True)

    mission_command = ExecuteProcess(
        cmd=['netns-exec', 'net'+str(station_id), 'zsh', '-c', 'source install/setup.zsh && ros2 run offboard_flocking simple_control --ros-args -p robot_id:='+str(robot_id)+' -p use_sim_time:=false'],
        name='mission_command',
        )
    
    num_run = len(os.listdir(os.environ['ROS_WS']+'/data/carousel_sim/'+str(experiment_name))) + 1
    pose_recorder = Node(
        package='offboard_flocking',
        executable='gz_pos_saver',
        name='pose_recorder',
        output='screen',
        parameters=[
            {"robot_id": robot_id},
            {"robot_model": "px4vision_lidar"},
            {"file_name": "data/carousel_sim/"+str(experiment_name)+"/pose_"+str(num_run)+".csv"},
            {"use_sim_time": False},
        ]
    )
    
    px4_estimator_recorder = ExecuteProcess(
        cmd=['netns-exec', 'net'+str(robot_id), 'zsh', '-c', 'source install/setup.zsh && ros2 run offboard_flocking px4_pos_saver --ros-args -p robot_id:='+str(robot_id)+' -p robot_model:=px4vision_lidar -p file_name:=data/carousel_sim/'+str(experiment_name)+'/px4_estimator_'+str(num_run)+'.csv'],
        name='px4_estimator_recorder',
        output='screen',
    )

    start_congestion = TimerAction(period=30.0, actions=[ExecuteProcess(
        cmd=['netns-exec', 'net'+str(station_id), 'zsh', '-c', '/home/theotime/experiment_ws/networking/trafficGeneration/client 1000 '+str(interpacket_time)+' 10.0.0.'+str(robot_id)+' 1'],
        name='congestion',
        output='screen',
    )], cancel_on_shutdown=True)
    
    stop_congestion = TimerAction(period=60.0, actions=[ExecuteProcess(
        cmd=['netns-exec', 'net'+str(station_id), 'zsh', '-c', 'pkill -9 client'],
        name='stop_congestion',
        output='screen',
    )], cancel_on_shutdown=True)
    
    kill_all = TimerAction(period=80.0, actions=[
        ExecuteProcess(
        cmd=['pkill', '-9', 'simple_control'],
        name='kill_all',
        output='screen',
        ),                                         
        ExecuteProcess(
        cmd=['pkill', '-9', 'gz_pos_saver'],
        name='kill_all',
        output='screen',
        ),
        ], cancel_on_shutdown=True)
    
    # start_tshark = ExecuteProcess(
    #     cmd=['tshark', '-i', 'wifi_veth'+str(robot_id), '-w', 'data/co-sim_validation/'+str(experiment_name)+'/capture.pcap'],
    #     output='screen',
    #     shell=True,
    # )
    
    ld = LaunchDescription([
        mission_command,
        pose_recorder,
        # px4_estimator_recorder,
        start_congestion,
        stop_congestion,
        kill_all
        # start_tshark,
    ])
    
    return ld
