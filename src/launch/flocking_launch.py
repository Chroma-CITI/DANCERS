import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, TimerAction
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    ws_path = '/home/theotime/simulation_ws'
    config_path = ws_path + '/src/config/config_flocking_4.yaml'

    # copy all the environment variables of the user
    node_env = os.environ.copy()

    # read the config file (Need pyYaml >= 5.1)
    with open(config_path) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        
    data_path: str = os.environ['ROS_WS']+'/data/flocking/'+str(config['experience_name'])
    
    Path(data_path).mkdir(parents=True, exist_ok=True)
    num_run = len(os.listdir(data_path)) + 1
    Path(data_path+"/run_"+str(num_run)).mkdir(parents=True, exist_ok=True)

    source_robot_id = config['source_robot_id']
    sink_robot_id = config['sink_robot_id']
    start_traffic_time = config['start_traffic_time'] # s
    stop_traffic_time = config['stop_traffic_time'] # s
    
    def flocking_instance(i):
        return Node(
            package='offboard_flocking',
            executable='VAT_flocking',
            name='vat_flocking_' + str(i),
            parameters=[
                {"config_file": config_path},
                {"robot_id": i},
                {"use_sim_time": False},
            ]
        )
    
    def flocking_instance_netns(i):
        target = "VAT_UDP_flocking"
        return ExecuteProcess(
            cmd=['netns-exec', 'net'+str(i), 'bash', '-c', 'source install/setup.bash && ROS_DOMAIN_ID='+str(10+i)+' ros2 run offboard_flocking '+target+' --ros-args -p config_file:='+config_path+' -p robot_id:='+str(i)+' -p use_sim_time:=true'],
            name='vat_flocking_' + str(i),
        )
    
    def record_pose(robot_id):
        return Node(
            package='offboard_flocking',
            executable='gz_pos_saver',
            name='pose_recorder',
            output='screen',
            parameters=[
                {"robot_id": robot_id},
                {"robot_model": "px4vision_lidar"},
                {"file_name": data_path+"/run_"+str(num_run)+"/pose_robot_"+str(robot_id)+".csv"},
            ]
        )
        
    start_sender = TimerAction(period=start_traffic_time, actions=[ExecuteProcess(
        cmd=['netns-exec', 'net'+str(source_robot_id), 'zsh', '-c', 'source install/setup.zsh && ROS_DOMAIN_ID='+str(10+source_robot_id)+' ros2 run simple_coms udp_sender --ros-args -p experience_name:='+config['experience_name']+' -p dest_ip:=10.0.0.'+str(sink_robot_id)+' -p port:='+str(config['port'])+' -p freq:='+str(config['frequency'])+' -p use_sim_time:=true'],
        name='congestion_sender',
        output='screen',
    )], cancel_on_shutdown=True)
    
    start_receiver = TimerAction(period=start_traffic_time, actions=[ExecuteProcess(
        cmd=['netns-exec', 'net'+str(sink_robot_id), 'zsh', '-c', 'source install/setup.zsh && ROS_DOMAIN_ID='+str(10+sink_robot_id)+' ros2 run simple_coms udp_receiver --ros-args -p experience_name:='+str(config['experience_name'])+' -p port:='+str(config['port'])+' -p use_sim_time:=true'],
        name='congestion_receiver',
        output='screen',
    )], cancel_on_shutdown=True)
    
    stop_congestion = TimerAction(period=stop_traffic_time, actions=[
        ExecuteProcess(
            cmd=['pkill', '-9', 'udp_sender'],
            name='kill_all',
            output='screen',
        ),
        ExecuteProcess(
            cmd=['pkill', '-9', 'udp_receiver'],
            name='kill_all',
            output='screen',
        )
        ])

    
    ld = LaunchDescription([
        LogInfo(msg='Launching flocking nodes...'),
        start_sender,
        start_receiver,
        stop_congestion
    ])

    for i in range(1, config['robots_number']+1):
        # ld.add_action(flocking_instance(i)) # uncomment if gazebo only
        ld.add_action(flocking_instance_netns(i)) # uncomment if DANCERS

        ld.add_action(record_pose(i))

    return ld