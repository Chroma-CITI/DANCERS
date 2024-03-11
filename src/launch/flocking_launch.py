import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
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
    

    ld = LaunchDescription([
        LogInfo(msg='Launching flocking nodes...')
    ])

    for i in range(1, config['robots_number']+1):
        # ld.add_action(flocking_instance(i)) # uncomment if gazebo only
        ld.add_action(flocking_instance_netns(i)) # uncomment if DANCERS

        ld.add_action(record_pose(i))

    return ld