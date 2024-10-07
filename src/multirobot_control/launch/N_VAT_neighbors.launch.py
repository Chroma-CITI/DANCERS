import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction

import yaml

def launch_setup(context, *args, **kwargs):
    
    config_lc = LaunchConfiguration("config_path")
    config_path_value = config_lc.perform(context)

    # read the config file (Need pyYaml >= 5.1)
    # try/catch to handle the case where the file is not found
    try:
        with open(config_path_value) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
    except FileNotFoundError:
        print("Error: config file not found")
        exit(1)
        
    nodes = []
        
    gz_odom_publisher = Node(
        package='multirobot_control',
        executable='gz_odom_publisher',
        name='gz_odom_publisher',
        output={'both': 'log'},
        parameters=[
            {"robots_number": config['robots_number']},
            {"robots_model": config['robots_model']}
        ]
    )
    nodes.append(gz_odom_publisher)

    def VAT_neighbor(i):
        return Node(
        package='multirobot_control',
        executable='VAT_neighbors',
        name='VAT_neighbors_'+str(i),
        output={'both': 'log'},
        parameters=[
            {"config_file": config_path_value},
            {"robot_id": i},
            {"use_gz_positions": True}
        ]   
    )
    
    for i in range(1, config['robots_number']+1):
        nodes.append(VAT_neighbor(i))    
    
    return nodes

def generate_launch_description():

    # String parameter to find the config file
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='config.yaml',
        description='Path to the config file')      
        
    get_config_file_function = OpaqueFunction(function=launch_setup)
            
    # The main ROS2 LaunchDescription object
    ld = LaunchDescription([
        config_path_arg,
        get_config_file_function,
    ])
    
    return ld