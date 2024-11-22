import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, DeclareLaunchArgument

import yaml

def generate_launch_description():

    # String parameter to find the config file
    config_path = LaunchConfiguration('config_path')
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='config.yaml',
        description='Path to the config file')

    ros_ws = os.environ.get('ROS_WS')
    
    config_path = ros_ws+'/src/config/config_2.yaml'
    
    # read the config file (Need pyYaml >= 5.1)
    # try/catch to handle the case where the file is not found
    try:
        with open(config_path) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
    except FileNotFoundError:
        print("Error: config file not found")


    gz_odom_publisher = Node(
        package='offboard_flocking',
        executable='gz_odom_publisher',
        name='gz_odom_publisher',
        output={'both': 'log'},
        parameters=[
            {"robots_number": config['robots_number']},
            {"robots_model": config['robots_model']}
        ]
    )

    def VAT_neighbor(i):
        return Node(
        package='offboard_flocking',
        executable='VAT_neighbors',
        name='VAT_neighbors_'+str(i),
        output={'both': 'log'},
        parameters=[
            {"config_file": config_path},
            {"robot_id": i},
            {"use_gz_positions": True}
        ]   
    )
    
    # The main ROS2 LaunchDescription object
    ld = LaunchDescription([
        gz_odom_publisher,
        config_path_arg,
    ] + [VAT_neighbor(i) for i in range(1, config['robots_number']+1)])
    
    return ld