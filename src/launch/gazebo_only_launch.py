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
    config_path = ws_path + '/src/config/config_flocking_3.yaml'

    # copy all the environment variables of the user
    node_env = os.environ.copy()

    # read the config file (Need pyYaml >= 5.1)
    with open(config_path) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    # ROS2 Action: Runs a command to launch an empty Gazebo GUI. It connects automatically to a running Gazebo Server 
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '>', '/dev/null'],
        name='gazebo_gui'
    )

    robotic_simulator = Node(
            package='gazebo_sim',
            executable='gazebo_only',
            name='gazebo',
            parameters=[
                {"config_file": config_path},
                {"verbose": False}
            ]
        )

    # ROS2 Action: Runs a bash script to launch N PX4-Autopilot instances from the ~/PX4-Autopilot directory
    # [config] robots_number: Number of px4_Autopilot to launch
    # [config] robots_model: name of the model that will be spawned in Gazebo (a model with this name must exist in PX4-Autopilot/Tools/simulation/gz/models)
    spawn_px4 = ExecuteProcess(
        cmd=['/home/theotime/simulation_ws/src/launch/spawn_px4.bash', str(config['robots_number']), str(config['robots_model'])],
        name='spawn_px4',
        env=node_env
    )

    def obstacle_detector(i):
        return Node(
            package='obstacle_detector',
            executable='obstacle_extractor_node',
            name='obstacle_detector'+str(i),
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
        )


    def ros_gz_bridge_lidar(i):
        gz_topic_name = '/world/multicopter/model/'+config['robots_model']+'_'+str(i)+'/link/base_link/sensor/gpu_lidar/scan'
        return Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge_lidar_'+str(i),
            arguments=[
                gz_topic_name+
                '@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
            ],
            ros_arguments=[
                '--remap', gz_topic_name+':='+config['robots_model']+'_'+str(i)+'/scan'
            ]
        )

    def lidar_static_tf2_broadcaster(i):
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf2_broadcaster_'+str(i),
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

    def shutdown(event, context):
        os.system('pkill MicroXRCEAgent')
        return [
            LogInfo(msg=['Launch was asked to shutdown: ', LocalSubstitution('event.reason')])
        ]

    # ROS2 Node: Launch the tf2 broadcaster node
    # [config] robot_model: name of the robot model used in Gazebo
    def tf2_broadcaster(i):
        return Node(
            package='tf2_cpp',
            executable='uav_tf2_broadcaster',
            name='uav_tf2_broadcaster_'+str(i),
            parameters=[
                {'robot_name': config['robots_model']+'_'+str(i)}
            ]
        ) 

    # The main ROS2 LaunchDescription object
    ld = LaunchDescription([
        robotic_simulator,
        gazebo_gui,
        RegisterEventHandler(
            OnProcessIO(
                target_action=robotic_simulator,
                on_stdout=lambda info: [spawn_px4]
                + [ros_gz_bridge_lidar(i) for i in range(1, config['robots_number']+1)]
                + [lidar_static_tf2_broadcaster(i) for i in range(1, config['robots_number']+1)]
                + [obstacle_detector(i) for i in range(1, config['robots_number']+1)] 
                if(info.text.decode().strip().find('Gazebo is ready') != -1) 
                else LogInfo(
                    msg='gazebo says "{}"'.format(info.text.decode().strip())
                )
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=shutdown
            )
        )
    ])

    # Add N tf2_broadcaster nodes to the LaunchDescription
    # for i in range(1, config['robots_number']+1):
    #     ld.add_action(tf2_broadcaster(i))
    #     ld.add_action(lidar_static_tf2_broadcaster(i))
    #     ld.add_action(ros_gz_bridge_lidar(i))
    #     ld.add_action(obstacle_detector(i))
    
    return ld


