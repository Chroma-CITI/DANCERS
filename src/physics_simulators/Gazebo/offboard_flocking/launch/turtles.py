from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='offboard_flocking',
            namespace='turtlesim1',
            executable='control_test',
            name='control',
            parameters=[
                {"robot_id": 1},
            ]
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='offboard_flocking',
            namespace='turtlesim2',
            executable='control_test',
            name='control',
            parameters=[
                {"robot_id": 2},
            ]
        )
    ])