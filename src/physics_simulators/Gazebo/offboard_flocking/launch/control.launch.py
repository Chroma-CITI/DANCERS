from launch_ros.actions import Node

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config_path = '/home/theotime/simulation_ws/src/config/example_config.yaml'
    robot_id = LaunchConfiguration('robot_id')

    robot_id_launch_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='1'
    )

    control_node = Node(
        package='offboard_flocking',
        namespace='px4_control',
        executable='VAT_flocking',
        parameters=[
                {"robot_id": robot_id},
                {"config_file": config_path}
            ],
        output='screen'
    )
    return LaunchDescription([
        robot_id_launch_arg,
        control_node
    ])