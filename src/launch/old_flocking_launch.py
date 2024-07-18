import os
import sys
import argparse

from launch import LaunchDescription
from launch import LaunchIntrospector
from launch import LaunchService
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.actions import LogInfo

def main(argv=sys.argv[1:]):

    config_path = '/home/theotime/simulation_ws/src/config/example_config.yaml'

    parser = argparse.ArgumentParser(
        description=("Launch a number of nodes performing flocking. Each node has only access to a TUN interface that must be created beforehand.")
    )
    parser.add_argument('--nodes', type=int, default=1, help='Number of nodes to launch')

    args = parser.parse_args(argv)

    ld = LaunchDescription([
        LogInfo(msg="Launching {} flocking nodes".format(args.nodes))
    ])

    for i in range(args.nodes):
        # Set environment variables
        ld.add_action(SetEnvironmentVariable(
            name='CYCLONEDDS_URI',
            value='file:///home/theotime/simulation_ws/src/offboard_flocking/cyclonedds_profiles/config_tun' + str(i) + '.xml'))

        ld.add_action(LogInfo(msg="CYCLONEDDS_URI set to {}".format(os.environ['CYCLONEDDS_URI'])))

        # Start the node
        ld.add_action(Node(
            package='offboard_flocking',
            executable='control_test',
            namespace='flocking' + str(i),
            parameters=[
                {"robot_id": i},
                {"config_file": config_path}
            ],
            output='screen'))

    print(LaunchIntrospector().format_launch_description(ld))

    ls = LaunchService(argv=argv)

    ls.include_launch_description(ld)

    return ls.run()

if __name__ == '__main__':
    sys.exit(main())