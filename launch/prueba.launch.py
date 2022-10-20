import os

import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from yaml.loader import SafeLoader


def generate_launch_description():
    with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
        vehicle_name = yaml.load(f, Loader=SafeLoader)['id_vehicle']

    parameters_file_path = '{}/../conf/{}.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))), vehicle_name)
    print(f'Parameters file path used: {parameters_file_path}')
    print(f'Param file {parameters_file_path.split("/")[-1]}')
    if not os.path.isfile(parameters_file_path):
        print(f'Params file is not correct')
        exit(0)
    return LaunchDescription([
        Node(
            package='INSIA_control',
            executable='decision',
            name='Decision',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
    ])
