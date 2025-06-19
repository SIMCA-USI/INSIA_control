import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters_file_path = '{}/../conf/ravo.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    namespace = 'RAVO'
    return LaunchDescription([
        Node(
            package='INSIA_control',
            executable='can',
            name='can_control',
            namespace=namespace,
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='brakeCAN',
            name='BrakeCAN',
            namespace=namespace,
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
    ])
