import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters_file_path = '{}/../conf/imiev.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    return LaunchDescription([
        Node(
            package='INSIA_control',
            executable='io_card',
            name='io_card',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
    ])
