import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters_file_path = '{}/../conf/imiev.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    return LaunchDescription([
        Node(
            package='INSIA_control',
            executable='can',
            name='CAN_6',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='can',
            name='CAN_7',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        # Node(
        #     package='INSIA_control',
        #     executable='epos',
        #     name='EPOS_Volante',
        #     parameters=[parameters_file_path],
        #     output='screen',
        #     emulate_tty=True
        # ),
        # Node(
        #     package='INSIA_control',
        #     executable='canadac',
        #     parameters=[parameters_file_path],
        #     output='screen',
        #     emulate_tty=True
        # ),
        Node(
            package='INSIA_control',
            executable='vehicledecoder',
            name='ImievDecoder',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
    ])
