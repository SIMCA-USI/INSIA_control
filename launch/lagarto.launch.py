import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters_file_path = '{}/../conf/lagarto.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    return LaunchDescription([
        # Node(
        #     package='INSIA_control',
        #     executable='can',
        #     name='CAN_Vehiculo',
        #     parameters=[parameters_file_path],
        #     output='screen',
        #     emulate_tty=True
        # ),
        Node(
            package='INSIA_control',
            executable='can',
            name='CAN_Control',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='maxon',
            name='EPOS_Volante',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='maxon',
            name='MCD60_Freno',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='canadac',
            name='CANADAC_Acelerador',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='gears_arduino',
            name='Arduino_Gears',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='gears_arduino',
            name='Arduino_DumpBox',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='vehicledecoder_base',
            name='LagartoDecoder',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='brake_lagarto',
            name='Brake',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='steering_lagarto',
            name='Steering',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='throttle_lagarto',
            name='Throttle',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='gears_lagarto',
            name='Gears',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='dumpbox_lagarto',
            name='DumpBox',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        # Node(
        #     package='INSIA_control',
        #     executable='longitudinal_control',
        #     name='Longitudinal_Control',
        #     parameters=[parameters_file_path],
        #     output='screen',
        #     emulate_tty=True
        # ),
        # Node(
        #     package='INSIA_control',
        #     executable='lateral_control_PID',
        #     name='Lateral_Control',
        #     parameters=[parameters_file_path],
        #     output='screen',
        #     emulate_tty=True
        # ),
    ])
