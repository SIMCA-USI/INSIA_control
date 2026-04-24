import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    parameters_file_path = '{}/../conf/mutt.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    return LaunchDescription([
        Node(
            package='INSIA_control',
            executable='telemetry_mutt',
            name='VehicleDecoderMUTT',
            parameters=[parameters_file_path],
            namespace='MUTT',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='canadac_mutt',
            parameters=[parameters_file_path],
            namespace='MUTT',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='device_mutt',
            parameters=[parameters_file_path],
            namespace='MUTT',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='control_mutt',
            name='Control_MUTT',
            parameters=[parameters_file_path],
            namespace='MUTT',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='decision_mutt',
            name='Decision',
            parameters=[parameters_file_path],
            namespace='MUTT',
            output='screen',
            remappings=[
                ('PathPlanning', 'WP/Result')
            ],
            emulate_tty=True
        ),
    ])
