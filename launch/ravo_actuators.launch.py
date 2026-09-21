import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    parameters_file_path = '{}/../conf/ravo.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    namespace = 'ravo'
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('INSIA_control'),
                '/ravo_socketcan.launch.py'])
        ),
        Node(
            package='INSIA_control',
            executable='telemetry_ravo',
            name='Telemetry',
            namespace=namespace,
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True,
            remappings=[
                ('CAN_vehiculo', 'can_Telemetry'),
                ('CAN_control', 'CAN'),
            ]
        ),
        Node(
            package='INSIA_control',
            executable='maxon',
            name='EPOS4_Volante',
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
        Node(
            package='INSIA_control',
            executable='brake_ravo',
            name='Brake',
            namespace=namespace,
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='canadacv3',
            name='CANADAC_Acelerador',
            namespace=namespace,
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='throttle_ravo',
            name='Throttle',
            namespace=namespace,
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='nimbus',
            name='BrushesDriver',
            namespace=namespace,
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='steering_ravo',
            name='Steering',
            namespace=namespace,
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
    ])
