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
            executable='telemetry_ravo',
            name='Telemetry',
            namespace=namespace,
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
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
    ])
