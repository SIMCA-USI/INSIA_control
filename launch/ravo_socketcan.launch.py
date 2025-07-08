import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    parameters_file_path = '{}/../conf/socketcan.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    namespace = 'ravo'
    return LaunchDescription([
        Node(
            package='INSIA_control',
            executable='socketcan_driver',
            name='can_control',
            namespace=namespace,
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='socketcan_driver',
            name='can_ravo',
            namespace=namespace,
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='ros2socketcan_bridge',
            executable='ros2socketcan',
            name='can0',
            namespace=namespace,
            parameters=[{'can_interface': 'can0', 'log_level': 50}]
        ),
        Node(
            package='ros2socketcan_bridge',
            executable='ros2socketcan',
            name='can1',
            namespace=namespace,
            parameters=[{'can_interface': 'can1', 'log_level': 50}]
        ),
    ])
