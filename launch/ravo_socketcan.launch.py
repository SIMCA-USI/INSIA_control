import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    parameters_file_path = '{}/../conf/socketcan.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    return LaunchDescription([
        Node(
            package='INSIA_control',
            executable='socketcan_driver',
            name='can_control',
            namespace='RAVO',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='socketcan_driver',
            name='can_ravo',
            namespace='RAVO',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('ros2socketcan_bridge'),
                '/launch/can.launch.py'])
        )
    ])
