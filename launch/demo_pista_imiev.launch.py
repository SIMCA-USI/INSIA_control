import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    parameters_file_path = '{}/../conf/imiev.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('INSIA_control'),
                '/imiev_new.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros2_waypoints'), 'launch'),
                '/waypoints_recorder.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('ros2_sensors'),
                '/nmea_trimble.launch.py'])
        ),
    ])
