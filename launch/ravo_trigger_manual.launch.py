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
            executable='manual_ravo',
            name='TriggerManual',
            namespace='ravo',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
    ])
