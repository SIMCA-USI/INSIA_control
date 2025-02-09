import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    parameters_file_path = '{}/../conf/emt.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    return LaunchDescription([

        Node(
            package='INSIA_control',
            executable='can',
            name='CAN_Gears',
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
            executable='gears_emt',
            name='Gears',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
    ])
