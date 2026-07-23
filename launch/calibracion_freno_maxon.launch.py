import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    parameters_file_path = '{}/../conf/irizarRework.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    namespace = 'irizarRework'
    return LaunchDescription([
        Node(
            package='INSIA_control',
            executable='maxon',
            name='MCD60_Volante',
            namespace=namespace,
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='cpp_nodes',
            executable='override_freno_EMT',
            name='OverrideFrenoEMTNode',
            namespace=namespace,
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
    ])

