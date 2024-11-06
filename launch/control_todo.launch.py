import os
import launch
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    parameters_file_path = '{}/../conf/imievcan.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    return LaunchDescription([
        ExecuteProcess(
            cmd=['/bin/bash', '-c', '/home/gdsbs/setup_can.sh'],
            output='screen'
        ),
        Node(
            package='INSIA_control',
            executable='driverCANServicio',
            name='can_coche',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='driverCANServicio',
            name='can_control',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='telemetry_imiev_srv',
            name='ImievDecoder',
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
            executable='maxon',
            name='EPOS4_Volante',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='faulhaber',
            name='FAULHABER_Freno',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='brake_imiev',
            name='Brake',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='steering_imiev',
            name='Steering',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='throttle_imiev_new',
            name='Throttle',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='longitudinal_control_simple',
            name='Longitudinal_Control',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='lateral_control_PID_simple',
            name='Lateral_Control',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='decision_low',
            name='Decision',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='INSIA_control',
            executable='pathplanning_basic',
            name='PathPlanning',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('nmea_navsat_driver'),
                '/gps_utm_fix.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('ros2_waypoints'),
                '/launch/waypoints_recorder.launch.py'])
        ),

    ])
