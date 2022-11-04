import os
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
from yaml.loader import SafeLoader


def generate_launch_description():
    parameters_file_path = '{}/../conf/jku.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
        vehicle_parameters = yaml.load(f, Loader=SafeLoader)
    return LaunchDescription([
        # Telemetry node to get data from openpilot
        Node(
            package='INSIA_control',
            executable='telemetry_jku',
            name='VehicleDecoder',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        # Accel node to transform Controller msgs to openpilot msgs
        Node(
            package='INSIA_control',
            executable='accel_jku',
            name='Speed',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        # Steering node to transform Controller msgs to openpilot msgs
        Node(
            package='INSIA_control',
            executable='steering_jku',
            name='Steering',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        # Lateral control PID
        Node(
            package='INSIA_control',
            executable='lateral_control_PID',
            name='Lateral_Control',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        # Longitudinal control PID
        Node(
            package='INSIA_control',
            executable='longitudinal_control_PID',
            name='Longitudinal_Control',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        # Decision system
        Node(
            package='INSIA_control',
            executable='decision_low',
            name='Decision',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        # PathPlanning system
        Node(
            package='INSIA_control',
            executable='pathplanning_basic',
            name='PathPlanning',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
    ])
