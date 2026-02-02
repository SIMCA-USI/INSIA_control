import os
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
from yaml.loader import SafeLoader


def generate_launch_description():
    parameters_file_path = '{}/../conf/JKU_Robot.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
        vehicle_parameters = yaml.load(f, Loader=SafeLoader)
    return LaunchDescription([
        # Car listener node to connect with openpilot
        # Node(
        #     package='car_actuators_controller',
        #     executable='joy_control_node',
        #     output='screen',
        #     emulate_tty=True
        # ),
        # Telemetry node to get data from openpilot
        Node(
            package='INSIA_control',
            executable='telemetry_jku_robot',
            name='VehicleDecoder',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        # Convert joy data to PetConduction
        Node(
            package='INSIA_control',
            executable='joy_transformer_pet',
            name='Joy',
            output='screen',
            emulate_tty=True,
            remappings=[
                ('/'+vehicle_parameters['id_vehicle'] + '/Joy_transformed_Pet',
                 '/' + vehicle_parameters['id_vehicle'] + '/TeleOperacion'),
            ]
        ),
        # Get data from joy and send it into correct topic
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
            remappings=[
                ('/joy', '/' + vehicle_parameters['id_vehicle'] + '/Joy'),
            ]
        ),
        # Accel node to transform Controller msgs to openpilot msgs
        Node(
            package='INSIA_control',
            executable='speed_jku_robot',
            name='Speed',
            parameters=[parameters_file_path],
            output='screen',
            emulate_tty=True
        ),
        # Steering node to transform Controller msgs to openpilot msgs
        Node(
            package='INSIA_control',
            executable='steering_jku_robot',
            name='Steering',
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
