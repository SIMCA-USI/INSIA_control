import os
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
from yaml.loader import SafeLoader


def generate_launch_description():
    parameters_file_path = '{}/../conf/emt.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
        vehicle_parameters = yaml.load(f, Loader=SafeLoader)
    return LaunchDescription([
        # Node(
        #     package='INSIA_control',
        #     executable='can',
        #     name='CAN_Control',
        #     parameters=[parameters_file_path],
        #     output='screen',
        #     emulate_tty=True
        # ),
        # Node(
        #     package='car_actuators_controller',
        #     executable='car_state_listener_node',
        # ),
        Node(
            package='INSIA_control',
            executable='joy_transformer_pet',
            name='Joy'
        ),
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
            remappings=[
                ('/joy', '/' + vehicle_parameters['id_vehicle'] + '/Joy'),
            ]
        ),
    ])
