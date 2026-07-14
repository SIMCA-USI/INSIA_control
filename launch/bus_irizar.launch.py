from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2socketcan_bridge',
            executable='ros2socketcan',
            name='ros2socketcan',
            parameters=[{'interface': 'can0'}]
        ),
        Node(
            package='bus_irizar',
            executable='can_node',
            name='can_node'
        ),
        Node(
            package='bus_irizar',
            executable='longitudinal_control_node',
            name='longitudinal_control_node'
        ),
        Node(
            package='bus_irizar',
            executable='throttle_node',
            name='throttle_node'
        ),
        Node(
            package='bus_irizar',
            executable='brake_node_override_test_2',
            name='brake_node'
        )
    
    ])