import os

import rclpy
import yaml
from insia_msg.msg import Telemetry, StringStamped
from numpy import interp
from openpilot_msgs.msg import CarState
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from yaml.loader import SafeLoader
from std_msgs.msg import Float64


class VehicleNode(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='VehicleDecoder', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        self.id_plataforma = vehicle_parameters['id_vehicle']
        self.steering_sensor_error = vehicle_parameters['steering']['sensor_error']
        self.steering_sensor_inverted = vehicle_parameters['steering']['inverted']
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False
        self.telemetry = Telemetry(id_plataforma=self.id_plataforma)

        self.create_subscription(msg_type=Float64, topic='/robot/motors/steering/state',
                                 callback=self.steering_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=Float64, topic='/robot/motors/propulsion/state', callback=self.speed_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_telemetry = self.create_publisher(msg_type=Telemetry, topic='Telemetry',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def steering_callback(self, data: Float64):
        self.telemetry.steering = (data.data - self.steering_sensor_error) * (
            -1 if self.steering_sensor_inverted else 1)
        self.pub_telemetry.publish(self.telemetry)

    def speed_callback(self, data: Float64):
        self.telemetry.speed = (data.data / 100) * 3.6
        self.pub_telemetry.publish(self.telemetry)

    def publish_heartbeat(self):
        """
        Heartbeat publisher to keep tracking every node
        :return: Publish on Heartbeat
        """
        msg = StringStamped(
            data=self.get_name()
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_heartbeat.publish(msg)

    def shutdown(self):
        try:
            self.shutdown_flag = True
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = VehicleNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
