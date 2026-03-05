import os

import rclpy
import yaml
from insia_msg.msg import CAN, Telemetry
from insia_msg.msg import StringStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from yaml.loader import SafeLoader

from INSIA_control.utils.filtro import Decoder
from INSIA_control.utils.utils import convert_types


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
        self.logger.set_level(10)
        self.shutdown_flag = False
        self.decoder = Decoder(dictionary=self.get_parameter('dictionary').value)
        self.vehicle_state = {}

        self.create_subscription(msg_type=CAN, topic='CAN', callback=self.msg_can, qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_telemetry = self.create_publisher(msg_type=Telemetry, topic='Telemetry',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_telemetry = self.create_timer(1 / 20, self.publish_telemetry)
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def create_msg(self):
        msg = Telemetry()
        fields = msg.get_fields_and_field_types()
        for field in fields.keys():
            data = self.vehicle_state.get(field)
            if data is not None:
                setattr(msg, field, convert_types(ros2_type=fields.get(field), data=data))
        msg.id_plataforma = self.id_plataforma
        msg.brake = int((int(msg.brake / 0.25) & 0x0FFF) * 0.25)
        msg.steering = (msg.steering - self.steering_sensor_error)
        if self.steering_sensor_inverted:
            msg.steering *= -1
        return msg

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

    def publish_telemetry(self):
        self.pub_telemetry.publish(msg=self.create_msg())

    def msg_can(self, msg):
        try:
            name, value = self.decoder.decode(msg)
            self.vehicle_state.update({name: value})
            self.logger.debug(f'Decoded {name}: {value} {msg}')
        except ValueError as e:
            self.logger.debug(f'{e}')

    def shutdown(self):
        try:
            self.shutdown_flag = True
            self.timer_telemetry.cancel()
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    # manager = None
    try:
        manager = VehicleNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(e)
    # finally:
    #     manager.shutdown()


if __name__ == '__main__':
    main()
