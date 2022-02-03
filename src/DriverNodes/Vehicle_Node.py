import os

import rclpy
import yaml
from insia_msg.msg import CAN, Telemetry
from insia_msg.msg import StringStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from yaml.loader import SafeLoader

from src.utils.filtro import Decoder
from src.utils.utils import convert_types


class VehicleNode(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='VehicleDecoder', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        self.id_plataforma = vehicle_parameters['id_vehicle']
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False
        self.decoder = Decoder(dictionary=self.get_parameter('dictionary').value)
        self.vehicle_state = {}
        self.pub_heartbit = self.create_publisher(msg_type=StringStamped,
                                                  topic='/' + vehicle_parameters['id_vehicle'] + '/Heartbit',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_telemetry = self.create_publisher(msg_type=Telemetry,
                                                   topic='/' + vehicle_parameters['id_vehicle'] + '/Telemetry',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=CAN,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/CAN',
                                 callback=self.msg_can, qos_profile=HistoryPolicy.KEEP_LAST)
        self.timer_telemetry = self.create_timer(1 / 20, self.publish_telemetry)
        self.timer_heartbit = self.create_timer(1, self.publish_heartbit)

    def create_msg(self):
        msg = Telemetry()
        fields = msg.get_fields_and_field_types()
        for field in fields.keys():
            data = self.vehicle_state.get(field)
            if data is not None:
                setattr(msg, field, convert_types(ros2_type=fields.get(field), data=data))
        msg.id_plataforma = self.id_plataforma
        return msg

    def publish_heartbit(self):
        msg = StringStamped(
            data=self.get_name()
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_heartbit.publish(msg)

    def publish_telemetry(self):
        self.pub_telemetry.publish(msg=self.create_msg())

    def msg_can(self, msg):
        try:
            name, value = self.decoder.decode(msg)
            self.vehicle_state.update({name: value})
            # self.logger.debug(f'Decoded {name}: {value}')
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
    manager = None
    try:
        manager = VehicleNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print('Vehicle Decoder: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
