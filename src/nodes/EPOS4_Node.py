import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header

import os
import yaml
from yaml.loader import SafeLoader
import queue
import threading
import struct
import time

from src.utils.connection import Connection
from insia_msg.msg import CAN, CANGroup, StringStamped, EPOSConsigna, EPOSDigital


class EPOS4_Node(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='EPOS4', namespace=vehicle_parameters['id_vehicle'], start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False

        self.cobid = self.get_parameter('cobid').value
        self.msgs_list = self.get_parameter('msgs').value
        self.EPOS_type = self.get_parameter('type').value

        self.pub_heartbit = self.create_publisher(msg_type=StringStamped,
                                                  topic='/' + vehicle_parameters['id_vehicle'] + '/Heartbit',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=CAN,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/CAN',
                                 callback=self.msg_can, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=EPOSConsigna,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/' + self.get_name() + '/Consigna',
                                 callback=self.consigna, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=EPOSDigital,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/' + self.get_name() + '/Digital',
                                 callback=self.digital, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=EPOSDigital,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/' + self.get_name() + '/Enable',
                                 callback=self.enable, qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbit = self.create_timer(1, self.publish_heartbit)

    def enable(self, msg):
        # TODO: Mensaje para cambiar de estado al motor/EPOS
        pass

    def digital(self, msg):
        # TODO: Habilitar salida digital comparando con las salidas existentes
        pass

    def consigna(self, msg):
        # TODO: Enviar consigna de actuacion
        if self.EPOS_type == 'MCD60:':
            pass
        elif self.EPOS_type == 'EPOS4':
            pass

    def msg_can(self, msg):
        if len(self.msgs_list) != 0:
            if msg.cobid in self.msgs_list:
                pass  # decode
        else:
            pass  # decode

    def publish_heartbit(self):
        msg = StringStamped(
            data=self.get_name()
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_heartbit.publish(msg)

    def shutdown(self):
        try:
            self.shutdown_flag = True
            self.timer_heartbit.cancel()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = EPOS4_Node()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print('EPOS: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
