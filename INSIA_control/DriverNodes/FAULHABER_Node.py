import importlib
import os

import networkx as nx
import rclpy
import yaml
from insia_msg.msg import CAN, CANGroup, StringStamped, EPOSConsigna, EPOSDigital, BoolStamped, IntStamped, EPOSAnalog
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader

from INSIA_control.utils.filtro import Decoder
from INSIA_control.utils.utils import make_can_msg
import INSIA_control.utils.FAULHABER as FAULHABER
from time import sleep


class FAULHABERNode(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='MCD60', namespace=vehicle_parameters['id_vehicle'], start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False
        self.cobid = self.get_parameter('cobid').value
        self.can_conected = self.get_parameter('can').value

        #self.decoder = Decoder(dictionary=self.get_parameter('dictionary').value, cobid=self.cobid)

        self.pub_heartbit = self.create_publisher(msg_type=StringStamped,
                                                  topic='/' + vehicle_parameters['id_vehicle'] + '/Heartbit',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_CAN = self.create_publisher(msg_type=CANGroup,
                                             topic='/' + vehicle_parameters['id_vehicle'] + '/' + str(
                                                 self.can_conected), qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=EPOSConsigna,
                                 topic='/' + vehicle_parameters[
                                     'id_vehicle'] + '/' + self.get_name() + '/TargetPosition',
                                 callback=self.target_position_update, qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbit = self.create_timer(1, self.publish_heartbit)

        sleep(1)
        self.init_device()

    def init_device(self):
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=FAULHABER.init_device(node=self.cobid)
        ))

    def target_position_update(self, msg):
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=FAULHABER.set_angle_value(node=self.cobid, angle=msg.position, absolute=msg.mode)
        ))

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
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = FAULHABERNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print('EPOS4: Keyboard interrupt')
    except Exception as e:
        print(f'{e}')
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
