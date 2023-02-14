import os
import struct
from traceback import format_exc

import rclpy
import yaml
from insia_msg.msg import CANGroup, StringStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader

from INSIA_control.utils.utils import make_can_msg


def load_gears(dictionary, big_endian=False):
    with open(os.getenv('ROS_WS') + '/src/INSIA_control/INSIA_control/diccionarios/' + dictionary) as f:
        dictionary_loaded = yaml.load(f, Loader=yaml.FullLoader)
    if big_endian:
        for item in dictionary_loaded:
            dictionary_loaded.update({item: struct.unpack('>i', struct.pack('<i', dictionary_loaded[item]))[0]})
    return dictionary_loaded


class ArduinoNode(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='Arduino', namespace=vehicle_parameters['id_vehicle'], start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False

        self.cobid = self.get_parameter('cobid').value
        self.can_connected = self.get_parameter('can').value

        try:
            dictionary = self.get_parameter('dictionary').value
            self.gear_value = load_gears(dictionary=dictionary, big_endian=self.get_parameter('dictionary').value)
            self.logger.info(f'Loaded dictionary {dictionary}')
            self.logger.debug(f'Valid gears: {list(self.gear_value.keys())}')
        except Exception as e:
            self.logger.error(f'Exception loading gears: {e}')

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_CAN = self.create_publisher(msg_type=CANGroup, topic=self.can_connected,
                                             qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=StringStamped, topic=self.get_name() + '/Consigna', callback=self.consigna,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def consigna(self, data):
        if data.data in self.gear_value.keys():
            self.publish_gear(data.data)
        else:
            self.logger.error(f' Gear selected not valid {data.data}')

    def publish_gear(self, gear):
        msg = make_can_msg(node=self.cobid, data=self.gear_value.get(gear), clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

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
            self.timer_heartbeat.cancel()
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = ArduinoNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        format_exc()
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
