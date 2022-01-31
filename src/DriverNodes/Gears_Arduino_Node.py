import os

import rclpy
import yaml
from insia_msg.msg import CANGroup, StringStamped, FloatStamped, BoolStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader

from src.utils.utils import make_can_msg
from traceback import format_exc


class CANADAC_Node(Node):
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

        self.gear_value = {
            'M': make_can_msg(node=self.cobid, data=0x0000200),
            'N': make_can_msg(node=self.cobid, data=0x00000100),
            'D1': make_can_msg(node=self.cobid, data=0x01010100),
            'D2': make_can_msg(node=self.cobid, data=0x02010100),
            'DA1': make_can_msg(node=self.cobid, data=0x03010100),
            'DA2': make_can_msg(node=self.cobid, data=0x04010100),
            'R1': make_can_msg(node=self.cobid, data=0x01020100),
            'R2': make_can_msg(node=self.cobid, data=0x02020100),
            'RA1': make_can_msg(node=self.cobid, data=0x03020100),
            'RA2': make_can_msg(node=self.cobid, data=0x04020100),
            'P1': make_can_msg(node=self.cobid, data=0x01030100),
            'P2': make_can_msg(node=self.cobid, data=0x02030100),
            'PA1': make_can_msg(node=self.cobid, data=0x03030100),
            'PA2': make_can_msg(node=self.cobid, data=0x4030100)
        }

        self.pub_heartbit = self.create_publisher(msg_type=StringStamped,
                                                  topic='/' + vehicle_parameters['id_vehicle'] + '/Heartbit',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_CAN = self.create_publisher(msg_type=CANGroup,
                                             topic='/' + vehicle_parameters['id_vehicle'] + '/' + self.can_connected,
                                             qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=StringStamped,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/' + self.get_name() + '/Consigna',
                                 callback=self.consigna, qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbit = self.create_timer(1, self.publish_heartbit)

    def consigna(self, data):
        if data.data in self.gear_value.keys():
            self.publish_gear(data.data)
        else:
            self.logger.error(f' Gear selected not valid {data.data}')

    def publish_gear(self, gear):
        msg = self.gear_value.get(gear)
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
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
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = CANADAC_Node()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print('Arduino Marchas: Keyboard interrupt')
    except Exception as e:
        format_exc()
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
