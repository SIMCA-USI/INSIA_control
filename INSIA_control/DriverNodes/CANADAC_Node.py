import os

import rclpy
import yaml
from insia_msg.msg import CANGroup, StringStamped, FloatStamped, BoolStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader

from INSIA_control.utils.utils import make_can_msg
from traceback import format_exc


class CANADACNode(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='CANADAC', namespace=vehicle_parameters['id_vehicle'], start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False

        self.cobid = self.get_parameter('cobid').value
        self.can_connected = self.get_parameter('can').value

        self.pub_heartbit = self.create_publisher(msg_type=StringStamped,
                                                  topic='/' + vehicle_parameters['id_vehicle'] + '/Heartbit',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_CAN = self.create_publisher(msg_type=CANGroup,
                                             topic='/' + vehicle_parameters['id_vehicle'] + '/' + self.can_connected,
                                             qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=FloatStamped,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/' + self.get_name() + '/Target',
                                 callback=self.consigna, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/' + self.get_name() + '/EnableRelay',
                                 callback=self.enable, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped,
                                 topic='/' + vehicle_parameters[
                                     'id_vehicle'] + '/' + self.get_name() + '/EnableTension',
                                 callback=self.enable_tension, qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbit = self.create_timer(1, self.publish_heartbit)

    def enable_tension(self, data):
        if data.data:
            msg = make_can_msg(node=self.cobid, index=0x0002, data=0x01, clock=self.get_clock().now().to_msg())
        else:
            msg = make_can_msg(node=self.cobid, index=0x0002, data=0x00, clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

    def enable(self, data):
        if data.data:
            msg = make_can_msg(node=self.cobid, index=0x0003, data=0x01, clock=self.get_clock().now().to_msg())
        else:
            msg = make_can_msg(node=self.cobid, index=0x0003, data=0x00, clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

    def consigna(self, data):
        msg = make_can_msg(node=self.cobid, index=0x0001, data=int(data.data * 100),
                           clock=self.get_clock().now().to_msg())
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
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = CANADACNode()
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
