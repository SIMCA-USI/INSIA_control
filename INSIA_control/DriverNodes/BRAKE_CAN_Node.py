import os
from traceback import format_exc

import rclpy
import yaml
from insia_msg.msg import CANGroup, StringStamped, FloatStamped, BoolStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader

from INSIA_control.utils.utils import make_can_msg


class BrakeCanNode(Node):
    def __init__(self):
        super().__init__(node_name='BrakeCAN', start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False

        self.cobid = self.get_parameter('cobid').value
        self.can_connected = self.get_parameter('can').value

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_CAN = self.create_publisher(msg_type=CANGroup, topic=self.can_connected,
                                             qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=FloatStamped, topic=self.get_name() + '/Target',
                                 callback=lambda data: self.consigna(data, 3),
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=FloatStamped, topic=self.get_name() + '/Target_DAC1',
                                 callback=lambda data: self.consigna(data, 1),
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=FloatStamped, topic=self.get_name() + '/Target_DAC2',
                                 callback=lambda data: self.consigna(data, 2),
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def consigna(self, data, dac):
        msg = make_can_msg(node=self.cobid, index=0x0100, sub_index=dac, data=int(data.data * 100),
                           clock=self.get_clock().now().to_msg())
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
        manager = BrakeCanNode()
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
