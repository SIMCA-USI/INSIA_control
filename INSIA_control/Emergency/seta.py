from traceback import format_exc

import rclpy
from INSIA_control.utils.filtro import Decoder
from insia_msg.msg import CAN, StringStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy


class SetaV3(Node):

    def parameters_callback(self, params):
        for param in params:
            if param.name == "log_level":
                self.logger.set_level(param.value)
        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__(node_name='Node', start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        # Logging configuration
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.vehicle_state = {}

        self.decoder = Decoder(dictionary=self.get_parameter('dictionary').value)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=CAN, topic='CAN', callback=self.msg_can, qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def msg_can(self, msg):
        try:
            name, value = self.decoder.decode(msg)
            self.vehicle_state.update({name: value})
            self.logger.error(f'Decoded {name}: {value}')
        except ValueError as e:
            self.logger.debug(f'{e}')

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
        pass


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = SetaV3()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print('Decision: Keyboard interrupt')
        manager.shutdown()
        manager.destroy_node()
    except Exception:
        print(format_exc())


if __name__ == '__main__':
    main()
