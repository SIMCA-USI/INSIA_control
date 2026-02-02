import os
from traceback import format_exc

import rclpy
import yaml
from insia_msg.msg import StringStamped, BoolStamped, Telemetry, ControladorFloat, FloatStamped
from numpy import interp
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader


class ThrottleNode(Node):

    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='ThrottleNode', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False
        params = vehicle_parameters.get('throttle')
        self.device_range = params['range']
        self.telemetry = Telemetry()
        self.controller = None

        self.create_subscription(msg_type=ControladorFloat,
                                 topic=self.get_name(),
                                 callback=self.controller_update, qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped,
                                                   topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_enable_throttle = self.create_publisher(msg_type=BoolStamped, topic='CANADAC_Acelerador/EnableTension',
                                                         qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_target = self.create_publisher(msg_type=FloatStamped, topic='CANADAC_Acelerador/Target',
                                                qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def controller_update(self, data):
        self.controller = data
        self.pub_enable_throttle.publish(BoolStamped(
            header=Header(stamp=self.get_clock().now().to_msg()),
            data=self.controller.enable,
        ))
        if self.controller.enable:
            self.pub_target.publish(FloatStamped(
                header=Header(stamp=self.get_clock().now().to_msg()),
                data=interp(self.controller.target, (0, 1), self.device_range)
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
            # Desactivar EPOS4
            # Desactivar reles
            self.pub_enable_throttle.publish(BoolStamped(
                header=Header(stamp=self.get_clock().now().to_msg()),
                data=False,
            ))
            # Poner target de motor a 0 por si acaso
            self.pub_target.publish(FloatStamped(
                header=Header(stamp=self.get_clock().now().to_msg()),
                data=interp(0, (0, 1), self.device_range)
            ))
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = ThrottleNode()
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
