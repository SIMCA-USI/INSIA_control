import os
from traceback import format_exc

import rclpy
import yaml
from insia_msg.msg import StringStamped, Telemetry, ControladorFloat, FloatStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader


class AccelNode(Node):

    def parameters_callback(self, params):
        print(params)
        for param in params:
            if param.name == "log_level":
                self.logger.set_level(param.value)
        return SetParametersResult(successful=True)

    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='SpeedNode', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False
        params = vehicle_parameters.get('steering')
        self.device_range = params['range']
        self.telemetry = Telemetry()
        self.controller = None
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.create_subscription(msg_type=ControladorFloat, topic=self.get_name(), callback=self.controller_update,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_target = self.create_publisher(msg_type=FloatStamped, topic='Accel',
                                                qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def controller_update(self, data: ControladorFloat):
        """
        Receive data from HL LongitudinalControlPID and transform it in Float stamped
        :param data: Controlador Float +-1
        :return: None
        """
        self.controller = data
        self.logger.debug(f'{self.controller.enable =}')
        if self.controller.enable:
            self.pub_target.publish(FloatStamped(
                header=Header(stamp=self.get_clock().now().to_msg()),
                data=4 * data.target
            ))
        else:
            self.pub_target.publish(FloatStamped(
                header=Header(stamp=self.get_clock().now().to_msg()),
                data=-1.
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
            # Full brake in case of finishing
            self.pub_target.publish(FloatStamped(
                header=Header(stamp=self.get_clock().now().to_msg()),
                data=-1.
            ))
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = AccelNode()
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
