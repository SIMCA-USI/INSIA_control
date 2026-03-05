import os
from traceback import format_exc

import rclpy
import yaml
from insia_msg.msg import StringStamped, Telemetry, PetConduccion
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Float64
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
        self.controller = None
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.telemetry = Telemetry()
        self.min_limit = 0.5
        self.pendiente = 3 / 4

        self.create_subscription(msg_type=PetConduccion, topic='Decision/Output', callback=self.controller_update,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=Telemetry, topic='Telemetry', callback=self.telemetry_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_target = self.create_publisher(msg_type=Float64, topic='/robot/motors/propulsion/setpoint',
                                                qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def telemetry_callback(self, data):
        self.telemetry = data

    def controller_update(self, data: PetConduccion):
        """
        Receive data from HL LongitudinalControlPID and transform it in Float stamped
        :param data: Controlador Float +-1
        :return: None
        """
        if self.telemetry.speed >= 0:
            max_value = self.telemetry.speed * (1 / self.pendiente) + self.min_limit
            min_value = self.telemetry.speed * self.pendiente - self.min_limit
        else:
            max_value = self.telemetry.speed * self.pendiente + self.min_limit
            min_value = self.telemetry.speed * (1 / self.pendiente) - self.min_limit
        self.logger.error(f'{max_value = :2f} {min_value = :2f} {self.telemetry.speed = :2f}')
        if data.b_throttle:
            out = min(max(data.speed, min_value), max_value)
            self.pub_target.publish(Float64(data=(out / 3.6) * 100))
        else:
            out = min(max(0., min_value), max_value)
            self.pub_target.publish(Float64(data=(out / 3.6) * 100))

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
            self.pub_target.publish(Float64(data=0.))
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
