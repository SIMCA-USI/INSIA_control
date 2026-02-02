import os
from traceback import format_exc

import rclpy
import yaml
from insia_msg.msg import StringStamped, EPOSConsigna, EPOSDigital, BoolStamped, IntStamped
from insia_msg.msg import Telemetry, ControladorFloat
from numpy import interp
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader


class SteeringNode(Node):

    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='SteeringNode', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        params = vehicle_parameters.get('steering')
        self.device_range = params['range']
        self.actuator_inverted = params['actuator_inverted']
        self.telemetry = Telemetry()
        self.rate_telemetry = 4
        self.controller = ControladorFloat()
        self.consigna_alcanzada = 25

        self.create_subscription(msg_type=ControladorFloat,
                                 topic=self.get_name(),
                                 callback=self.controller_update, qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_enable = self.create_publisher(msg_type=BoolStamped, topic='EPOS4_Volante/Enable',
                                                qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_enable_steering = self.create_publisher(msg_type=EPOSDigital, topic='EPOS4_Volante/Digital',
                                                         qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_target = self.create_publisher(msg_type=EPOSConsigna, topic='EPOS4_Volante/TargetPosition',
                                                qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)
        self.timer_control = self.create_timer(1/4, self.controller_function)

    def controller_update(self, data:ControladorFloat):
        self.controller = data
        self.pub_enable.publish(BoolStamped(
            header=Header(stamp=self.get_clock().now().to_msg()),
            data=self.controller.enable
        ))
        self.pub_enable_steering.publish(EPOSDigital(
            header=Header(stamp=self.get_clock().now().to_msg()),
            enable=self.controller.enable,
            io_digital=2
        ))

    def controller_function(self):
        if self.controller.enable:
            result = int(interp(self.controller.target, (-1, 1), self.device_range))
            if -self.consigna_alcanzada < result < self.consigna_alcanzada:
                self.logger.debug(f'Consigna alcanzada')
            else:
                self.logger.debug(f'Consigna no alcanzada {result}')
                self.pub_target.publish(EPOSConsigna(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    position=-result if self.actuator_inverted else result,
                    mode=EPOSConsigna.RELATIVO
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
            self.timer_heartbeat.cancel()
            # Desactivar EPOS4
            self.pub_enable.publish(BoolStamped(
                header=Header(stamp=self.get_clock().now().to_msg()),
                data=False
            ))
            # Desactivar reles
            self.pub_enable_steering.publish(EPOSDigital(
                header=Header(stamp=self.get_clock().now().to_msg()),
                enable=False,
                io_digital=2
            ))
            # Poner target de motor a 0 por si acaso
            self.pub_target.publish(EPOSConsigna(
                header=Header(stamp=self.get_clock().now().to_msg()),
                position=int(interp(0, (-1, 1), self.device_range)),
                mode=EPOSConsigna.RELATIVO
            ))
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = SteeringNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(format_exc())
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
