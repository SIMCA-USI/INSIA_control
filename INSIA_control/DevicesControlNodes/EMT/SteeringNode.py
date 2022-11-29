import os
from traceback import format_exc

import rclpy
import yaml
from insia_msg.msg import StringStamped, BoolStamped, Telemetry, ControladorFloat, EPOSDigital, EPOSConsigna
from numpy import interp
from rclpy import timer
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header, Bool
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
        self.shutdown_flag = False
        params = vehicle_parameters.get('steering')
        self.device_range = params['range']
        if not self.has_parameter('delay_turn'):
            self.declare_parameter('delay_turn', 2.)

        self.delay_turn = self.get_parameter('delay_turn')
        self.timer_delay: timer = None
        self.telemetry = Telemetry()
        self.controller: ControladorFloat = ControladorFloat()
        # Bloquea el giro para poder probar la activación del electroiman
        self.lock_turn = False
        self.prev_lock_turn = False

        self.create_subscription(msg_type=ControladorFloat, topic=self.get_name(), callback=self.controller_update,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=Bool, topic=self.get_name() + '/LockTurn', callback=self.lock_turn_update,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_enable = self.create_publisher(msg_type=BoolStamped, topic='MCD60_Volante/Enable',
                                                qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_enable_steering = self.create_publisher(msg_type=EPOSDigital, topic='MCD60_Volante/Digital',
                                                         qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_target = self.create_publisher(msg_type=EPOSConsigna, topic='MCD60_Volante/TargetPosition',
                                                qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=Telemetry, topic='Telemetry', callback=self.telemetry_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def lock_turn_update(self, data: Bool):
        if self.timer_delay is None:
            # Si no hay timer actuar directamente
            self.lock_turn = data.data
        # Si hay timer actuar solo sobre prev para que lo cargue al finalizar
        self.prev_lock_turn = data.data

    def telemetry_callback(self, data):
        self.telemetry = data

    def delay_callback(self):
        self.lock_turn = self.prev_lock_turn
        self.timer_delay.cancel()
        self.timer_delay = None

    def controller_update(self, data: ControladorFloat):
        if not self.controller.enable and data.enable:
            # Solo en el caso que pasemos de deshabilitado a habilitado
            self.prev_lock_turn = self.lock_turn
            self.lock_turn = True
            if self.timer_delay is None:
                self.timer_delay = self.create_timer(self.get_parameter('delay_turn').value, self.delay_callback)
            else:
                self.timer_delay: timer
                self.timer_delay.reset()
        self.controller = data
        self.pub_enable.publish(BoolStamped(
            header=Header(stamp=self.get_clock().now().to_msg()),
            data=self.controller.enable
        ))
        self.pub_enable_steering.publish(EPOSDigital(
            header=Header(stamp=self.get_clock().now().to_msg()),
            enable=self.controller.enable,
            io_digital=4
        ))
        # if self.controller.enable and self.telemetry.brake < 50 and not self.lock_turn:
        if self.controller.enable and not self.lock_turn:
            self.pub_target.publish(EPOSConsigna(
                header=Header(stamp=self.get_clock().now().to_msg()),
                position=int(interp(self.controller.target, (-1, 1), self.device_range)),
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
            self.shutdown_flag = True
            self.timer_heartbeat.cancel()
            # Desactivar EPOS
            self.pub_enable.publish(BoolStamped(
                header=Header(stamp=self.get_clock().now().to_msg()),
                data=False
            ))
            # Desactivar reles
            self.pub_enable_steering.publish(EPOSDigital(
                header=Header(stamp=self.get_clock().now().to_msg()),
                enable=False,
                io_digital=4
            ))
            # Poner target de motor a 0 por si acaso
            self.pub_target.publish(EPOSConsigna(
                header=Header(stamp=self.get_clock().now().to_msg()),
                position=int(interp(0, (-1, 1), self.device_range))
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
        format_exc()
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
