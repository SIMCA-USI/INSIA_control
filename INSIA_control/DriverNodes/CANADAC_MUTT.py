from traceback import format_exc

import rclpy
from insia_msg.msg import CANGroup, StringStamped, FloatStamped, BoolStamped, CAN
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from INSIA_control.utils.filtro import Decoder
from INSIA_control.utils.utils import convert_types
from yaml.loader import SafeLoader
from numpy import interp

from INSIA_control.utils.utils import make_can_msg


class CANADACNode(Node):
    def __init__(self):
        super().__init__(node_name='CANADAC_MUTT', start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        # Configuración inicial del logger
        self.logger = self.get_logger()
        self._configure_logging()
        self.enabled = False
        self.decoder = Decoder(dictionary=self.get_parameter('dictionary').value)
        self.vehicle_state = {}

        self.shutdown_flag = False
        self.cobid = 0x100

        try:
            self._init_parameters()
            self._init_publishers()
            self._init_subscriptions()
            self._init_timers()
            self.logger.info("Node initialized successfully")
        except Exception as e:
            self.logger.critical(f"Initialization failed: {str(e)}\n{format_exc()}")
            raise
        
    def update_values(self, name, value):
        if "rpm" in self.vehicle_state:
            if self.vehicle_state["rpm"] != value:
                self.pub_rpm.publish(FloatStamped(header=Header(stamp=self.get_clock().now().to_msg()), data=value))
                self.vehicle_state["rpm"] = value
            # Si el valor es igual, no se hace nada
        else:
            self.vehicle_state["rpm"] = value
            self.pub_rpm.publish(FloatStamped(header=Header(stamp=self.get_clock().now().to_msg()), data=value))
        
    def msg_can(self, msg):
        try:
            name, value = self.decoder.decode(msg)
            self.update_values(name, value)
            # self.logger.debug(f'Decoded {name}: {value}')
        except ValueError as e:
            self.logger.debug(f'{e}')

    def _configure_logging(self):
        """Configura los niveles de log y formato"""
        log_level = self.get_parameter(
            'log_level'
        ).value
        self.logger.set_level(log_level)
        self.logger.debug("Logger configured with level %d" % log_level)

    def _init_parameters(self):
        """Carga y valida parámetros"""
        self.can_connected = self.get_parameter_or(
            'can',
            Parameter(name='can', value='can_output')
        ).value
        self.logger.info(f"Using CAN interface: {self.can_connected}")

    def _init_publishers(self):
        """Inicializa los publishers"""
        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_rpm = self.create_publisher(msg_type=FloatStamped, topic='RPM',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_CAN = self.create_publisher(msg_type=CANGroup, topic=self.can_connected,
                                             qos_profile=HistoryPolicy.KEEP_LAST)
        self.logger.debug("Publishers initialized")

    def _init_subscriptions(self):
        """Inicializa las suscripciones"""
        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/Enable', callback=self.set_enable,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=FloatStamped, topic=self.get_name() + '/Steering', callback=self.set_steering,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=FloatStamped, topic=self.get_name() + '/Throttle', callback=self.set_throttle,
                                 qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=CAN, topic='CAN', callback=self.msg_can, qos_profile=HistoryPolicy.KEEP_LAST)
        self.logger.debug("Subscriptions initialized")

    def _init_timers(self):
        """Configura los timers"""
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)
        self.logger.debug("Timers initialized")

    def set_enable(self, data: BoolStamped):
        """Maneja el estado de habilitación"""
        try:
            if self.enabled != data.data:
                self.logger.debug(f"Received enable signal: {data.data}")
                msg_data = 0x01 if data.data else 0x00
                mode = "Drive" if data.data else "Stop"
                msg = make_can_msg(
                    node=self.cobid,
                    index=0x0001,
                    data=msg_data,
                    clock=self.get_clock().now().to_msg()
                )
                self.pub_CAN.publish(CANGroup(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    can_frames=[msg]
                ))
                self.logger.debug(f"Sent {mode} mode command")
            self.enabled = data.data
        except Exception as e:
            self.logger.error(f'{e}')

    def set_steering(self, data: FloatStamped):
        if self.enabled:
            msg = make_can_msg(node=self.cobid, index=0x0002, sub_index=0x02, data=-data.data, c_type='f',
                               clock=self.get_clock().now().to_msg())
            self.pub_CAN.publish(CANGroup(
                header=Header(stamp=self.get_clock().now().to_msg()),
                can_frames=[
                    msg
                ]
            ))

    def set_throttle(self, data: FloatStamped):
        if self.enabled:
            msg = make_can_msg(node=self.cobid, index=0x0002, sub_index=0x01, data=data.data, c_type='f',
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
