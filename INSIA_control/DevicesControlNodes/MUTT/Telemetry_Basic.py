import rclpy
from insia_msg.msg import FloatStamped, BoolStamped, Telemetry, StringStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy


class TelemetryMUTT(Node):
    def __init__(self):
        super().__init__(node_name='Telemetry',
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        self.id_plataforma = self.get_namespace()

        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)

        self.shutdown_flag = False

        self.telemetry = Telemetry(id_plataforma='MUTT')

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_telemetry = self.create_publisher(msg_type=Telemetry, topic='Telemetry',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic='CANADAC_MUTT/Enable', callback=self.set_enable,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=FloatStamped, topic='CANADAC_MUTT/Steering', callback=self.set_steering,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=FloatStamped, topic='CANADAC_MUTT/Throttle', callback=self.set_throttle,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_telemetry = self.create_timer(1 / 20, self.publish_telemetry)
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def set_enable(self, msg: BoolStamped):
        if msg.data:
            self.telemetry.gears = 'Drive'
        else:
            self.telemetry.gears = 'Stop'

    def set_steering(self, msg: FloatStamped):
        self.telemetry.steering = msg.data

    def set_throttle(self, msg: FloatStamped):
        self.telemetry.speed = msg.data

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

    def publish_telemetry(self):
        self.telemetry.header.stamp = self.get_clock().now().to_msg()
        self.pub_telemetry.publish(msg=self.telemetry)

    def shutdown(self):
        try:
            self.shutdown_flag = True
            self.timer_telemetry.cancel()
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    # manager = None
    try:
        manager = TelemetryMUTT()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(e)
    # finally:
    #     manager.shutdown()


if __name__ == '__main__':
    main()
