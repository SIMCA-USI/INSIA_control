from traceback import format_exc

import rclpy
from insia_msg.msg import StringStamped, FloatStamped, BoolStamped, ControladorFloat
from numpy import interp
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header


class MUTT_Node(Node):

    def on_set_params_callback(self, params):

        for param in params:
            if param.name == 'log_level':
                if param.value in [0, 10, 20, 30, 40, 50]:
                    self.logger.set_level(param.value)
                else:
                    SetParametersResult(
                        successful=False,
                        reason='Not logging valid value'
                    )
            elif param.name == 'throttle_range':
                if 0 <= param.value <= 100:
                    self._set_range_throttle(param.value)
                    self.logger.info(f'New Throttle range: {param.value}')
                else:
                    SetParametersResult(
                        successful=False,
                        reason='Throttle out of range'
                    )
            elif param.name == 'steering_range':
                if 0 <= param.value <= 100:
                    self._set_range_steering(param.value)
                    self.logger.info(f'New Steering range: {param.value}')
                else:
                    SetParametersResult(
                        successful=False,
                        reason='Steering out of range'
                    )

        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__(node_name='MUTT_Device',
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)

        self.declare_parameter('throttle_range', 80.0)
        self.throttle_range = 0
        self._set_range_throttle(self.get_parameter('throttle_range').value)

        self.declare_parameter('steering_range', 30.0)
        self.steering_range = 0
        self._set_range_steering(self.get_parameter('steering_range').value)

        self.add_on_set_parameters_callback(self.on_set_params_callback)

        self.shutdown_flag = False

        self.enable_throttle = False
        self.enable_steering = False

        self.create_subscription(msg_type=ControladorFloat,
                                 topic=self.get_name() + '/Throttle',
                                 callback=self.controller_throttle, qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=ControladorFloat,
                                 topic=self.get_name() + '/Steering',
                                 callback=self.controller_steering, qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped,
                                                   topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_enable = self.create_publisher(msg_type=BoolStamped, topic='CANADAC_MUTT/Enable',
                                                qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_target_throttle = self.create_publisher(msg_type=FloatStamped, topic='CANADAC_MUTT/Throttle',
                                                         qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_target_steering = self.create_publisher(msg_type=FloatStamped, topic='CANADAC_MUTT/Steering',
                                                         qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def controller_throttle(self, data: ControladorFloat):
        mutt_enable = data.enable or self.enable_steering
        self.enable_throttle = data.enable
        self.pub_enable.publish(BoolStamped(
            header=Header(stamp=self.get_clock().now().to_msg()),
            data=mutt_enable,
        ))
        if mutt_enable:
            self.pub_target_throttle.publish(FloatStamped(
                header=Header(stamp=self.get_clock().now().to_msg()),
                data=interp(data.target, (-1, 1), self.throttle_range)
            ))

    def controller_steering(self, data: ControladorFloat):
        mutt_enable = data.enable or self.enable_throttle
        self.enable_steering = data.enable
        self.pub_enable.publish(BoolStamped(
            header=Header(stamp=self.get_clock().now().to_msg()),
            data=mutt_enable,
        ))
        if mutt_enable:
            self.pub_target_steering.publish(FloatStamped(
                header=Header(stamp=self.get_clock().now().to_msg()),
                data=interp(data.target, (-1, 1), self.steering_range)
            ))

    def _set_range_throttle(self, range: float):
        self.throttle_range = (-range, range)

    def _set_range_steering(self, range: float):
        self.steering_range = (-range, range)

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
            self.pub_enable.publish(BoolStamped(
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
        manager = MUTT_Node()
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
