import rclpy
from insia_msg.msg import Telemetry, StringStamped, PetConduccion, ControladorFloat, ModoMision
from numpy import interp
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from simple_pid import PID
from std_msgs.msg import Header


class PID_params:
    def __init__(self, params):
        try:
            self.kp = params['kp'].value
            self.ti = params['ti'].value
            self.td = params['td'].value
        except:
            print('Error en el pid')
            exit(0)


class Control_MUTT(Node):
    def parameters_callback(self, params):
        for param in params:
            if param.name == "log_level":
                self.logger.set_level(param.value)
            elif param.name in ['throttle.kp', 'throttle.ti', 'throttle.td']:
                if param.name == 'throttle.kp':
                    self.th_params.kp = param.value
                elif param.name == 'throttle.ti':
                    self.th_params.ti = param.value
                elif param.name == 'throttle.td':
                    self.th_params.td = param.value
                self.set_throttle_tunnings()
            elif param.name == "speed_range":
                self.speed_range = (-param.value, param.value)
        return SetParametersResult(successful=True)

    def set_throttle_tunnings(self):
        self.logger.debug(f'Accel tunnings modified: {self.th_params =}')
        self.throttle_pid.tunings = (self.th_params.kp, self.th_params.ti, self.th_params.td)

    def __init__(self):
        super().__init__(node_name='Control_MUTT',
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False
        self.telemetry = Telemetry()
        self.pet_conduccion: PetConduccion = PetConduccion()
        self.current_status = ModoMision.MANUAL
        self.declare_parameter('speed_range', 20.)
        param_range_speed = self.get_parameter('speed_range').value
        self.speed_range = (-param_range_speed, param_range_speed)
        self.th_params = PID_params(self.get_parameters_by_prefix('throttle'))
        self.throttle_pid = PID(self.th_params.kp, self.th_params.ti, self.th_params.td, setpoint=0,
                                output_limits=(-1, 1))
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.create_subscription(msg_type=PetConduccion, topic='Decision/Output', callback=self.decision_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=Telemetry, topic='Telemetry', callback=self.telemetry_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=ModoMision, topic='Decision/Status',
                                 callback=self.decision_status_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_steering = self.create_publisher(msg_type=ControladorFloat, topic='MUTT_Device/Steering',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_throttle = self.create_publisher(msg_type=ControladorFloat, topic='MUTT_Device/Throttle',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_control = self.create_timer(1 / 10, self.control)
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def decision_status_callback(self, msg: ModoMision):
        self.current_status = msg.modo_mision

    def control(self):

        if self.pet_conduccion.b_steering:
            self.pub_steering.publish(
                ControladorFloat(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    enable=True,
                    target=interp(self.pet_conduccion.steering, (-100, 100), (-1, 1))
                )
            )
        else:
            self.pub_steering.publish(
                ControladorFloat(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    enable=False,
                    target=0.
                )
            )

        if self.pet_conduccion.b_throttle:
            if self.current_status == ModoMision.TELE_OPERADO:
                self.pub_throttle.publish(
                    ControladorFloat(
                        header=Header(stamp=self.get_clock().now().to_msg()),
                        enable=True,
                        target=interp(self.pet_conduccion.speed, (-100, 100), (-1, 1))
                    )
                )
                self.throttle_pid.reset()
            else:
                current_speed = interp(self.telemetry.speed, self.speed_range, [0, 1])
                target_speed = interp(self.pet_conduccion.speed, self.speed_range, [0, 1])
                error_speed = target_speed - current_speed
                if target_speed == 0 and abs(current_speed) < 2:
                    target_pid = 0.
                else:
                    target_pid = float(self.throttle_pid(-error_speed))

                self.pub_throttle.publish(
                    ControladorFloat(
                        header=Header(stamp=self.get_clock().now().to_msg()),
                        enable=True,
                        target=target_pid
                    )
                )
        else:
            self.pub_throttle.publish(
                ControladorFloat(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    enable=False,
                    target=0.
                )
            )
            self.throttle_pid.reset()

    def decision_callback(self, decision: PetConduccion):
        self.pet_conduccion = decision

    def telemetry_callback(self, telemetry: Telemetry):
        self.telemetry = telemetry

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
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = Control_MUTT()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
