import rclpy
import numpy as np
from insia_msg.msg import Telemetry, StringStamped, PetConduccion, ControladorFloat, ModoMision
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header


class PID_params:
    def __init__(self, params):
        try:
            self.kp = params.get('kp', Parameter('kp', value=0.0)).value
            self.ki = params.get('ki', Parameter('ki', value=0.0)).value
            self.kd = params.get('kd', Parameter('kd', value=0.0)).value
            self.i_windup = params.get('i_windup', Parameter('i_windup', value=1.0)).value
        except Exception as e:
            print(f'Error al cargar los parámetros del PID: {e}')
            exit(0)


class Control_MUTT(Node):
    def parameters_callback(self, params):
        for param in params:
            if param.name == "log_level":
                self.logger.set_level(param.value)
            elif param.name in ['throttle.kp', 'throttle.ki', 'throttle.kd', 'throttle.i_windup']:
                if param.name == 'throttle.kp':
                    self.th_params.kp = param.value
                elif param.name == 'throttle.ki':
                    self.th_params.ki = param.value
                elif param.name == 'throttle.kd':
                    self.th_params.kd = param.value
                elif param.name == 'throttle.i_windup':
                    self.th_params.i_windup = param.value
                self.logger.debug(f'Accel tunnings modified: Kp={self.th_params.kp}, Ki={self.th_params.ki}, Kd={self.th_params.kd}, Windup={self.th_params.i_windup}')
            elif param.name == "speed_range":
                self.speed_range = (-param.value, param.value)
        return SetParametersResult(successful=True)

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
        
        # Cargar parámetros PID personalizados
        self.th_params = PID_params(self.get_parameters_by_prefix('throttle'))
        
        # Variables de estado del PID Manual
        self.integral = 0.0
        self.prev_error = 0.0
        self.dt = 0.1  # 1/10 basado en el timer_control
        
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

        self.timer_control = self.create_timer(self.dt, self.control)
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def _pid(self, error, min_speed, max_speed):
        """Implementación manual del PID suministrada por tus compañeros"""
        self.integral = np.clip(self.integral + error * self.dt, -self.th_params.i_windup, self.th_params.i_windup)
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        
        output = self.th_params.kp * error + self.th_params.ki * self.integral + self.th_params.kd * derivative
        return np.clip(output, min_speed, max_speed)

    def _reset_pid(self):
        """Reinicia la memoria del PID"""
        self.integral = 0.0
        self.prev_error = 0.0

    def decision_status_callback(self, msg: ModoMision):
        self.current_status = msg.modo_mision

    def control(self):

        if self.pet_conduccion.b_steering:
            self.pub_steering.publish(
                ControladorFloat(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    enable=True,
                    target=np.interp(self.pet_conduccion.steering, (-100, 100), (-1, 1))
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
            if self.current_status == ModoMision.TELE_OPERADO or self.current_status == ModoMision.RADIO_CONTROL:
                self.pub_throttle.publish(
                    ControladorFloat(
                        header=Header(stamp=self.get_clock().now().to_msg()),
                        enable=True,
                        target=np.interp(self.pet_conduccion.speed, (-100, 100), (-1, 1))
                    )
                )
                self._reset_pid()
            else:
                # MODOS AUTÓNOMOS: Con PID y NUNCA hacia atrás
                
                # Interpolaciones para el cálculo del error
                current_speed = np.interp(self.telemetry.speed, self.speed_range, [0, 1])
                target_speed = np.interp(self.pet_conduccion.speed, self.speed_range, [0, 1])
                error_speed = target_speed - current_speed
                
                velocidad_objetivo = self.pet_conduccion.speed
                velocidad_actual = self.telemetry.speed

                # Zona muerta de parada
                if velocidad_objetivo <= 0.0 and abs(velocidad_actual) <= 1.5:
                    target_pid = 0.0
                    self._reset_pid() # Limpiamos el término integral y derivativo
                else:
                    # Aplicamos nuestro PID y limitamos la salida estrictamente entre 0.0 y 1.0
                    target_pid = float(self._pid(error_speed, min_speed=0.0, max_speed=1.0))

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
            self._reset_pid()

    def decision_callback(self, decision: PetConduccion):
        self.pet_conduccion = decision

    def telemetry_callback(self, telemetry: Telemetry):
        self.telemetry = telemetry

    def publish_heartbeat(self):
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
        if manager:
            manager.shutdown()


if __name__ == '__main__':
    main()