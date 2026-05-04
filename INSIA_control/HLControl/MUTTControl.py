import rclpy
import numpy as np
from insia_msg.msg import Telemetry, Telemetry2, StringStamped, PetConduccion, ControladorFloat, ModoMision
from numpy import interp
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header

# --- Clase CustomPID con Anti-Windup ---
class CustomPID:
    def __init__(self, kp, ki, kd, i_windup, dt=0.1, output_limits=(-1.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_windup = i_windup
        self.dt = dt
        self.output_limits = output_limits
        
        self.integral = 0.0
        self.prev_error = 0.0

    def update_tunings(self, kp, ki, kd, i_windup):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_windup = i_windup

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def __call__(self, error):
        self.integral = np.clip(self.integral + error * self.dt, -self.i_windup, self.i_windup)
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        min_out, max_out = self.output_limits
        return np.clip(output, min_out, max_out)
# ---------------------------------------

class PID_params:
    def __init__(self, params):
        try:
            self.kp = params['kp'].value if 'kp' in params else 1.0
            self.ki = params['ki'].value if 'ki' in params else 0.0
            self.kd = params['kd'].value if 'kd' in params else 0.0
            self.i_windup = params['i_windup'].value if 'i_windup' in params else 1.0
        except KeyError as e:
            print(f'Falta el parámetro en el pid: {e}')
            exit(0)


class Control_MUTT(Node):
    def parameters_callback(self, params):
        for param in params:
            if param.name == "log_level":
                self.logger.set_level(param.value)
            
            # --- Actualización de parámetros PID ---
            elif param.name in ['throttle.kp', 'throttle.ki', 'throttle.kd', 'throttle.i_windup']:
                if param.name == 'throttle.kp':
                    self.th_params.kp = param.value
                elif param.name == 'throttle.ki':
                    self.th_params.ki = param.value
                elif param.name == 'throttle.kd':
                    self.th_params.kd = param.value
                elif param.name == 'throttle.i_windup':
                    self.th_params.i_windup = param.value
                self.set_throttle_tunnings()
            
            # --- Actualización de parámetros Feedforward ---
            elif param.name in ['throttle.ff_0kmh', 'throttle.ff_1kmh', 'throttle.ff_2kmh', 'throttle.ff_3kmh', 'throttle.ff_4kmh']:
                if param.name == 'throttle.ff_0kmh':
                    self.ff_throttle_points[0] = param.value
                elif param.name == 'throttle.ff_1kmh':
                    self.ff_throttle_points[1] = param.value
                elif param.name == 'throttle.ff_2kmh':
                    self.ff_throttle_points[2] = param.value
                elif param.name == 'throttle.ff_3kmh':
                    self.ff_throttle_points[3] = param.value
                elif param.name == 'throttle.ff_4kmh':
                    self.ff_throttle_points[4] = param.value
                self.logger.debug(f'Feedforward points updated: {self.ff_throttle_points}')
            
            elif param.name == "speed_range":
                self.speed_range_val = param.value
                self.speed_range = (-param.value, param.value)
                
        return SetParametersResult(successful=True)

    def set_throttle_tunnings(self):
        self.logger.debug(f'Accel tunnings modified: {self.th_params.__dict__}')
        self.throttle_pid.update_tunings(
            self.th_params.kp, 
            self.th_params.ki, 
            self.th_params.kd, 
            self.th_params.i_windup
        )

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
        self.vehicle_ready = False
        
        self.current_status = ModoMision.MANUAL
        self.throttle_limit = 40.0
        self.base_steering_limit = 25.0

        self.declare_parameter('speed_range', 30.)
        self.speed_range_val = self.get_parameter('speed_range').value
        self.speed_range = (-self.speed_range_val, self.speed_range_val)

        # =====================================================================
        # DECLARACIÓN E INICIALIZACIÓN DE PARÁMETROS FEEDFORWARD
        # =====================================================================
        self.declare_parameter('throttle.ff_0kmh', 0.0)
        self.declare_parameter('throttle.ff_1kmh', 0.10)
        self.declare_parameter('throttle.ff_2kmh', 0.135)
        self.declare_parameter('throttle.ff_3kmh', 0.16)
        self.declare_parameter('throttle.ff_4kmh', 0.18)

        # Eje X: Velocidad objetivo en km/h
        self.ff_speed_points = [0.0, 1.0, 2.0, 4.0]
        
        # Eje Y: Valores leídos de los parámetros
        self.ff_throttle_points = [
            self.get_parameter('throttle.ff_0kmh').value,
            self.get_parameter('throttle.ff_1kmh').value,
            self.get_parameter('throttle.ff_2kmh').value,
            self.get_parameter('throttle.ff_3kmh').value,
            self.get_parameter('throttle.ff_4kmh').value
        ]
        # =====================================================================
        
        self.th_params = PID_params(self.get_parameters_by_prefix('throttle'))
        self.throttle_pid = CustomPID(
            self.th_params.kp, self.th_params.ki, self.th_params.kd, self.th_params.i_windup,
            dt=0.1, output_limits=(-1.0, 1.0)
        )
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.create_subscription(msg_type=PetConduccion, topic='Decision/Output', callback=self.decision_callback, qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=Telemetry, topic='Telemetry', callback=self.telemetry_callback, qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=Telemetry2, topic='Telemetry2', callback=self.telemetry2_callback, qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=ModoMision, topic='Decision/Status', callback=self.decision_status_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat', qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_steering = self.create_publisher(msg_type=ControladorFloat, topic='MUTT_Device/Steering', qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_throttle = self.create_publisher(msg_type=ControladorFloat, topic='MUTT_Device/Throttle', qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_control = self.create_timer(1 / 10, self.control)
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def decision_status_callback(self, msg: ModoMision):
        self.current_status = msg.modo_mision
        match self.current_status:
            case ModoMision.AUTONOMO:
                self.throttle_limit = 60.0
                self.base_steering_limit = 35.0
            case ModoMision.TELE_OPERADO:
                self.throttle_limit = 80.0
                self.base_steering_limit = 50.0
            case ModoMision.FOLLOW_ME:
                self.throttle_limit = 40.0
                self.base_steering_limit = 25.0
            case ModoMision.RADIO_CONTROL:
                self.throttle_limit = 100.0
                self.base_steering_limit = 60.0
            case _:
                self.throttle_limit = 40.0
                self.base_steering_limit = 25.0

    def control(self):
        # --- LÓGICA DE GIRO ---
        if self.pet_conduccion.b_steering:
            velocidad_actual = abs(self.telemetry.speed)
            dynamic_steering_limit = interp(velocidad_actual, [0.0, self.speed_range_val], [self.base_steering_limit, 25.0])
            target_steering = interp(self.pet_conduccion.steering, (-100, 100), (-1, 1))
            target_steering_limited = target_steering * (dynamic_steering_limit / 100.0)

            self.pub_steering.publish(
                ControladorFloat(header=Header(stamp=self.get_clock().now().to_msg()), enable=True, target=target_steering_limited)
            )
        else:
            self.pub_steering.publish(
                ControladorFloat(header=Header(stamp=self.get_clock().now().to_msg()), enable=False, target=0.)
            )

        # --- LÓGICA DE ACELERADOR ---
        if self.pet_conduccion.b_throttle:
            if self.current_status == ModoMision.TELE_OPERADO or self.current_status == ModoMision.RADIO_CONTROL:
                target_throttle = interp(self.pet_conduccion.speed, (-100, 100), (-1, 1))
                target_throttle_limited = target_throttle * (self.throttle_limit / 100.0)
                
                self.pub_throttle.publish(
                    ControladorFloat(header=Header(stamp=self.get_clock().now().to_msg()), enable=True, target=target_throttle_limited)
                )
                self.throttle_pid.reset()
            else:
                self.throttle_pid.output_limits = (0.0, self.throttle_limit / 100.0)

                current_speed = interp(self.telemetry.speed, self.speed_range, [0, 1])
                target_speed = interp(self.pet_conduccion.speed, self.speed_range, [0, 1])
                error_speed = target_speed - current_speed
                
                velocidad_objetivo = self.pet_conduccion.speed
                velocidad_actual = self.telemetry.speed

                if not self.vehicle_ready:
                    target_pid = 0.0
                    self.throttle_pid.reset()
                elif velocidad_objetivo <= 0.0 and abs(velocidad_actual) <= 1.5:
                    target_pid = 0.0
                    self.throttle_pid.reset()
                else:
                    # 1. CÁLCULO DEL FEEDFORWARD
                    ff_base = interp(abs(velocidad_objetivo), self.ff_speed_points, self.ff_throttle_points)
                    
                    # 2. CÁLCULO DEL PID
                    pid_correction = float(self.throttle_pid(-error_speed))
                    
                    # 3. SUMA TOTAL (Limitada a los máximos de seguridad de la misión)
                    target_total = ff_base + pid_correction
                    target_pid = float(np.clip(target_total, 0.0, self.throttle_limit / 100.0))

                self.pub_throttle.publish(
                    ControladorFloat(header=Header(stamp=self.get_clock().now().to_msg()), enable=True, target=target_pid)
                )
        else:
            self.pub_throttle.publish(
                ControladorFloat(header=Header(stamp=self.get_clock().now().to_msg()), enable=False, target=0.)
            )
            self.throttle_pid.reset()

    def decision_callback(self, decision: PetConduccion):
        self.pet_conduccion = decision

    def telemetry_callback(self, telemetry: Telemetry):
        self.telemetry = telemetry

    def telemetry2_callback(self, msg: Telemetry2):
        self.vehicle_ready = msg.vehicle_ready

    def publish_heartbeat(self):
        msg = StringStamped(data=self.get_name())
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