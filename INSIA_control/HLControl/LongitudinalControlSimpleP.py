import os
from traceback import format_exc

import rclpy
import yaml
from insia_msg.msg import Telemetry, StringStamped, PetConduccion, ControladorFloat

from numpy import interp
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from example_interfaces.msg import Bool
from geometry_msgs.msg import Vector3
from simple_pid import PID
from yaml.loader import SafeLoader


class PID_params:
    def __init__(self, params):
        try:
            self.kp = params['kp'].value
            self.ti = params['ti'].value
            self.td = params['td'].value
        except:
            print('Error en el PID')
            exit(0)


class LongitudinalController(Node):
    def parameters_callback(self, params):
        for param in params:
            if param.name == "log_level":
                self.logger.set_level(param.value)
            elif param.name in ['throttle.kp', 'throttle.ti', 'throttle.td']:
                setattr(self.th_params, param.name.split('.')[1], param.value)
                self.set_accel_tunnings()
            elif param.name in ['brake.kp', 'brake.ti', 'brake.td']:
                setattr(self.br_params, param.name.split('.')[1], param.value)
                self.set_brake_tunnings()
        return SetParametersResult(successful=True)

    def set_accel_tunnings(self):
        self.logger.debug(f'Accel PID actualizado: {self.th_params.__dict__}')
        self.accel_pid.tunings = (self.th_params.kp, self.th_params.ti, self.th_params.td)

    def set_brake_tunnings(self):
        self.logger.debug(f'Brake PID actualizado: {self.br_params.__dict__}')
        self.brake_pid.tunings = (self.br_params.kp, self.br_params.ti, self.br_params.td)

    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)

        super().__init__(
            node_name='LongitudinalControlNode',
            namespace=vehicle_parameters['id_vehicle'],
            start_parameter_services=True,
            allow_undeclared_parameters=False,
            automatically_declare_parameters_from_overrides=True
        )

        self.logger = self.get_logger()
        log_level = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(log_level.value)

        self.speed_range = vehicle_parameters['speed']['range']
        self.th_params = PID_params(self.get_parameters_by_prefix('throttle'))
        self.br_params = PID_params(self.get_parameters_by_prefix('brake'))

        self.accel_pid = PID(self.th_params.kp, self.th_params.ti, self.th_params.td, setpoint=0, output_limits=(0, 1))
        self.brake_pid = PID(self.br_params.kp, self.br_params.ti, self.br_params.td, setpoint=0, output_limits=(0, 1))

        self.telemetry: Telemetry = None
        self.target: PetConduccion = None
        self.emergency_brake_active = False

        self.transition_time = 0.3
        self.last_action_time = self.get_clock().now().nanoseconds / 1e9
        self.prev_mode = "none"
        self.transitioning = False
        self.last_accel = 0.0
        self.last_brake = 0.0
        self.prev_accel = 0.0
        self.prev_brake = 0.0

        # Subscripciones
        self.telemetry_sub = self.create_subscription(Telemetry, 'Telemetry', self.telemetry_callback, HistoryPolicy.KEEP_LAST)
        self.target_sub = self.create_subscription(PetConduccion, 'Decision/Output', self.target_callback, HistoryPolicy.KEEP_LAST)
        self.emergency_brake_sub = self.create_subscription(Bool, 'emergency_brake', self.emergency_brake_callback, HistoryPolicy.KEEP_LAST)

        # Publicaciones
        self.throttle_pub = self.create_publisher(ControladorFloat, 'Throttle', HistoryPolicy.KEEP_LAST)
        self.brake_pub = self.create_publisher(ControladorFloat, 'Brake', HistoryPolicy.KEEP_LAST)
        self.pub_heartbeat = self.create_publisher(StringStamped, 'Heartbeat', HistoryPolicy.KEEP_LAST)
        self.pub_pid_values_th = self.create_publisher(Vector3, f'{self.get_name()}/PID_throttle_values', HistoryPolicy.KEEP_LAST)
        self.pub_pid_values_br = self.create_publisher(Vector3, f'{self.get_name()}/PID_brake_values', HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1.0, self.publish_heartbeat)
        self.add_on_set_parameters_callback(self.parameters_callback)

    def publish_heartbeat(self):
        msg = StringStamped(data=self.get_name())
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_heartbeat.publish(msg)

    def emergency_brake_callback(self, msg: Bool):
        self.emergency_brake_active = msg.data

    def target_callback(self, msg: PetConduccion):
        self.target = msg

    def smooth_transition(self, prev_value, new_value, elapsed, total):
        if elapsed >= total:
            return new_value
        return prev_value + (new_value - prev_value) * (elapsed / total)

    def telemetry_callback(self, msg: Telemetry):
        if self.target is None:
            return

        brake_signal = 0.0
        accel_signal = 0.0

        # 🛑 Freno de emergencia
        if self.emergency_brake_active:
            accel_signal = 0.0
            brake_signal = 1.0

        else:
            # Lógica PID
            telemetry_speed = interp(msg.speed, self.speed_range, [0., 1.])
            target_speed = interp(self.target.speed, self.speed_range, [0., 1.])
            error = target_speed - telemetry_speed

            if self.target.b_throttle and msg.brake < 25:
                accel_signal = self.accel_pid(-error)
                th_pid = self.accel_pid.components
                self.pub_pid_values_th.publish(Vector3(x=float(th_pid[0]), y=float(th_pid[1]), z=float(th_pid[2])))
            else:
                self.accel_pid.reset()

            if self.target.b_brake:
                brake_signal = self.brake_pid(error)
                br_pid = self.brake_pid.components
                self.pub_pid_values_br.publish(Vector3(x=float(br_pid[0]), y=float(br_pid[1]), z=float(br_pid[2])))
            else:
                self.brake_pid.reset()

            # Detecta el nuevo modo de actuación
            if accel_signal > 0 and brake_signal == 0:
                current_mode = "throttle"
            elif brake_signal > 0 and accel_signal == 0:
                current_mode = "brake"
            else:
                current_mode = "none"

            now = self.get_clock().now().nanoseconds / 1e9
            if current_mode != self.prev_mode and (accel_signal > 0. or brake_signal > 0.):
                self.transitioning = True
                self.last_action_time = now
                self.prev_accel = self.last_accel
                self.prev_brake = self.last_brake

            if self.transitioning:
                elapsed = now - self.last_action_time
                if current_mode == "throttle":
                    accel_signal = self.smooth_transition(self.prev_brake, accel_signal, elapsed, self.transition_time)
                    brake_signal = 0.0
                elif current_mode == "brake":
                    brake_signal = self.smooth_transition(self.prev_accel, brake_signal, elapsed, self.transition_time)
                    accel_signal = 0.0

                if elapsed >= self.transition_time:
                    self.transitioning = False
                    self.last_accel = accel_signal
                    self.last_brake = brake_signal
            else:
                self.last_accel = accel_signal
                self.last_brake = brake_signal

            self.prev_mode = current_mode

            # 🅿️ Freno de estacionamiento si detenido
            if self.target.speed == 0 and msg.speed < 1:
                brake_signal = max(min(self.get_parameter('static_brake').value, 1.), 0.)

        # Publicar las salidas de forma controlada
        self.throttle_pub.publish(ControladorFloat(
            header=Header(stamp=self.get_clock().now().to_msg()),
            #enable=not self.emergency_brake_active and self.target.b_throttle,
            enable=self.target.b_throttle,
            target=float(accel_signal)
        ))

        self.brake_pub.publish(ControladorFloat(
            header=Header(stamp=self.get_clock().now().to_msg()),
            enable=self.emergency_brake_active or self.target.b_brake,
            target=float(brake_signal)
        ))

    def shutdown(self):
        self.logger.info("Shutting down Longitudinal Controller")

def main(args=None):
    rclpy.init(args=args)
    controller = LongitudinalController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("LongitudinalController: Interrupción con Ctrl+C")
    except Exception as e:
        print(format_exc())
    finally:
        controller.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
