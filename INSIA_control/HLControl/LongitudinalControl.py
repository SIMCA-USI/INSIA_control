import os

import rclpy
import yaml
from geometry_msgs.msg import Vector3
from insia_msg.msg import Telemetry, StringStamped, PetConduccion, ControladorFloat
from numpy import interp
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader

from INSIA_control.utils.pid import PID


class PID_params:
    def __init__(self, params):
        self.kp = params['kp'].value
        self.ti = params['ti'].value
        self.td = params['td'].value


class LongitudinalControlNode(Node):

    def parameters_callback(self, params):
        print(params)
        for param in params:
            if param.name == "log_level":
                self.logger.set_level(param.value)
            elif param.name == 'throttle.kp':
                self.th_params.kp = param.value
            elif param.name == 'throttle.ti':
                self.th_params.ti = param.value
            elif param.name == 'throttle.td':
                self.th_params.td = param.value
            elif param.name == 'brake.kp':
                self.br_params.kp = param.value
            elif param.name == 'brake.ti':
                self.br_params.ti = param.value
            elif param.name == 'brake.td':
                self.br_params.td = param.value
        return SetParametersResult(successful=True)

    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='LongitudinalControlNode', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        self.id_plataforma = vehicle_parameters['id_vehicle']
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False
        self.control_msg: PetConduccion = PetConduccion()
        self.current_speed = 0.
        self.speed_range = vehicle_parameters['speed']['range']
        self.telemetry = Telemetry()
        self.th_params = PID_params(self.get_parameters_by_prefix('throttle'))
        self.br_params = PID_params(self.get_parameters_by_prefix('brake'))
        self.pid_throttle = PID(kp=self.th_params.kp, ti=self.th_params.ti, td=self.th_params.td, anti_wind_up=0.3)
        self.pid_brake = PID(kp=self.br_params.kp, ti=self.br_params.ti, td=self.br_params.td, anti_wind_up=0.65)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_brake = self.create_publisher(msg_type=ControladorFloat, topic='Brake',
                                               qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_throttle = self.create_publisher(msg_type=ControladorFloat, topic='Throttle',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_pid_values_th = self.create_publisher(msg_type=Vector3, topic=f'{self.get_name()}/PID_throttle_values',
                                                       qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_pid_values_br = self.create_publisher(msg_type=Vector3, topic=f'{self.get_name()}/PID_brake_values',
                                                       qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=Telemetry, topic='Telemetry', callback=self.telemetry_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=PetConduccion, topic='Decision/Output', callback=self.decision_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        # # ##### Cliente del servicio de calibración del freno #####
        # self.cli_calibration = self.create_client(BrakeCalibration, 'brake_calibration')
        # while not self.cli_calibration.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.req_calibration = BrakeCalibration.Request()

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)
        self.timer_control = self.create_timer(1 / 10, self.control_loop)

    def telemetry_callback(self, telemetry: Telemetry):
        self.current_speed = telemetry.speed
        # self.telemetry = telemetry

    # # #### Función request para el servicio de calibración ####
    # def send_request(self):
    #     self.req_calibration.bool.data = bool(sys.argv[1])
    #     self.future = self.cli_calibration.call_async(self.req_calibration)

    def control_loop(self):
        # Normalizar valores
        target_speed_norm = interp(self.control_msg.speed, self.speed_range, [0, 1])
        current_speed_norm = interp(self.current_speed, self.speed_range, [0, 1])
        self.logger.debug(f'Target speed {round(self.control_msg.speed, 2)}\t{target_speed_norm}')
        self.logger.debug(f'Target speed {round(self.current_speed, 2)}\t{current_speed_norm}')
        # Calcular pids
        if self.control_msg.b_brake:
            brake, br_pid_values = self.pid_brake.calcValue(target_value=target_speed_norm,
                                                            current_value=current_speed_norm, kp=self.br_params.kp,
                                                            ti=self.br_params.ti, td=self.br_params.td)
            self.pub_pid_values_br.publish(Vector3(x=br_pid_values[0], y=br_pid_values[1], z=br_pid_values[2]))
        else:
            self.pid_brake.reset_values()
            brake = 0
        if self.control_msg.b_throttle:
            throttle, th_pid_values = self.pid_throttle.calcValue(target_value=target_speed_norm,
                                                                  current_value=current_speed_norm,
                                                                  kp=self.th_params.kp, ti=self.th_params.ti,
                                                                  td=self.th_params.td)
            self.pub_pid_values_th.publish(Vector3(x=th_pid_values[0], y=th_pid_values[1], z=th_pid_values[2]))
        else:
            self.pid_throttle.reset_values()
            throttle = 0
        self.logger.debug(f'PID Brake: {-brake}')
        self.logger.debug(f'PID Throttle: {throttle}')
        if self.control_msg.speed <= 0.1:
            self.logger.debug(f'Case frenada a 0')
            if self.current_speed < 0.5:
                brake_solution = -0.8
                throttle_solution = 0.
            else:
                brake_solution = brake
                throttle_solution = 0.
        elif self.control_msg.speed > self.current_speed:
            self.logger.debug('Caso aceleración')
            throttle_solution = throttle
            brake_solution = 0.
        else:
            self.logger.debug('Caso freno')
            if 0 >= self.control_msg.speed - self.current_speed >= -5 and self.current_speed > 10:
                self.logger.debug(f'Case frenada < 5km/h')
                brake_solution = 0.
                throttle_solution = 0.
            else:
                brake_solution = brake
                throttle_solution = 0.

        # elif self.control_msg.speed - self.current_speed >= -5 and self.control_msg.speed < 5:
        #     self.logger.debug(f'Case ')
        #     brake_solution = brake / 2
        #     throttle_solution = 0.
        # elif self.control_msg.speed - self.current_speed > -2:
        #     self.logger.debug(f'Case 3')
        #     brake_solution = 0.
        #     throttle_solution = throttle
        # elif self.control_msg.speed - self.current_speed >= -5:
        #     self.logger.debug(f'Case 4')
        #     brake_solution = 0.
        #     throttle_solution = 0.
        # else:
        #     brake_solution = brake
        #     self.logger.debug(f'Case 5')
        #     throttle_solution = 0.
        self.logger.debug(f'\nSolution: \n\tThrottle:\t{throttle_solution}\n\tBrake:\t\t{brake_solution}')
        self.pub_throttle.publish(ControladorFloat(
            header=Header(stamp=self.get_clock().now().to_msg()),
            enable=self.control_msg.b_throttle,
            target=float(throttle_solution)
        ))
        self.pub_brake.publish(ControladorFloat(
            header=Header(stamp=self.get_clock().now().to_msg()),
            enable=self.control_msg.b_brake,
            target=float(-brake_solution)
        ))
        # self.logger.debug(f'{self.control_msg.b_brake} Valor freno: {brake_solution}')

    def decision_callback(self, decision):
        self.control_msg = decision

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
    # TODO: Falta crear el cliente del servicio y mandar la petición
    try:
        manager = LongitudinalControlNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
