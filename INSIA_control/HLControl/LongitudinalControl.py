import os
import sys

import rclpy
import yaml
from insia_msg.msg import Telemetry, StringStamped, PetConduccion, ControladorFloat
from insia_msg.srv import BrakeCalibration
from numpy import interp
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from yaml.loader import SafeLoader
from std_msgs.msg import Header

from INSIA_control.utils.pid import PID


class LongitudinalControlNode(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='LongitudinalControlNode', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        self.id_plataforma = vehicle_parameters['id_vehicle']
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False
        self.control_msg: PetConduccion = PetConduccion()
        self.current_speed = 0.
        self.speed_range = self.get_parameter('speed_range').value
        self.telemetry = Telemetry()
        th_params = self.get_parameters_by_prefix('throttle')
        br_params = self.get_parameters_by_prefix('brake')
        self.pid_throttle = PID(kp=th_params['kp'].value, ti=th_params['ti'].value, td=th_params['td'].value,
                                anti_wind_up=0.1)
        self.pid_brake = PID(kp=br_params['kp'].value, ti=br_params['ti'].value, td=br_params['td'].value,
                             anti_wind_up=0.2)

        self.pub_heartbit = self.create_publisher(msg_type=StringStamped,
                                                  topic='/' + vehicle_parameters['id_vehicle'] + '/Heartbit',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_brake = self.create_publisher(msg_type=ControladorFloat,
                                               topic='/' + vehicle_parameters['id_vehicle'] + '/Brake',
                                               qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_throttle = self.create_publisher(msg_type=ControladorFloat,
                                                  topic='/' + vehicle_parameters['id_vehicle'] + '/Throttle',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=Telemetry,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/Telemetry',
                                 callback=self.telemetry_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=PetConduccion,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/Decision/Output',
                                 callback=self.decision_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        # # ##### Cliente del servicio de calibración del freno #####
        # self.cli_calibration = self.create_client(BrakeCalibration, 'brake_calibration')
        # while not self.cli_calibration.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.req_calibration = BrakeCalibration.Request()

        self.timer_heartbit = self.create_timer(1, self.publish_heartbit)
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
            brake = self.pid_brake.calcValue(target_value=target_speed_norm, current_value=current_speed_norm)
        else:
            self.pid_brake.reset_values()
            brake = 0
        if self.control_msg.b_throttle:
            throttle = self.pid_throttle.calcValue(target_value=target_speed_norm, current_value=current_speed_norm)
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
            brake_solution=0.
        else:
            self.logger.debug('Caso freno')
            if 0 >= self.control_msg.speed - self.current_speed >= -5 and self.current_speed > 10:
                self.logger.debug(f'Case frenada < 5km/h')
                brake_solution = 0.
                throttle_solution = 0.
            else:
                brake_solution=brake
                throttle_solution=0.

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

    def publish_heartbit(self):
        msg = StringStamped(
            data=self.get_name()
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_heartbit.publish(msg)

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
        print('Node: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
