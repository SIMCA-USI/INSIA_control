import os

import rclpy
import yaml
from insia_msg.msg import Telemetry, StringStamped, PetConduccion
from numpy import interp
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from yaml.loader import SafeLoader

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
        self.control_msg:PetConduccion = PetConduccion()
        self.current_speed = 0.
        self.speed_range = self.get_parameter('speed_range').value
        th_params = self.get_parameters_by_prefix('throttle')
        br_params = self.get_parameters_by_prefix('brake')
        self.pid_throttle = PID(kp=th_params['kp'].value, ti=th_params['ti'], td=th_params['td'], anti_wind_up=0.1)
        self.pid_brake = PID(kp=br_params['kp'], ti=br_params['ti'], td=br_params['td'], anti_wind_up=0.2)

        self.pub_telemetry = self.create_publisher(msg_type=Telemetry,
                                                   topic='/' + vehicle_parameters['id_vehicle'] + '/Telemetry',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbit = self.create_publisher(msg_type=StringStamped,
                                                  topic='/' + vehicle_parameters['id_vehicle'] + '/Heartbit',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        # TODO: crear subscriber current speed
        # self.create_subscription(msg_type=OpenLoop,
        #                          topic='/' + vehicle_parameters['id_vehicle'] + '/Decision/Output',
        #                          callback=self.decision, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=PetConduccion,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/Decision/Output',
                                 callback=self.decision, qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbit = self.create_timer(1, self.publish_heartbit)
        self.timer_control = self.create_timer(1 / 10, self.control_loop)

    def control_loop(self):
        # Normalizar valores
        target_speed_norm = interp(self.control_msg.speed, self.speed_range, [0,1])
        current_speed_norm = interp(self.current_speed, self.speed_range, [0,1])

        # Calcular pids

        brake = self.pid_brake.calcValue(target_value=target_speed_norm, current_value=current_speed_norm)
        throttle = self.pid_throttle.calcValue(target_value=target_speed_norm, current_value=current_speed_norm)

        if self.control_msg.speed <= 0.1:
            if self.current_speed < 0.5:
                brake_solution = -0.8
                throttle_solution = 0.
            else:
                brake_solution = brake
                throttle_solution = 0.
        elif self.control_msg.speed - self.current_speed >= -5 and self.control_msg.speed < 5:
            brake_solution = brake / 2
            throttle_solution = 0.
        elif self.control_msg.speed - self.current_speed > -2:
            brake_solution = 0.
            throttle_solution = throttle
        elif self.control_msg.speed - self.current_speed >= -5:
            brake_solution = 0
            throttle_solution = 0.
        else:
            brake_solution = brake
            throttle_solution = 0.

        if self.control_msg.b_speed:
            pub_throttle.publish(throttle_solution)
            pub_brake.publish(-brake_solution)
        else:
            pub_brake.publish(0)
        self.logger.debug('Valor freno: {}'.format(brake_solution))

    def decision(self, decision):
        self.control = decision

    def publish_heartbit(self):
        msg = StringStamped(
            data=self.get_name()
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_heartbit.publish(msg)

    def shutdown(self):
        try:
            self.shutdown_flag = True
            self.timer_telemetry.cancel()
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
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
