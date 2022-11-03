import os

import rclpy
import yaml
from insia_msg.msg import Telemetry, StringStamped, PetConduccion, ControladorFloat
from numpy import interp
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from yaml.loader import SafeLoader
from std_msgs.msg import Header

from INSIA_control.utils.pid import PIDF


class LateralControlPIDNode(Node):
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
        self.current_steering = 0.
        self.steering_range = vehicle_parameters['steering']['wheel_range']
        pid_params = self.get_parameters_by_prefix('steering')
        self.pid_steering = PIDF(kp=pid_params['kp'].value, ti=pid_params['ti'].value, td=pid_params['td'].value,
                                 anti_wind_up=0.1)

        self.pub_heartbit = self.create_publisher(msg_type=StringStamped,
                                                  topic='/' + vehicle_parameters['id_vehicle'] + '/Heartbit',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_steering = self.create_publisher(msg_type=ControladorFloat,
                                                  topic='/' + vehicle_parameters['id_vehicle'] + '/Steering',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=Telemetry,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/Telemetry',
                                 callback=self.telemetry_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=PetConduccion,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/Decision/Output',
                                 callback=self.decision, qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbit = self.create_timer(1, self.publish_heartbit)
        self.timer_control = self.create_timer(1 / 50, self.control_loop)

    def telemetry_callback(self, telemetry: Telemetry):
        self.current_steering = telemetry.steering

    def control_loop(self):
        if self.control_msg.b_steering:
            self.logger.debug('Steering enabled')
            # Normalizar valores
            target_steering_norm = interp(self.control_msg.steering, self.steering_range, [-1., 1.])
            current_steering_norm = interp(self.current_steering, self.steering_range, [-1., 1.])
            self.logger.debug(f'Steering range {self.steering_range}')
            self.logger.debug(f'Target steering {self.control_msg.steering} {target_steering_norm}')
            self.logger.debug(f'Current steering {self.current_steering} {current_steering_norm}')
            # Calcular pids
            pid_params = self.get_parameters_by_prefix('steering')
            steering = -self.pid_steering.calcValue(target_value=target_steering_norm,
                                                    current_value=current_steering_norm, kp=pid_params['kp'].value,
                                                    ti=pid_params['ti'].value, td=pid_params['td'].value)
            self.logger.debug(f'PID result {steering}')
            self.pub_steering.publish(ControladorFloat(
                header=Header(stamp=self.get_clock().now().to_msg()),
                enable=self.control_msg.b_steering,
                target=float(steering)
            ))
        else:
            self.logger.debug('Steering disabled')
            self.pid_steering.reset_values()
            self.pub_steering.publish(ControladorFloat(
                header=Header(stamp=self.get_clock().now().to_msg()),
                enable=self.control_msg.b_steering,
                target=0.
            ))

    def decision(self, decision):
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
    try:
        manager = LateralControlPIDNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
