import os

import rclpy
import yaml
from insia_msg.msg import Telemetry, StringStamped, PetConduccion, ControladorFloat
from numpy import interp
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader

from INSIA_control.utils.pid import PIDF


class LateralControlPIDNode(Node):

    def parameters_callback(self, params: list):
        """
        Parameter callback to update desired parameters on the fly
        :param params: Params from ROS
        :type params: list
        :return: Return new set of params
        :rtype: SetParametersResult
        """
        for param in params:
            if param.name == "log_level":
                self.logger.set_level(param.value)
        return SetParametersResult(successful=True)

    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='LongitudinalControlNode', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        self.id_plataforma: str = vehicle_parameters['id_vehicle']
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag: bool = False
        self.control_msg: PetConduccion = PetConduccion()
        self.current_steering: float = 0.
        self.steering_range = vehicle_parameters['steering']['wheel_range']
        pid_params: dict = self.get_parameters_by_prefix('steering')
        self.pid_steering = PIDF(kp=pid_params['kp'].value, ti=pid_params['ti'].value, td=pid_params['td'].value,
                                 anti_wind_up=0.3)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_steering = self.create_publisher(msg_type=ControladorFloat, topic='Steering',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=Telemetry, topic='Telemetry', callback=self.telemetry_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=PetConduccion, topic='Decision/Output', callback=self.decision_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)
        self.timer_control = self.create_timer(1 / 50, self.control_loop)

    def telemetry_callback(self, telemetry: Telemetry):
        """
        Function to get current steering form vehicle telemetry
        :param telemetry: Vehicle telemetry basic
        :type telemetry: Telemetry
        :return: No return
        """
        self.current_steering = telemetry.steering

    def decision_callback(self, decision: PetConduccion):
        """
        Decision callback to get target steering and b_steering
        :param decision: PetConduccion from low level decision
        :type decision: PetConduccion
        :return: No return
        """
        self.control_msg = decision

    def control_loop(self):
        """
        Thread of control loop to calculate pid
        """
        if self.control_msg.b_steering:
            self.logger.debug('Steering enabled')
            # Normalizar valores
            target_steering_norm = interp(self.control_msg.steering, self.steering_range, [-1., 1.])
            current_steering_norm = interp(self.current_steering, self.steering_range, [-1., 1.])
            self.logger.debug(f'Steering range {self.steering_range}')
            self.logger.debug(f'Target steering {self.control_msg.steering} {target_steering_norm}')
            self.logger.debug(f'Current steering {self.current_steering} {current_steering_norm}')
            # Calcular pids
            pid_params: dict = self.get_parameters_by_prefix('steering')
            steering = self.pid_steering.calcValue(target_value=target_steering_norm,
                                                      current_value=current_steering_norm, kp=pid_params['kp'].value,
                                                      ti=pid_params['ti'].value, td=pid_params['td'].value)
            steering = -steering
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
