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
from simple_pid import PID
from geometry_msgs.msg import Vector3


class PID_params:
    def __init__(self, params):
        try:
            self.kp = params['kp'].value
            self.ti = params['ti'].value
            self.td = params['td'].value
        except:
            print('Error en el pid')
            exit(0)


class LateralControlPIDNode(Node):

    def parameters_callback(self, params):
        for param in params:
            if param.name == "log_level":
                self.logger.set_level(param.value)
            elif param.name in ['steering.kp', 'steering.ti', 'steering.td']:
                if param.name == 'steering.kp':
                    self.pid_params.kp = param.value
                elif param.name == 'steering.ti':
                    self.pid_params.ti = param.value
                elif param.name == 'steering.td':
                    self.pid_params.td = param.value
                self.set_pid_tunnings()
        return SetParametersResult(successful=True)

    def set_pid_tunnings(self):
        self.logger.debug(f'Steering tunnings modified: {self.pid_params =}')
        self.pid_steering.tunings = (self.pid_params.kp, self.pid_params.ti, self.pid_params.td)

    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='LongitudinalControlNode', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        self.id_plataforma: str = vehicle_parameters['id_vehicle']
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag: bool = False
        self.control_msg: PetConduccion = PetConduccion()
        self.current_steering: float = 0.
        self.steering_range = vehicle_parameters['steering']['wheel_range']
        self.pid_params = PID_params(self.get_parameters_by_prefix('steering'))
        self.pid_steering = PID(self.pid_params.kp, self.pid_params.ti, self.pid_params.td, setpoint=0,
                                output_limits=(-1, 1))

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_steering = self.create_publisher(msg_type=ControladorFloat, topic='Steering',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_pid_values = self.create_publisher(msg_type=Vector3, topic=f'{self.get_name()}/PID_steering_values',
                                                    qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=Telemetry, topic='Telemetry', callback=self.telemetry_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=PetConduccion, topic='Decision/Output', callback=self.decision_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)
        self.add_on_set_parameters_callback(self.parameters_callback)

    def telemetry_callback(self, telemetry: Telemetry):
        """
        Function to get current steering form vehicle telemetry
        :param telemetry: Vehicle telemetry basic
        :type telemetry: Telemetry
        :return: No return
        """
        self.current_steering = telemetry.steering
        steering_signal = 0.
        if self.control_msg.b_steering:
            self.logger.debug('Steering enabled')
            # Normalizar valores
            target_steering_norm = interp(self.control_msg.steering, self.steering_range, [-1., 1.])
            current_steering_norm = interp(self.current_steering, self.steering_range, [-1., 1.])
            self.logger.debug(f'Steering range {self.steering_range}')
            self.logger.debug(f'Target steering {self.control_msg.steering} {target_steering_norm}')
            self.logger.debug(f'Current steering {self.current_steering} {current_steering_norm}')
            # Calcular pids
            error = target_steering_norm - current_steering_norm
            steering_signal = self.pid_steering(-error)
            steering_pid_components = self.pid_steering.components
            self.pub_pid_values.publish(
                Vector3(x=float(steering_pid_components[0]), y=float(steering_pid_components[1]),
                        z=float(steering_pid_components[2])))
            self.logger.debug(f'PID result {steering_signal}')
        else:
            self.logger.debug('Steering disabled')
            self.pid_steering.reset()

        self.pub_steering.publish(ControladorFloat(
            header=Header(stamp=self.get_clock().now().to_msg()),
            enable=self.control_msg.b_steering,
            target=float(steering_signal)
        ))

    def decision_callback(self, decision: PetConduccion):
        """
        Decision callback to get target steering and b_steering
        :param decision: PetConduccion from low level decision
        :type decision: PetConduccion
        :return: No return
        """
        self.control_msg = decision

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
