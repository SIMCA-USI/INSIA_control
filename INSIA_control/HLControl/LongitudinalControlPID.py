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


class LongitudinalControlPIDNode(Node):

    def parameters_callback(self, params):
        print(params)
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
        self.id_plataforma = vehicle_parameters['id_vehicle']
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False
        self.control_msg: PetConduccion = PetConduccion()
        self.current_speed = 0.
        self.speed_range = vehicle_parameters['speed']['range']
        pid_params: dict = self.get_parameters_by_prefix('speed')
        self.pid = PIDF(kp=pid_params['kp'].value, ti=pid_params['ti'].value, td=pid_params['td'].value,
                        anti_wind_up=0.1)
        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_speed = self.create_publisher(msg_type=ControladorFloat, topic='Speed',
                                               qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=Telemetry, topic='Telemetry', callback=self.telemetry_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=PetConduccion, topic='Decision/Output', callback=self.decision,
                                 qos_profile=HistoryPolicy.KEEP_LAST)
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)
        self.timer_control = self.create_timer(1 / 50, self.control_loop)

    def telemetry_callback(self, telemetry: Telemetry):
        self.current_speed = telemetry.speed

    def control_loop(self):
        if self.control_msg.b_throttle:
            self.logger.debug('Steering enabled')
            # Normalizar valores
            target_speed_norm = interp(self.control_msg.speed, self.speed_range, [0, 1.])
            self.logger.debug(f'{target_speed_norm =}')
            current_speed_norm = interp(self.current_speed, self.speed_range, [0., 1.])
            self.logger.debug(f'{current_speed_norm =}')
            # Calcular pids

            pid_params = self.get_parameters_by_prefix('speed')
            try:
                if target_speed_norm == 0 and current_speed_norm < 0.05:
                    target = -1
                else:
                    target, _ = self.pid.calcValue(target_value=target_speed_norm,
                                                current_value=current_speed_norm, kp=pid_params['kp'].value,
                                                ti=pid_params['ti'].value, td=pid_params['td'].value)
            except Exception as e:
                self.logger.error(f'Error calc pid: {e}')
            else:
                self.logger.debug(f'PID result {target}')
                self.pub_speed.publish(ControladorFloat(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    enable=self.control_msg.b_throttle,
                    target=float(target)
                ))
        else:
            self.logger.debug('Speed disabled')
            self.pid.reset_values()
            self.pub_speed.publish(ControladorFloat(
                header=Header(stamp=self.get_clock().now().to_msg()),
                enable=self.control_msg.b_throttle,
                target=0.
            ))

    def decision(self, decision):
        self.control_msg = decision

    def publish_heartbeat(self):
        """
        Heartbeat publisher to keep tracking every node
        :return: Publish on Heartbeat
        """
        msg = StringStamped(
            header=Header(stamp=self.get_clock().now().to_msg()),
            data=self.get_name()
        )
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
        manager = LongitudinalControlPIDNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
