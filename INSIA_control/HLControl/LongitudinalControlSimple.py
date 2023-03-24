import os

import rclpy
import yaml
from insia_msg.msg import Telemetry, StringStamped, PetConduccion, ControladorFloat
from numpy import interp
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from simple_pid import PID
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from yaml.loader import SafeLoader


class PID_params:
    def __init__(self, params):
        try:
            self.kp = params['kp'].value
            self.ti = params['ti'].value
            self.td = params['td'].value
        except:
            print('Error en el pid')
            exit(0)


class LongitudinalController(Node):
    def parameters_callback(self, params):
        for param in params:
            if param.name == "log_level":
                self.logger.set_level(param.value)
            elif param.name in ['throttle.kp', 'throttle.ti', 'throttle.td']:
                if param.name == 'throttle.kp':
                    self.th_params.kp = param.value
                elif param.name == 'throttle.ti':
                    self.th_params.ti = param.value
                elif param.name == 'throttle.td':
                    self.th_params.td = param.value
                self.set_accel_tunnings()
            elif param.name in ['brake.kp', 'brake.ti', 'brake.td']:
                if param.name == 'brake.kp':
                    self.br_params.kp = param.value
                elif param.name == 'brake.ti':
                    self.br_params.ti = param.value
                elif param.name == 'brake.td':
                    self.br_params.td = param.value
                self.set_brake_tunnings()
        return SetParametersResult(successful=True)

    def set_accel_tunnings(self):
        self.logger.debug(f'Accel tunnings modified: {self.th_params =}')
        self.accel_pid.tunings = (self.th_params.kp, self.th_params.ti, self.th_params.td)

    def set_brake_tunnings(self):
        self.logger.debug(f'Brake tunnings modified: {self.br_params =}')
        self.brake_pid.tunings = (self.br_params.kp, self.br_params.ti, self.br_params.td)

    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='LongitudinalControlNode', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False

        my_params = [
            ('log_level', 10, ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Ganancia proporcional para el controlador de aceleración'
            )),
            ('throttle.kp', 1.0, ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Ganancia proporcional para el controlador de aceleración'
            )),
            ('throttle.ti', 0.5, ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Tiempo integral para el controlador de aceleración'
            )),
            ('throttle.td', 0.2, ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Tiempo derivativo para el controlador de aceleración'
            )),
            ('brake.kp', 1.0, ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Ganancia proporcional para el controlador de frenado'
            )),
            ('brake.ti', 0.5, ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Tiempo integral para el controlador de frenado'
            )),
            ('brake.td', 0.2, ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Tiempo derivativo para el controlador de frenado'
            ))
        ]
        # Registrar los parámetros en el nodo
        # self.declare_parameters('', my_params)
        self.th_params = PID_params(self.get_parameters_by_prefix('throttle'))
        self.br_params = PID_params(self.get_parameters_by_prefix('brake'))
        self.speed_range = vehicle_parameters['speed']['range']
        # Inicialización de los controladores PID
        self.accel_pid = PID(self.th_params.kp, self.th_params.ti, self.th_params.td, setpoint=0, output_limits=(0, 1))
        self.brake_pid = PID(self.br_params.kp, self.br_params.ti, self.br_params.td, setpoint=0, output_limits=(0, 1))

        # Subscripción a los topics de telemetría y velocidad objetivo
        self.telemetry_sub = self.create_subscription(Telemetry, 'Telemetry', self.telemetry_callback,
                                                      HistoryPolicy.KEEP_LAST)
        self.target_sub = self.create_subscription(PetConduccion, 'Decision/Output', self.target_callback,
                                                   HistoryPolicy.KEEP_LAST)
        # Publicación de la señal de control
        self.brake_pub = self.create_publisher(msg_type=ControladorFloat, topic='Brake',
                                               qos_profile=HistoryPolicy.KEEP_LAST)
        self.throttle_pub = self.create_publisher(msg_type=ControladorFloat, topic='Throttle',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_pid_values_th = self.create_publisher(msg_type=Vector3, topic=f'{self.get_name()}/PID_throttle_values',
                                                       qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_pid_values_br = self.create_publisher(msg_type=Vector3, topic=f'{self.get_name()}/PID_brake_values',
                                                       qos_profile=HistoryPolicy.KEEP_LAST)

        # Variables para almacenar los valores de telemetría y velocidad objetivo
        self.telemetry: Telemetry = None
        self.target: PetConduccion = None
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)
        self.add_on_set_parameters_callback(self.parameters_callback)

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

    def telemetry_callback(self, msg: Telemetry):
        # Almacenar el valor de telemetría
        telemetry_speed = interp(msg.speed, self.speed_range, [0, 1])

        # Calcular la señal de control del PID
        if self.target is not None:
            target_speed = interp(self.target.speed, self.speed_range, [0, 1])
            error = target_speed - telemetry_speed
            accel_signal = 0
            brake_signal = 0
            if self.target.b_throttle and msg.brake < 10:
                accel_signal: float = self.accel_pid(-error)
                th_pid_values = self.accel_pid.components
                self.pub_pid_values_th.publish(
                    Vector3(x=float(th_pid_values[0]), y=float(th_pid_values[1]), z=float(th_pid_values[2])))
            else:
                self.accel_pid.reset()
            if self.target.b_brake:
                brake_signal = self.brake_pid(error)
                br_pid_values = self.brake_pid.components
                self.pub_pid_values_br.publish(
                    Vector3(x=float(br_pid_values[0]), y=float(br_pid_values[1]), z=float(br_pid_values[2])))
            else:
                self.brake_pid.reset()
            if accel_signal > brake_signal:
                # Publicar la señal de control en el topic /throttle
                brake_signal = 0
            else:
                # Publicar la señal de control en el topic /brake
                accel_signal = 0
            if self.target.speed == 0 and msg.speed < 1:
                brake_signal = 0.60
            self.throttle_pub.publish(ControladorFloat(
                header=Header(stamp=self.get_clock().now().to_msg()),
                enable=self.target.b_throttle,
                target=float(accel_signal)
            ))
            self.brake_pub.publish(ControladorFloat(
                header=Header(stamp=self.get_clock().now().to_msg()),
                enable=self.target.b_brake,
                target=float(brake_signal)
            ))

    def target_callback(self, msg: PetConduccion):
        # Almacenar el valor de velocidad objetivo
        self.target = msg

    def shutdown(self):
        try:
            self.shutdown_flag = True
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = LongitudinalController()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
