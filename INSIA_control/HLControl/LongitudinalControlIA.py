import os
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'

import rclpy
import yaml
from insia_msg.msg import Telemetry, StringStamped, PetConduccion, ControladorFloat
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader
import tensorflow as tf

import numpy as np
import joblib


class LongitudinalController(Node):

    def neural_network_predict(self, velocidad_actual, velocidad_objetivo_10, resta_velocidad_5, giro_volante_10, resta_altitud_10):

        datos_coche = np.column_stack((velocidad_actual, velocidad_objetivo_10, resta_velocidad_5, giro_volante_10, resta_altitud_10))
        datos_coche_scaled = self.scaler.transform(datos_coche)

        datos_coche_scaled_expanded = np.expand_dims(datos_coche_scaled, axis=-1)

        clasificacion, magnitud = self.modelo_cargado.predict(datos_coche_scaled_expanded)
        return clasificacion, magnitud

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
        #self.scaler = joblib.load('/home/simca/Escritorio/Javi/modelos_red_neuronal2/scaler_nuevo.pkl')
        self.scaler = joblib.load('/home/simca/Escritorio/Javi/modelos_red_neuronal2/mucho_frenado.pkl')
        self.modelo_cargado = tf.keras.models.load_model('/home/simca/Escritorio/Javi/modelos_red_neuronal2/mucho_frenado.h5')
        #self.modelo_cargado = tf.keras.models.load_model('/home/simca/Escritorio/Javi/modelos_red_neuronal2/modelo_entrenado_nuevo.h5')


        # Subscripción a los topics de telemetría, gps y velocidad objetivo
        self.telemetry_sub = self.create_subscription(Telemetry, 'Telemetry', self.telemetry_callback,
                                                      HistoryPolicy.KEEP_LAST)
        self.target_sub = self.create_subscription(PetConduccion, 'Decision/Output', self.target_callback,
                                                   HistoryPolicy.KEEP_LAST)
        self.gps_fix = self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback,
                                                      HistoryPolicy.KEEP_LAST)
        # Publicación de la señal de control
        self.brake_pub = self.create_publisher(msg_type=ControladorFloat, topic='Brake',
                                               qos_profile=HistoryPolicy.KEEP_LAST)
        self.throttle_pub = self.create_publisher(msg_type=ControladorFloat, topic='Throttle',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        # Variables para almacenar los valores de telemetría y velocidad objetivo
        self.speed = None
        self.speed_history = []
        self.steering_history = []
        self.altitude_history = []
        self.resta_altitud_10 = 0
        self.resta_velocidad_5 = 0
        self.giro_volante_10 = 0

        self.full_brake = False
        self.status_brake = False
        self.obj_speed = 0

        self.target: PetConduccion = None
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

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
        self.speed = msg.speed
        self.speed_history.append(msg.speed)
        if len(self.speed_history) > 5:
            self.speed_history = self.speed_history[-5:]
        self.resta_velocidad_5 = self.speed_history[-1] - self.speed_history[0]

        self.steering_history.append(msg.steering_deg)
        if len(self.steering_history) > 10:
            self.steering_history = self.steering_history[-10:]
        self.giro_volante_10 = self.steering_history[-1] - self.steering_history[0]


    def gps_callback(self, msg:NavSatFix):
        self.altitude_history.append(msg.altitude)
        if len(self.altitude_history) > 10:
            self.altitude_history = self.altitude_history[-10:]
        self.resta_altitud_10 = self.altitude_history[-1] - self.altitude_history[0]

        if self.target is not None and self.speed is not None and not self.shutdown_flag:

            if self.obj_speed - self.speed > 5:
                self.obj_speed = self.speed + 5
            if self.obj_speed - self.speed < -20:
                self.obj_speed = self.speed - 20

            predicciones_clasificacion, predicciones_magnitud = self.neural_network_predict(self.speed, self.obj_speed,
                                                                                            self.resta_velocidad_5,
                                                                                               self.giro_volante_10,
                                                                                            self.resta_altitud_10)
            clase_predicha = np.argmax(predicciones_clasificacion)
            magnitud = predicciones_magnitud



            if magnitud > 77.0:
                magnitud = 77.0
            if magnitud < 0.0:
                magnitud = 0.0

            if self.obj_speed != 0. and self.full_brake:
                self.full_brake = False

            if self.obj_speed == 0. and self.speed < 6. and not self.full_brake:
                self.full_brake = True
                self.status_brake = magnitud

            if not self.full_brake:
                if clase_predicha == 0:
                    magnitud = magnitud * 2
                    self.brake_pub.publish(ControladorFloat(
                        header=Header(stamp=self.get_clock().now().to_msg()),
                        enable=self.target.b_brake,
                        target=float(magnitud/77.0)
                    ))
                    self.throttle_pub.publish(ControladorFloat(
                        header=Header(stamp=self.get_clock().now().to_msg()),
                        enable=self.target.b_throttle,
                        target=0.
                    ))
                elif clase_predicha == 1:
                    self.throttle_pub.publish(ControladorFloat(
                        header=Header(stamp=self.get_clock().now().to_msg()),
                        enable=self.target.b_throttle,
                        target=float(magnitud/100.0)
                    ))
                    self.brake_pub.publish(ControladorFloat(
                        header=Header(stamp=self.get_clock().now().to_msg()),
                        enable=self.target.b_brake,
                        target=0.
                    ))
                elif clase_predicha == 2:
                    self.brake_pub.publish(ControladorFloat(
                        header=Header(stamp=self.get_clock().now().to_msg()),
                        enable=self.target.b_brake,
                        target=0.
                    ))
                    self.throttle_pub.publish(ControladorFloat(
                        header=Header(stamp=self.get_clock().now().to_msg()),
                        enable=self.target.b_throttle,
                        target=0.
                    ))
            else:
                self.status_brake += 0.05
                if self.status_brake > 0.75:
                    self.status_brake = 0.75
                self.throttle_pub.publish(ControladorFloat(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    enable=self.target.b_throttle,
                    target=0.))
                self.brake_pub.publish(ControladorFloat(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    enable=self.target.b_brake,
                    target=self.status_brake))

    def target_callback(self, msg: PetConduccion):
        # Almacenar el valor de velocidad objetivo
        self.target = msg
        self.obj_speed = msg.speed

    def shutdown(self):
        try:
            self.shutdown_flag = True
            self.brake_pub.publish(ControladorFloat(
                header=Header(stamp=self.get_clock().now().to_msg()),
                enable=False,
                target=0.
            ))
            self.throttle_pub.publish(ControladorFloat(
                header=Header(stamp=self.get_clock().now().to_msg()),
                enable=False,
                target=0.
            ))
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
