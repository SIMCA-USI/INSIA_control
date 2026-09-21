import os
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.parameter import Parameter
from std_msgs.msg import Header

from can_msgs.msg import Frame
from insia_msg.msg import CAN, ModoMision

from INSIA_control.utils.filtro import Decoder
from yaml.loader import SafeLoader
from traceback import format_exc
from rcl_interfaces.msg import SetParametersResult

class ManualModeNode(Node):

    

    def __init__(self):
        # Parametros del vehiculo
        super().__init__(
            node_name='ManualModeNode',
            start_parameter_services=True,
            allow_undeclared_parameters=False,
            automatically_declare_parameters_from_overrides=True
        )

        
        self.declare_parameter("enable_brake", True)
        self.declare_parameter("enable_throttle", True)
        self.declare_parameter("enable_steering", True)
        self.modoActual = ModoMision.MANUAL
        # Logger
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or(
            'log_level',
            Parameter(name='log_level', value=10)
        )
        self.logger.set_level(self._log_level.value)
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Decoder para los mensajes CAN
        self.decoder = Decoder(dictionary='ravo_manual.yaml')

        # Publisher
        self.pub_mode = self.create_publisher(
            msg_type=ModoMision,
            topic='Mode',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        # Suscripciones a CAN
        self.create_subscription(
            msg_type=CAN,
            topic='CAN',
            callback=self.msg_can,
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.create_subscription(
            msg_type=ModoMision,
            topic='Decision/Status',
            callback=self.ver_modo,
            qos_profile=HistoryPolicy.KEEP_LAST
        )

    def parameters_callback(self, params):
        """
        Callback para actualizar parámetros dinámicos
        """
        for param in params:
            if param.name == "log_level":
                self.logger.set_level(param.value)
            elif param.name == "enable_brake":
                self.enable_brake = param.value
                self.logger.info(f"Parametro enable_brake cambiado a {self.enable_brake}")
            elif param.name == "enable_throttle":
                self.enable_throttle = param.value
                self.logger.info(f"Parametro enable_throttle cambiado a {self.enable_throttle}")
            elif param.name == "enable_steering":
                self.enable_steering = param.value
                self.logger.info(f"Parametro enable_steering cambiado a {self.enable_steering}")
        return SetParametersResult(successful=True)

        
    def ver_modo(self, msg:ModoMision):
        self.modoActual = msg.modo_mision
        self.logger.debug(f"Modo actual recibido: {self.modoActual}")



    def msg_can(self, msg):
        """
        Callback de los mensajes CAN: decodificamos y miramos si son de freno, acelerador o volante.
        """
        try:
            name, value = self.decoder.decode(msg)
            self.logger.debug(f"Decoded {name}: {value}")

            enabled = {
            "brake": self.get_parameter("enable_brake").get_parameter_value().bool_value,
            "throttle": self.get_parameter("enable_throttle").get_parameter_value().bool_value,
            "steering": self.get_parameter("enable_steering").get_parameter_value().bool_value,
            }      

            if name in enabled and enabled [name] and value == 1:  # los nombres exactos dependen de tu yaml
                self.logger.info(f"Se detectó manipulación manual: {name}")
                if self.modoActual != ModoMision.MANUAL:
                    self.publish_manual_mode()

        except ValueError as e:
            self.logger.debug(f'{e}')
    
    def publish_manual_mode(self):
        msg = ModoMision()
        msg.modo_mision = ModoMision.MANUAL

        self.pub_mode.publish(msg)
        self.logger.info(f"Modo Manual publicado en {self.get_namespace}/Mode")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ManualModeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Manual Trigger: Keyboard interrupt')
    except Exception as e:
        print(format_exc())
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()