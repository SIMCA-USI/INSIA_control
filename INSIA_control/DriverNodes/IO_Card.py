import os

import rclpy
import yaml
from insia_msg.msg import IOAnalogue, EPOSDigital
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from yaml.loader import SafeLoader

from INSIA_control.utils.connection import Connection, Local_Connection


class IOCard(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='IOCard', namespace=vehicle_parameters['id_vehicle'], start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False

        self.connection_mode = 'tcp'
        self.ip = '192.168.0.7'
        # self.port = self.get_parameter('port').value
        self.local = self.get_parameter('local').value

        if self.local:
            self.logger.info(f'Running in local mode')
            self.cli_DIO = Local_Connection()
            self.cli_AIO = Local_Connection()
        else:
            self.logger.info(f'Running in remote mode')
            self.cli_DIO = Connection(name=f'Connection {self.ip}:4601', mode=self.connection_mode,
                                      ip=self.ip, port=4601, deco_function=self.deco_function,
                                      log_level=self._log_level.value)
            self.cli_AIO = Connection(name=f'Connection {self.ip}:4600', mode=self.connection_mode,
                                      ip=self.ip, port=4600, deco_function=self.deco_function,
                                      log_level=self._log_level.value)

        self.create_subscription(msg_type=IOAnalogue, topic=self.get_name() + '/ioanalogue',
                                 callback=self.callback_analog, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=EPOSDigital, topic=self.get_name() + '/iodigital',
                                 callback=self.callback_digital, qos_profile=HistoryPolicy.KEEP_LAST)

    def deco_function(self, data):
        pass

    def callback_analog(self, msg):
        if not self.local:
            if self.is_aio_connected():
                value = '+{:06.3f}'.format(msg.voltage)
                comando = '#01' + str(msg.channel) + value + '\r\n'
                self.cli_AIO.send(comando.encode())
                self.logger.debug(f' Mensaje AIO enviado')
        else:
            self.logger.debug(f'')

    def callback_digital(self, msg):
        value = ""
        if not self.local:
            if self.is_dio_connected():
                if msg.enable:
                    value = "0" + str(msg.io_digital) + "0\r\n"
                else:
                    value = "0" + str(msg.io_digital) + "1\r\n"
            self.cli_DIO.send(value.encode())
        log = 'ON' if msg.enable else 'OFF'
        self.logger.debug(f' IO {msg.io_digital} {log}')

    def is_dio_connected(self):
        if self.cli_DIO.connected:
            self.logger.debug(f' Cliente DIO conectado {self.cli_DIO.connected}')
        else:
            self.logger.debug(f' Cliente DIO desconectado {self.cli_DIO.connected}')

        return self.cli_DIO.connected

    def is_aio_connected(self):
        if self.cli_AIO.connected:
            self.logger.debug(f' Cliente AIO conectado {self.cli_AIO.connected}')
        else:
            self.logger.debug(f' Cliente AIO desconectado {self.cli_AIO.connected}')

        return self.cli_AIO.connected

    def shutdown(self):
        try:
            self.shutdown_flag = True
            for _ in range(3):
                rclpy.spin_once(self)
            self.cli_AIO.shutdown()
            self.cli_DIO.shutdown()
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = IOCard()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
