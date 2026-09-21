import traceback
from traceback import format_exc

import rclpy
from can_msgs.msg import Frame
from insia_msg.msg import CAN, CANGroup
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header

from INSIA_control.utils.utils import decoder_can


class CANDecoderNode(Node):

    def parameters_callback(self, params):
        for param in params:
            if param.name == "log_level":
                self.logger.set_level(param.value)
        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__('can_decoder_node', start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=20))
        self.logger.set_level(self._log_level.value)

        self.add_on_set_parameters_callback(self.parameters_callback)



        # Publicador para el mensaje CAN procesado
        self.pub_CAN = self.create_publisher(CAN, 'CAN', 10)
        self.can_connected = self.get_parameter_or('can', Parameter(name='can', value='can0')).value
        self.extended = self.get_parameter_or('extend', Parameter(name='extend', value=True)).value
        extend: True  # Puedes ajustar si usáis IDs extendidos

        # Subscripciones a can0 y can1
        self.create_subscription(Frame, f'CAN/{self.can_connected}/receive', self.callback_can, 10)

        self.pub_can = self.create_publisher(Frame, f'CAN/{self.can_connected}/transmit', 10)

        self.create_subscription(msg_type=CANGroup, topic=self.get_name(), callback=self.send_msgs,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.logger.info(f'Escuchando /CAN/{self.can_connected}/receive')

    def send_msgs(self, data: CANGroup):
        if len(data.can_frames) == 0:
            data.can_frames = [CAN()]
        for frame in data.can_frames:
            frame: CAN
            f = Frame(
                header=Header(stamp=self.get_clock().now().to_msg()),
                id=int((frame.msg_raw[2] << 8) + frame.msg_raw[3]),
                is_extended=frame.is_extended,
                dlc=8,
                data=frame.msg_raw[4:-1]
            )
            self.pub_can.publish(f)

    def callback_can(self, msg):
        self.logger.debug('Mensaje recibido en CAN0')
        self.handle_frame(msg)

    def handle_frame(self, msg: Frame):
        # Convertir Frame a bytearray compatible con decoder_can
        try:
            can_id = msg.id
            data = bytes(msg.data)
            can_frame = can_id.to_bytes(4, byteorder='big')  # Dummy para 2 bytes iniciales
            can_frame += data
            can_frame += bytes([0x08])  # Final según tu protocolo
            self.decode_can(can_frame)
        except Exception as e:
            self.logger.error(f"Error al convertir Frame a bytearray: {e}")
            traceback.print_exc()

    def decode_can(self, can_frame):
        try:
            _, data_raw, cobid, specifier, index, sub_index = decoder_can(msg=can_frame, extended=self.extended)
            data = bytearray(can_frame[8:-1])  # Extraer los datos útiles

            msg = CAN(
                is_extended=self.extended,
                cobid=cobid,
                specifier=specifier,
                index=index,
                sub_index=sub_index,
                data=data,
                msg_raw=bytearray(can_frame),
                header=Header(stamp=self.get_clock().now().to_msg())
            )
            """
            self.logger.info(
                f'Decoded: COB-ID={hex(cobid)}, Index={hex(index)}, SubIndex={sub_index}, Data={list(data)}'
            )"""
            self.pub_CAN.publish(msg)

        except Exception as e:
            self.logger.error(f'Error decodificando CAN: {e}')
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = CANDecoderNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        format_exc()
        print(e)
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
