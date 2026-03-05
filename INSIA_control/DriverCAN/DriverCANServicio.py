
from rclpy.qos import HistoryPolicy
from INSIA_control.utils.utils import decoder_can, decoder_libreria
import rclpy
from rclpy.node import Node
import can
from rclpy.parameter import Parameter

from insia_msg.msg import CAN, CANGroup
from insia_msg.srv import RequestCANMessage
from rclpy.qos import HistoryPolicy
import yaml
import os
from yaml.loader import SafeLoader
from time import sleep


class CanNode(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)

        super().__init__('can_node', namespace=vehicle_parameters['id_vehicle'], start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.bus = can.Bus(channel=self.get_parameter('can').value, bustype='socketcan')
        self.can_id_dict = {}
        self.topic_publisher_dict = {}
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        # Crear el servicio
        self.srv = self.create_service(RequestCANMessage, self.get_name() + '/request_can_messages', self.handle_can_request)
        self.listener = self.get_parameter_or('listener', Parameter(name='listener', value=False)).value
        # Notificador para recibir todos los mensajes del bus CAN
        self.notifier = can.Notifier(self.bus, [self.receive_message], timeout=0.00001)
        self.create_subscription(msg_type=CANGroup, topic=self.get_name(), callback=self.save_msg,
                                 qos_profile=HistoryPolicy.KEEP_LAST)
        self.logger.debug(f'Listener mode = {self.listener}')
        if self.listener:
            self.pub_CAN = self.create_publisher(msg_type=CAN, topic='CAN', qos_profile=HistoryPolicy.KEEP_LAST)

    def save_msg(self, msg: CANGroup):
        for can_frame in msg.can_frames:
            self.send_can_message(can_frame)

    def send_can_message(self, data: CAN):
        #self.logger.error(f'Driver CAN send: {data.msg_raw}')
        msg = can.Message(arbitration_id=data.cobid, data=data.msg_raw[4:12], is_extended_id=data.is_extended)
        try:
            self.bus.send(msg)
            # self.get_logger().info(f"Mensaje enviado al bus CAN: {msg} {type(msg) = }")
        except can.CanError as e:
            self.get_logger().error(f"Error al enviar el mensaje al bus CAN: {e}")

    def handle_can_request(self, request, response):
        can_id = request.can_id
        topic = request.topic

        # Agregar can_id y topic al primer diccionario
        if can_id not in self.can_id_dict:
            self.can_id_dict[can_id] = []
        if topic not in self.can_id_dict[can_id]:
            self.can_id_dict[can_id].append(topic)

        # Crear el publisher si no existe ya
        if topic not in self.topic_publisher_dict:
            self.topic_publisher_dict[topic] = self.create_publisher(CAN, topic + '/CAN', qos_profile=HistoryPolicy.KEEP_LAST)

        response.success = True

        return response

    def receive_message(self, can_frame:can.Message):
        # Decodificar el mensaje CAN
        can_id = can_frame.arbitration_id
        # Publicar el mensaje en los topics correspondientes
        if can_id in self.can_id_dict.keys():
            for topic in self.can_id_dict[can_id]:
                msg = self.decode_can(can_frame)
                self.topic_publisher_dict[topic].publish(msg)
        if self.listener:
            msg = self.decode_can(can_frame)
            self.pub_CAN.publish(msg)


    def decode_can(self, can_frame):
        # Decodificación del mensaje CAN
        _, data_raw, cobid, specifier, index, sub_index, data = decoder_libreria(msg=can_frame)

        msg = CAN(
            is_extended=False,
            cobid=cobid,
            specifier=specifier,
            index=index,
            sub_index=sub_index,
            data=data,
            msg_raw=data_raw
        )

        msg.header.stamp = self.get_clock().now().to_msg()
        return msg


def main(args=None):
    rclpy.init(args=args)
    can_node = CanNode()
    try:
        rclpy.spin(can_node)
    except KeyboardInterrupt:
        pass
    can_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
