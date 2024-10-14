
from rclpy.qos import HistoryPolicy
from INSIA_control.utils.utils import decoder_can, decoder_libreria
import rclpy
from rclpy.node import Node
import can
from insia_msg.msg import CAN
from insia_msg.srv import RequestCANMessage
from rclpy.qos import HistoryPolicy
import yaml
import os
from yaml.loader import SafeLoader


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

        # Crear el servicio
        self.srv = self.create_service(RequestCANMessage, 'request_can_messages', self.handle_can_request)

        # Notificador para recibir todos los mensajes del bus CAN
        self.notifier = can.Notifier(self.bus, [self.receive_message], timeout=0.00001)

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
            self.topic_publisher_dict[topic] = self.create_publisher(CAN, topic, qos_profile=HistoryPolicy.KEEP_LAST)

        response.success = True
        self.get_logger().info(f'Registrado CAN ID: {can_id} en topic: {topic}')
        return response

    def receive_message(self, can_frame):
        # Decodificar el mensaje CAN
        msg = self.decode_can(can_frame)
        can_id = msg.cobid

        # Publicar el mensaje en los topics correspondientes
        if can_id in self.can_id_dict:
            for topic in self.can_id_dict[can_id]:
                self.topic_publisher_dict[topic].publish(msg)
                self.get_logger().info(f'Publicado mensaje en {topic} para CAN ID: {can_id}')


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
