import rclpy
from rclpy.node import Node
import can
from insia_msg.msg import CAN, CANGroup
from rclpy.qos import HistoryPolicy
from INSIA_control.utils.utils import decoder_can, decoder_libreria
import yaml
import os
from yaml.loader import SafeLoader
from INSIA_control.utils.filtro import Decoder

class CanNode(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)

        super().__init__(node_name='can_node', namespace=vehicle_parameters['id_vehicle'], start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        #self.can_message_ids = vehicle_parameters['mensajes']
        self.bus = can.Bus(channel= self.get_parameter('can').value, bustype='socketcan')
        qos_profile = rclpy.qos.QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST)
        self.subscription = self.create_subscription(msg_type=CANGroup, topic='can_control',callback=self.can_control_callback,qos_profile=qos_profile)
        self.publisher = self.create_publisher(msg_type=CAN, topic='CAN', qos_profile=HistoryPolicy.KEEP_LAST)


        #self.timer = self.create_timer(0.0001, self.receive_can_message)
        self.notifier = can.Notifier(self.bus, [self.receive_can_message], timeout=0.00001)
    def can_control_callback(self, msg:CANGroup):

        for can_frame in msg.can_frames:
            self.get_logger().debug(f"Procesando CAN frame: {can_frame}")
            self.send_can_message(can_frame)

    def send_can_message(self, data:CAN):

        msg = can.Message(arbitration_id=data.cobid, data=data.msg_raw[4:12], is_extended_id=data.is_extended)
        #self.get_logger().info(f"Mensaje {msg}")
        try:
            self.bus.send(msg)
            #self.get_logger().info(f"Mensaje enviado al bus CAN: {msg} {type(msg) = }")
        except can.CanError as e:
            self.get_logger().error(f"Error al enviar el mensaje al bus CAN: {e}")

    def receive_can_message(self, msg:can.Message):
        try:
            if msg is not None:
                #arbitration_id = msg.arbitration_id
                # Comprobar si el arbitration_id está en la lista de mensajes CAN del YAML
                #if arbitration_id in self.can_message_ids:
                self.get_logger().info(f"Mensaje recibido con arbitration_id: {msg}")
                self.decode_can(msg)
            else:
                pass
        except can.CanError as e:
            self.get_logger().error(f"Error al recibir el mensaje del bus CAN: {e}")

    def decode_can(self, can_frame):
        self.get_logger().debug(f'can_frame es {can_frame}')
        _, data_raw, cobid, specifier, index, sub_index, data = decoder_libreria(msg=can_frame)  # TODO: cambiar a parametro

        msg = CAN(
            is_extended=False,  # TODO: cambiar a parametro
            cobid=cobid,
            specifier=specifier,
            index=index,
            sub_index=sub_index,
            data=data,
            msg_raw=data_raw
            )

        self.get_logger().debug(f'Decoded msg {hex(cobid)} {hex(index)} {hex(sub_index)} {data_raw}')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


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

