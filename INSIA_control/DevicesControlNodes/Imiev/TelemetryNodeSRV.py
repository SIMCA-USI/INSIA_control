import os

import rclpy
import yaml
from std_msgs.msg import Header
from insia_msg.msg import CAN, Telemetry, StringStamped, CANGroup
from insia_msg.srv import RequestCANMessage


from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from yaml.loader import SafeLoader

from INSIA_control.utils.filtro import Decoder
from INSIA_control.utils.utils import convert_types


class VehicleNode(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='VehicleDecoder', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.id_plataforma = vehicle_parameters['id_vehicle']
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False
        self.decoder = Decoder(dictionary=self.get_parameter('dictionary').value)

        self.can_name = self.get_parameter_or('can_recepcion', Parameter(name='can_recepcion', value='CAN')).value

        self.client = self.create_client(RequestCANMessage, self.can_name + '/request_can_messages')
        self.alta_de_mensajes = self.get_parameter_or('AltaDeMensajes', Parameter(name='AltaDeMensajes', value=True))
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.info('Esperando por el servicio de request_can_messages...')

        self.vehicle_state = {}
        self.steering_wheel_conversion = vehicle_parameters['steering']['steering_wheel_conversion']

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_telemetry = self.create_publisher(msg_type=Telemetry, topic='Telemetry',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=CAN, topic=self.get_name() +'/CAN', callback=self.msg_can, qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_telemetry = self.create_timer(1 / 20, self.publish_telemetry)
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

        if self.alta_de_mensajes:
            self.request_can_messages()

    def create_msg_Telemetry(self):
        self.logger.error(f'Create msg telemetry  {self.vehicle_state = }')
        msg = Telemetry()
        fields = msg.get_fields_and_field_types()
        for field in fields.keys():
            data = self.vehicle_state.get(field)
            if data is not None:
                if field == 'throttle':
                    data= max(data,0)
                try:
                    setattr(msg, field, convert_types(ros2_type=fields.get(field), data=data))
                except Exception as e:
                    self.logger.error(f'Error setting atributes {e}')

        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.id_plataforma = self.id_plataforma
        #msg.brake = 0
        msg.steering_deg = msg.steering / self.steering_wheel_conversion
        # msg.brake = int((int(msg.brake / 0.25) & 0x0FFF) * 0.25)
        self.logger.error(f'{msg = }')
        return msg

    def request_can_messages(self):
        for can_id in self.decoder.dic_parameters.keys():
            self.logger.error(f'Request can message {hex(int(can_id))}')
            self.send_request(can_id)

    def send_request(self, can_id):

        request = RequestCANMessage.Request()
        request.can_id = int(can_id)
        request.topic = self.get_name()

        future = self.client.call_async(request)

        # if future.result() is not None:
        #     self.get_logger().info(f'Mensaje CAN {can_id} solicitado exitosamente.')
        # else:
        #     self.get_logger().error(f'Error al solicitar el mensaje CAN {can_id}.')

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

    def publish_telemetry(self):
        self.pub_telemetry.publish(msg=self.create_msg_Telemetry())

    def msg_can(self, msg):
        try:
            name, value = self.decoder.decode(msg)
            self.vehicle_state.update({name: value})
            self.logger.debug(f'Decoded {name}: {value}')
            self.logger.error(f'{msg =} {type(msg) = }')
        except ValueError as e:
            self.logger.debug(f'{e}')

    def shutdown(self):
        try:
            self.shutdown_flag = True
            self.timer_telemetry.cancel()
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = VehicleNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
