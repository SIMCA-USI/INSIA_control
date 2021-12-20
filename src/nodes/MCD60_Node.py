import os

import rclpy
import yaml
from insia_msg.msg import CAN, CANGroup, StringStamped, EPOSConsigna, EPOSDigital, BoolStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader

import src.utils.epos as epos


class MCD60_Node(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='MCD60', namespace=vehicle_parameters['id_vehicle'], start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False

        self.cobid = self.get_parameter('cobid').value
        self.msgs_list = self.get_parameter('msgs').value
        self.op_mode = self.get_parameter('mode').value
        self.can_conected = self.get_parameter('can').value
        self.motor_status = None
        self.digital_outputs = {1: False, 2: False, 3: False, 4: False}
        self.digital_outputs_target = self.digital_outputs
        self.current_position = None

        self.object_dictionary = {}  # TODO: Implementar diccionario con los mensajes que puede recibir y su decodificación

        self.status_freq: Parameter = self.get_parameter_or('status_freq', Parameter(name='status_freq', value=2)).value

        self.pub_heartbit = self.create_publisher(msg_type=StringStamped,
                                                  topic='/' + vehicle_parameters['id_vehicle'] + '/Heartbit',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_CAN = self.create_publisher(msg_type=CANGroup,
                                             topic='/' + vehicle_parameters['id_vehicle'] + '/CAN_' + str(
                                                 self.can_conected), qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=CAN,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/CAN',
                                 callback=self.msg_can, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=EPOSConsigna,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/' + self.get_name() + '/Consigna',
                                 callback=self.consigna, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=EPOSDigital,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/' + self.get_name() + '/Digital',
                                 callback=self.digital, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/' + self.get_name() + '/Enable',
                                 callback=self.enable, qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbit = self.create_timer(1, self.publish_heartbit)
        self.timer_status = self.create_timer(self.status_freq, self.read_status)
        self.timer_io = None

    def read_status(self):
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=epos.read_status(node=self.cobid)
        ))

    def read_io(self):
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=epos.read_io(node=self.cobid)
        ))
        if self.digital_outputs_target != self.digital_outputs:
            self.send_io()

    def enable(self, msg):
        if msg.data:
            if self.motor_status is None:
                pass
                # TODO: Mandar mensaje para resetear hasta modo habilitado/deshabilitado
            elif self.motor_status == 8:  # Fault_mode
                pass
                # TODO: Mandar mensaje para resetear hasta modo habilitado/deshabilitado
            elif self.motor_status == 35:  # Modo Switched on
                pass
                # TODO: Mandar mensaje 4
            elif self.motor_status == 39:  # Modo Operation enabled
                pass
                # No hacer nada
            else:
                pass
                # TODO: Mandar mensaje para resetear hasta modo habilitado/deshabilitado
        else:
            if self.motor_status is None:
                pass
                # TODO: Mandar mensaje para resetear hasta modo habilitado/deshabilitado
            elif self.motor_status == 8:  # Fault_mode
                pass
                # TODO: Mandar mensaje para resetear hasta modo habilitado/deshabilitado
            elif self.motor_status == 35:  # Modo Switched on
                pass
                # No hacer nada
            elif self.motor_status == 39:  # Modo Operation enabled
                pass
                # Mandar mensaje 6
            else:
                pass
                # TODO: Mandar mensaje para resetear hasta modo habilitado/deshabilitado

    def digital(self, msg):
        # TODO: Habilitar salida digital comparando con las salidas existentes
        if msg.io_digital in self.digital_outputs:
            if self.timer_io is None:
                self.timer_io = self.create_timer(self.status_freq, self.read_io)
            self.digital_outputs_target.update({msg.io_digital: msg.enable})
            if self.digital_outputs_target != self.digital_outputs:
                self.send_io()

    def send_io(self):
        msg_data = 0
        for output in self.digital_outputs_target:
            msg_data = (msg_data << 1) + self.digital_outputs_target.get(output)
        msg_data = msg_data << (16 - len(self.digital_outputs_target))

        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[self.make_can_msg(node=self.cobid, index=0x2078, data=msg_data)()]
        ))

    def consigna(self, msg):
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=epos.set_angle_value(node=self.cobid, angle=msg.position, absolute=msg.mode)
        ))

    def msg_can(self, msg):
        if msg.cobid == (0x580 + self.cobid):

        else:
            self.logger.debug(f'Msg CAN received with cobid: {msg.cobid}')

    def load_msgs(self):


    def publish_heartbit(self):
        msg = StringStamped(
            data=self.get_name()
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_heartbit.publish(msg)

    def shutdown(self):
        try:
            self.shutdown_flag = True
            self.timer_heartbit.cancel()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = MCD60_Node()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print('MCD60: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
