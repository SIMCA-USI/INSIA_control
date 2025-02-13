import importlib
import os

import networkx as nx
import rclpy
import yaml
from insia_msg.msg import CAN, CANGroup, StringStamped, EPOSConsigna, EPOSDigital, BoolStamped, IntStamped, EPOSAnalog, \
    EPOSStatus
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader

from INSIA_control.utils.filtro import Decoder
from INSIA_control.utils.utils import make_can_msg


class MaxonNode(Node):
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
        self.op_mode = self.get_parameter('mode').value
        self.can_conected = self.get_parameter('can').value
        self.speed = self.get_parameter('speed').value
        driver_type = self.get_parameter('driver_type').value
        if driver_type == 'epos4':
            self.epos = importlib.import_module('INSIA_control.utils.epos4')
        elif driver_type == 'epos':
            self.epos = importlib.import_module('INSIA_control.utils.epos')
        else:
            raise ValueError(f'Driver {driver_type} is not implemented')
        self.EPOSStatus = getattr(self.epos, 'EPOSStatus')
        self.epos_dictionary = {}
        self.digital_outputs = {}
        for i in range(self.get_parameter('digital_outputs').value):
            self.digital_outputs.update({i + 1: False})
        self.digital_outputs_target = self.digital_outputs
        self.current_position = None
        self.target_state = self.EPOSStatus.Switched_on
        self.auto_fault_reset = self.get_parameter('auto_fault_reset').value
        self.decoder = Decoder(dictionary=self.get_parameter('dictionary').value, cobid=self.cobid)
        self.factor = self.get_parameter_or('factor', Parameter(name='factor', value=1)).value
        self.last_position_updated = 0
        self.motor_graph = nx.read_graphml(
            os.environ['ROS_WS'] + '/src/INSIA_control/INSIA_control/utils/maxon.graphml')
        self.informed_fault = False

        self.status_freq = self.get_parameter_or('status_freq', Parameter(name='status_freq', value=2)).value

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_CAN = self.create_publisher(msg_type=CANGroup, topic=str(self.can_conected),
                                             qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_status = self.create_publisher(msg_type=EPOSStatus, topic=self.get_name() + '/Status',
                                                qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=CAN, topic='CAN', callback=self.msg_can, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=EPOSConsigna, topic=self.get_name() + '/TargetPosition',
                                 callback=self.target_position_update, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=IntStamped, topic=self.get_name() + '/TargetTorque',
                                 callback=self.target_torque_update, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=EPOSDigital, topic=self.get_name() + '/Digital', callback=self.digital,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=EPOSAnalog, topic=self.get_name() + '/Analog', callback=self.analog,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/Enable', callback=self.enable,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=Header, topic=self.get_name() + '/FaultReset', callback=self.fault_reset,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=IntStamped, topic=self.get_name() + '/ResetPosition',
                                 callback=self.reset_position, qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)
        self.timer_read_dictionary = self.create_timer(0.1, self.read_dictionary)
        self.timer_status = self.create_timer(0.1, self.publish_status)
        # self.timer_print_dictionary = self.create_timer(1, self.print_dictionary)
        self.timer_io = None
        from time import sleep
        sleep(1)
        self.init_device()

    def init_device(self):
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=self.epos.init_device(node=self.cobid, mode=self.op_mode, rpm=self.speed)
        ))

    def publish_status(self):
        status = self.epos_dictionary.get('Statusword')
        if status is not None:
            status = self.epos.get_status(status)
        operation_mode = self.epos_dictionary.get('Modes_of_operation_display')
        if operation_mode is not None:
            operation_mode = self.epos.mode_epos_reverse.get(operation_mode)
        position = int(self.epos_dictionary.get(
            'position_actual_value')) if 'position_actual_value' in self.epos_dictionary.keys() else 0
        following_error = self.epos_dictionary.get('following_error_actual_value')
        self.pub_status.publish(
            EPOSStatus(
                header=Header(stamp=self.get_clock().now().to_msg()),
                status=status if status is not None else '',
                operation_mode=operation_mode if operation_mode is not None else '',
                position=position if position is not None else 0,
                following_error=following_error if following_error is not None else 0
            )
        )

    def fault_reset(self, data: Header):
        self.update_state(fault_reset=True)

    def reset_position(self, data):
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=self.epos.reset_position(node=self.cobid, position=data.data, prev_mode=self.op_mode,
                                                status_word=self.epos_dictionary.get('Statusword'))
        ))

    def print_dictionary(self):
        for key, value in self.epos_dictionary.items():
            print(f'{key}: {value}')

    def read_dictionary(self):
        keys = list(self.decoder.dic_parameters.keys())
        next_key = (self.last_position_updated + 1) % len(keys)
        self.last_position_updated = next_key
        key = keys[next_key].split(':')
        if 500 < int(key[0]):
            key = key + ([0] * (
                    3 - len(key)))  # array de longitud 3 relleno de cobid index subindex y los 0's necesarios
            self.pub_CAN.publish(CANGroup(
                header=Header(stamp=self.get_clock().now().to_msg()),
                can_frames=[make_can_msg(node=self.cobid, index=int(key[1]), sub_index=int(key[2]), write=False)]
            ))
            self.logger.debug(f'Read {hex(int(key[1]))}:{hex(int(key[2]))}')

    def enable(self, msg):
        if msg.data:
            self.target_state = self.EPOSStatus.Operation_enabled
        else:
            self.target_state = self.EPOSStatus.Switched_on
        self.update_state()

    def analog(self, msg: EPOSAnalog):
        if 1 <= msg.io_analog <= 2 and -4 <= msg.voltaje <= 4:
            self.pub_CAN.publish(CANGroup(
                header=Header(stamp=self.get_clock().now().to_msg()),
                can_frames=self.epos.set_analog(node=self.cobid, analog_out=msg.io_analog, voltage=msg.voltaje)
            ))
        else:
            self.logger.warn(f'Set voltage out of range out: {msg.io_analog} voltage: {msg.voltaje}')

    def digital(self, msg: EPOSDigital):
        if msg.io_digital in self.digital_outputs.keys():
            self.digital_outputs.update({msg.io_digital: msg.enable})
            self.pub_CAN.publish(CANGroup(
                header=Header(stamp=self.get_clock().now().to_msg()),
                can_frames=self.epos.set_digital(node=self.cobid, outputs=self.digital_outputs)
            ))
        else:
            self.logger.warn(f'Digital output out of range')
        # # TODO: Habilitar salida digital comparando con las salidas existentes
        # if msg.io_digital in self.digital_outputs:
        #     if self.timer_io is None:
        #         self.timer_io = self.create_timer(self.status_freq, self.read_io)
        #     self.digital_outputs_target.update({msg.io_digital: msg.enable})
        #     if self.digital_outputs_target != self.digital_outputs:
        #         self.send_io()

    def target_position_update(self, msg):
        if self.op_mode == 'PPM':
            status = self.epos.get_status_from_dict(self.epos_dictionary)
            if status == self.EPOSStatus.Operation_enabled:
                self.pub_CAN.publish(CANGroup(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    can_frames=self.epos.set_angle_value(node=self.cobid, angle=msg.position * self.factor,
                                                         absolute=msg.mode)
                ))
            else:
                self.logger.debug(f'Consigna {msg.position} {msg.mode} no enviada, motor en status: {status}')
        else:
            self.logger.debug(f'Received position but driver is in {self.op_mode} mode')

    def target_torque_update(self, msg: IntStamped):
        if self.op_mode == 'CST':
            status = self.epos.get_status_from_dict(self.epos_dictionary)
            if status == self.EPOSStatus.Operation_enabled:
                self.pub_CAN.publish(CANGroup(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    can_frames=self.epos.set_torque(node=self.cobid, torque=msg.data)
                ))
            else:
                self.logger.debug(f'Torque {msg.data} no enviada, motor en status: {status}')
        else:
            self.logger.debug(f'Received position but driver is in {self.op_mode} mode')

    def update_state(self, fault=False, fault_reset=False):
        status = self.epos.get_status_from_dict(self.epos_dictionary)
        fault_mode = (status == self.EPOSStatus.Fault or status == self.EPOSStatus.Fault_reaction_active)
        if self.auto_fault_reset or fault_reset or (not fault_mode and not fault):
            if fault_reset:
                self.logger.info('Reset fault')
            msg = self.epos.set_state(node=self.cobid, target_state=self.target_state, graph=self.motor_graph,
                                      status_word=self.epos_dictionary.get('Statusword') if not fault else None)
            if msg is not None:
                self.pub_CAN.publish(CANGroup(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    can_frames=msg
                ))
            self.informed_fault = False

    def msg_can(self, msg: CAN):
        try:
            name, value = self.decoder.decode(msg)
            self.epos_dictionary.update({name: value})
            if name == 'Fault':
                self.logger.warn(f'Fault: {self.epos.get_fault(value)}')
                self.update_state(fault=True)
            if name == 'Statusword':
                self.update_state()
            self.logger.debug(f'Decoded {name}: {value}')
        except ValueError as e:
            pass
            self.logger.debug(f'{e}')
        except Exception as e:
            self.logger.debug(f'{e}')

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

    def shutdown(self):
        try:
            self.shutdown_flag = True
            for _ in range(2):
                rclpy.spin_once(self)
            # msg = self.epos.set_state(node=self.cobid, target_state=self.EPOSStatus.Switched_on, graph=self.motor_graph,
            #                           status_word=self.epos_dictionary.get('Statusword'))
            # if msg is not None:
            #     self.pub_CAN.publish(CANGroup(
            #         header=Header(stamp=self.get_clock().now().to_msg()),
            #         can_frames=msg
            #     ))
            for key in self.digital_outputs.keys():
                self.digital_outputs.update({key: False})
            self.pub_CAN.publish(CANGroup(
                header=Header(stamp=self.get_clock().now().to_msg()),
                can_frames=self.epos.set_digital(node=self.cobid, outputs=self.digital_outputs)
            ))
            self.timer_heartbeat.cancel()
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = MaxonNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(f'{e}')
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
