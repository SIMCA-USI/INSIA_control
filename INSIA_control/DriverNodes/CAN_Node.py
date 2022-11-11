import os
import queue
import threading
import time

import rclpy
import yaml
from insia_msg.msg import CAN, CANGroup, StringStamped, CANEthStatus
from numpy import interp
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from yaml.loader import SafeLoader

from INSIA_control.utils.connection import Connection
from INSIA_control.utils.utils import decoder_can


class CanNode(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='CAN', namespace=vehicle_parameters['id_vehicle'], start_parameter_services=True,
                         allow_undeclared_parameters=False, automatically_declare_parameters_from_overrides=True)

        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False

        self.connection_mode = self.get_parameter('connection_mode').value
        self.ip = self.get_parameter('ip').value
        self.port = self.get_parameter('port').value
        self.extended = self.get_parameter('extend').value
        self.local = self.get_parameter('local').value
        self.queue = queue.Queue()
        self.min_frec_writer = self.min_frec_writer_default = 100
        self.max_frec_writer = self.max_frec_writer_default = 400
        if self.has_parameter('min_frec'):
            self.min_frec_writer = self.get_parameter('min_frec').value
        else:
            self.declare_parameter('min_frec', self.min_frec_writer_default)

        if self.has_parameter('max_frec'):
            self.max_frec_writer = self.get_parameter('max_frec').value
        else:
            self.declare_parameter('max_frec', self.max_frec_writer_default)

        if self.has_parameter('dinamic_frec'):
            self.flag_dinamic_frec = self.get_parameter('dinamic_frec').value
        else:
            self.declare_parameter('dinamic_frec', True)
        self.current_frec = 0

        self.pub_CAN = self.create_publisher(msg_type=CAN, topic='CAN', qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_status = self.create_publisher(msg_type=CANEthStatus, topic=self.get_name() + '/Status',
                                                qos_profile=HistoryPolicy.KEEP_LAST)

        if self.local:
            self.logger.info(f'Running in local mode')
            self.connection = None
        else:
            self.logger.info(f'Init CAN with {self.ip}:{self.port} extended={self.extended}')
            self.connection = Connection(name=f'Connection {self.ip}:{self.port}', mode=self.connection_mode,
                                         ip=self.ip, port=self.port, deco_function=self.decode_can,
                                         log_level=self._log_level.value)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)
        self.timer_write = threading.Thread(target=self.write_th, daemon=False, name=f'Writer {self.get_name()}')
        if not self.local:
            self.timer_write.start()

        self.create_subscription(msg_type=CANGroup, topic=self.get_name(), callback=self.save_msg,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

    def publish_heartbeat(self):
        msg = StringStamped(
            data=self.get_name()
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_heartbeat.publish(msg)
        msg_status = CANEthStatus(
            connected=self.connection.connected,
            queue_lenth=len(self.queue.queue),
            frequency=self.current_frec,
        )
        msg_status.header.stamp = self.get_clock().now().to_msg()
        self.pub_status.publish(msg_status)

    def save_msg(self, data: CANGroup):
        if not self.local:
            if self.is_connected():
                if len(self.queue.queue) > 100:
                    for _ in data.can_frames:
                        self.queue.get()
                [self.queue.put(frame.msg_raw) for frame in data.can_frames]
                self.logger.debug(f'Q len: {len(self.queue.queue)}')
        else:
            [self.logger.debug(
                f'Msg received: \n cobid: {hex(frame.cobid)}\n specifier: {hex(frame.specifier)}\n '
                f'index: {hex(frame.index)}\n sub_index: {hex(frame.sub_index)}\n'
                f' data: {" ".join(hex(x) for x in frame.data)}')
                for frame in data.can_frames]

    def is_connected(self):
        if self.local:
            return True
        else:
            return self.connection.connected

    def write(self):
        # Write
        if self.is_connected():
            msg = None
            try:
                msg = self.queue.get_nowait()
                assert len(msg) == 13
            except AssertionError:
                self.logger.error('Error in message length')
            except queue.Empty:
                pass
            except Exception as e:
                self.logger.error(f'Not managed exception: {e} {msg}')
            else:
                self.connection.send(msg)

    def write_th(self):
        from time import sleep
        while not self.shutdown_flag:
            # Update frequency parameter
            try:
                self.flag_dinamic_frec = self.get_parameter('dinamic_frec').value
                self.min_frec_writer = max(int(self.get_parameter('min_frec').value), self.min_frec_writer_default)
                self.max_frec_writer = min(int(self.get_parameter('max_frec').value), self.max_frec_writer_default)
            except Exception as e:
                self.logger.error(
                    f'Error in CAN parameters Flag:{self.flag_dinamic_frec} '
                    f'Min:{self.min_frec_writer} Max:{self.max_frec_writer}')
                self.logger.error(f'{e}')
                self.flag_dinamic_frec = True
                self.min_frec_writer = self.min_frec_writer_default
                self.max_frec_writer = self.max_frec_writer_default
            if not self.flag_dinamic_frec:
                timer_period = self.max_frec_writer
            else:
                timer_period = int(
                    interp(len(self.queue.queue), (10, 50), (self.min_frec_writer, self.max_frec_writer)))
            # Write
            self.write()
            self.current_frec = timer_period
            sleep(1 / timer_period)

        # Shutdown state
        sleep(0.5)  # Sleep for waiting last messages
        while len(self.queue.queue) > 0 and self.is_connected():
            self.logger.info(f'waiting to close q len:{len(self.queue.queue)}')
            self.write()
            time.sleep(1. / 100)
        self.connection.shutdown()

    def decode_can(self, can_frame):
        _, data_raw, cobid, specifier, index, sub_index = decoder_can(msg=can_frame, extended=self.extended)

        data = bytearray(can_frame[8:-1])
        msg = CAN(
            is_extended=self.extended,
            cobid=cobid,
            specifier=specifier,
            index=index,
            sub_index=sub_index,
            data=data,
            msg_raw=bytearray(can_frame)
        )
        self.logger.debug(f'Decoded msg {hex(cobid)} {hex(index)} {hex(sub_index)} {data_raw}')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_CAN.publish(msg)

    def shutdown(self):
        try:
            self.shutdown_flag = True
            for _ in range(3):
                rclpy.spin_once(self)
            self.timer_heartbeat.cancel()
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = CanNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
