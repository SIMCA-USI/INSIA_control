import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header

import os
import yaml
from yaml.loader import SafeLoader
import queue
import threading
import struct
import time

from src.utils.connection import Connection
from insia_msg.msg import CAN, CANGroup, StringStamped


class CAN_Node(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='CAN', namespace=vehicle_parameters['id_vehicle'], start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

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

        self.pub_CAN = self.create_publisher(msg_type=CAN, topic='/' + vehicle_parameters['id_vehicle'] + '/CAN',
                                             qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_heartbit = self.create_publisher(msg_type=StringStamped,
                                                  topic='/' + vehicle_parameters['id_vehicle'] + '/Heartbit',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)
        if self.local:
            self.logger.info(f'Running in local mode')
            self.connection = None
        else:
            self.logger.info(f'Init CAN with {self.ip}:{self.port} extended={self.extended}')
            self.connection = Connection(name=f'Connection {self.ip}:{self.port}', mode=self.connection_mode,
                                         ip=self.ip, port=self.port, deco_function=self.decode_can,
                                         log_level=self.log_level)

        self.timer_heartbit = self.create_timer(1, self.publish_heartbit)
        self.timer_write = threading.Thread(target=self.write_th, daemon=False, name=f'Writer {self.get_name()}')
        if not self.local:
            self.timer_write.start()

        self.create_subscription(msg_type=CANGroup,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/' + self.get_name(),
                                 callback=self.save_msg, qos_profile=HistoryPolicy.KEEP_LAST)

    def publish_heartbit(self):
        msg = StringStamped(
            data=self.get_name()
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_heartbit.publish(msg)

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
                f'index: {hex(frame.index)}\n sub_index: {hex(frame.sub_index)}\n data: {" ".join(hex(x) for x in frame.data)}')
                for frame in data.can_frames]

    def is_connected(self):
        return self.connection.connected

    def write(self):
        # Write
        if self.is_connected():
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
            # Write
            if self.is_connected():
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
            sleep(1. / self.write_timer_period)
        while len(self.queue.queue) > 0:
            self.logger.info(f'waiting to close q len:{len(self.queue.queue)}')
            self.write()
            time.sleep(0.1)

    def decode_can(self, can_frame):
        if self.extended:
            decoder = '<IBHB'
            cobid, specifier, index, sub_index = struct.unpack(decoder, can_frame[0:8])
        else:
            decoder = '<HBHB'
            cobid, specifier, index, sub_index = struct.unpack(decoder, can_frame[2:8])

        data = can_frame[8:-1]
        msg = CAN(
            is_extended=self.extended,
            cobid=cobid,
            specifier=specifier,
            index=index,
            sub_index=sub_index,
            data=data,
            msg_raw=can_frame
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_CAN.publish(msg)

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
        manager = CAN_Node()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print('CAN: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
