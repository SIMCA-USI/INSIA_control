import threading
import socket
import time
from traceback import format_exc
import datetime
from rclpy.logging import get_logger


class Connection:
    def __init__(self, name='unknown connection', mode: str = 'tcp', ip='', port=0, send_t=1, recv_t=1,
                 read_len=13, deco_function=None, alarms=None, log_level=10):
        self.status = 'init'
        self.name = name
        self.ip = ip
        self.port = port
        self.connecting = False
        self.connected = False
        self.connected_recv = False
        self.connected_send = False
        self.thread_reconnection = None
        self.send_t = send_t  # Timeout send
        self.recv_t = recv_t  # Timeout recv
        self.read_len = read_len
        self.deco_function = deco_function
        self.logger = get_logger(self.name)
        self.logger.set_level(log_level)
        self.alarms = alarms if alarms is not None else self.default_alarms
        self.shutdown_flag = False
        self.t_frec = time.time()
        self.t_msgs_send = 0
        self.t_msgs_recv = 0
        self.thread_frec = threading.Thread(target=self.frec, daemon=True, name=f'Frec {self.name}')
        self.thread_frec.start()
        if mode == 'tcp':
            self.socket = None
            self.connect = self.connect_tcp
            self.reconnect = self.reconnect_tcp
            self.send = self.send_tcp
            self.recv = self.recv_tcp
            self.shutdown = self.shutdown_tcp
        elif mode == 'udp':
            self.socket_send = None
            self.socket_recv = None
            self.connect = self.connect_udp
            self.reconnect = self.reconnect_udp
            self.send = self.send_udp
            self.recv = self.recv_udp
            self.shutdown = self.shutdown_udp
            self.create_sender_udp()

        self.thread_read = threading.Thread(target=self.recv, daemon=True, name=f'Read {self.name}')
        self.thread_read.start()
        self.logger.info(f'Created connexion {self.name} in mode {mode} with ip {self.ip}:{self.port}')

    def frec(self):
        while not self.shutdown_flag:
            time.sleep(2)
            t = time.time() - self.t_frec
            frec_recv = self.t_msgs_recv / t
            frec_send = self.t_msgs_send / t
            self.logger.debug(
                f'{datetime.datetime.now().strftime("%H:%M:%S")}:  Frec send: {int(frec_send)} Hz'
                f' Frec recv: {int(frec_recv)} Hz')
            self.t_msgs_send, self.t_msgs_recv, self.t_frec = 0, 0, time.time()

    def default_alarms(self, id_alarma, estado, t_alarma, notas=''):
        print(f'{id_alarma} {estado} {t_alarma}')

    def connect_tcp(self):
        try:
            self.logger.info(f'Connecting tcp {self.ip}:{self.port}')
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5)
            self.socket.connect((self.ip, self.port))
            self.logger.info(f'Socket connected')
            self.connected = True
            self.connecting = False
            self.socket.settimeout(2)
        except socket.timeout:
            pass
        except OSError as e:
            self.logger.error(f'OSERROR conecting {self.name}: {e}')
            self.status = 'error'
        except Exception as e:
            self.logger.error(f'Exception connecting {self.name}: {e}')
            self.connecting = False
            self.connected = False

    def reconnect_tcp(self):
        if not self.connected and not self.shutdown_flag:
            if not self.connecting:
                self.logger.debug(
                    f'Reconnecting: Connected:{self.connected} '
                    f'Connecting:{self.connecting} Socket: {not socket is None}')
                self.connecting = True
                if self.socket is not None:
                    try:
                        self.socket.close()
                    except (socket.error, AttributeError):
                        pass
                    self.socket = None
                if self.thread_reconnection is None or not self.thread_reconnection.is_alive():
                    try:
                        self.thread_reconnection = threading.Thread(target=self.connect, daemon=True,
                                                                    name=f'Connecting {self.name}')
                        self.thread_reconnection.start()
                    except Exception as e:
                        self.connecting = False
                        self.logger.error(f'Exception in reconnection: {e}')
        else:
            if self.connected:
                self.logger.info('Socket is already connected')

    def send_tcp(self, msg):
        # self.logger.info(f'Sent msg: {msg}')
        if not self.shutdown_flag:
            try:
                self.socket.sendall(msg)
                self.t_msgs_send += 1
            except BrokenPipeError as e:
                try:
                    self.connected = False
                    self.socket.close()
                finally:
                    self.reconnect()
            except OSError as e:
                self.logger.error(f'Exception in send OSERROR {e}')
                self.status = 'error'
                try:
                    self.connected = False
                    self.socket.close()
                finally:
                    self.reconnect()
            except Exception as e:
                self.logger.error(f'Exception in send {e}')
                self.logger.error(f'Format {format_exc()}')

    def recv_tcp(self):
        while not self.shutdown_flag:
            try:
                if self.connected:
                    data = self.socket.recv(self.read_len)
                else:
                    if not self.connecting:
                        self.logger.warn('Request reconnection')
                        self.reconnect()
                    continue
            except socket.timeout:
                pass
            except OSError as e:
                self.logger.error(f' OSERROR in recv: {e}')
            except Exception as e:
                self.logger.error(f' Exception in recv: {e}')
            else:
                self.deco_function(data)
                self.t_msgs_recv += 1

    def shutdown_tcp(self):
        try:
            self.logger.info('Shutdown')
            self.shutdown_flag = True
            self.connected = False
            if self.socket is not None:
                self.socket.close()
            self.status = 'shutdown'
        except Exception:
            self.logger.error(f'Exception in close')

    def connect_udp(self):
        pass

    def reconnect_udp(self):
        pass

    def recv_udp(self) -> None:
        """
        Thread to receive from the CAN cards
        """

        recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv.settimeout(self.recv_t)
        recv.bind(('', self.port))
        self.connected_recv = True
        if self.connected_send:
            self.connected = True
        try:
            while not self.shutdown_flag:
                try:
                    data_raw, source = recv.recvfrom(13)
                    self.deco_function(data_raw)
                    self.t_msgs_recv += 1

                    # Revisar codigos de error
                    # if frame.protocol == CANOpenProtocols.EMCY and frame.data[0] != 0:
                    #     error = frame.load_error()()
                    #     self.logger.error(
                    #         f"CAN device {frame.can_id} is in error state | {error} | {error.__class__.__name__}")
                    #     self.logger.warning(f"Resetting CAN device {frame.can_id}")
                    #     self.add_to_queue(epos_motor.fault_reset(frame.can_id))
                    #     self.add_to_queue(epos_motor.init_device(frame.can_id))

                except socket.timeout:
                    pass
                    # self.logger.warning(f"No traffic for {self.time_out_s.value}s")
                except Exception:
                    format_exc()
        finally:
            try:
                recv.close()
            except (socket.error, AttributeError):
                pass

    def create_sender_udp(self):
        try:
            self.socket_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket_send.bind(('', 10000))
            self.connected_send = True
            if self.connected_recv:
                self.connected = True
        except Exception as e:
            format_exc()

    def send_udp(self, msg):
        try:
            self.socket_send.sendto(msg, (self.ip, self.port))
            self.t_msgs_send += 1
        except socket.timeout:
            pass
        except socket.error:
            format_exc()

    def shutdown_udp(self):
        self.shutdown_flag = True
        try:
            self.socket_send.close()
        except (socket.error, AttributeError):
            pass
        finally:
            try:
                self.socket_recv.close()
            except (socket.error, AttributeError):
                pass
