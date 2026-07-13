import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from can_msgs.msg import Frame
from insia_msg.msg import ControladorFloat


class BrakeOverrideTest(Node):
    BRAKE_CAN_ID = 0x18F0010B

    def __init__(self):
        super().__init__('brake_node_override_test')

        self.declare_parameter('node_id', 3)
        self.declare_parameter('profile_velocity', 2000)

        # brake_counts_min / brake_counts_max son OFFSETS RELATIVOS a
        # start_position (la posición del Maxon al arrancar, pedal sin pisar).
        # brake_counts_min: offset donde la cadena empieza a tensar y el CAN
        #                    empieza a leer "40 00" (pedal_real = 0 pero ya tensado)
        # brake_counts_max: offset límite (pedal_real = 250)
        self.declare_parameter('brake_counts_min', 102000)
        self.declare_parameter('brake_counts_max', 374000)
        self.declare_parameter('brake_pedal_min', 0.0)
        self.declare_parameter('brake_pedal_max', 250.0)

        self.declare_parameter('override_threshold_pedal', 20.0)
        self.declare_parameter('override_confirm_cycles', 3)

        self.declare_parameter('ignore_own_brake_echo_ms', 300)

        # Periodo del sondeo automático de posición (segundos). Se lee la
        # posición del Maxon a este ritmo, pero solo se escribe en el log
        # cuando el valor leído difiere del anterior.
        self.declare_parameter('position_poll_period_sec', 0.2)

        self.node_id = int(self.get_parameter('node_id').value)
        self.profile_velocity = int(self.get_parameter('profile_velocity').value)

        self.brake_counts_min = int(self.get_parameter('brake_counts_min').value)
        self.brake_counts_max = int(self.get_parameter('brake_counts_max').value)
        self.brake_pedal_min = float(self.get_parameter('brake_pedal_min').value)
        self.brake_pedal_max = float(self.get_parameter('brake_pedal_max').value)

        self.override_threshold_pedal = float(self.get_parameter('override_threshold_pedal').value)
        self.override_confirm_cycles = int(self.get_parameter('override_confirm_cycles').value)

        self.ignore_own_brake_echo_ms = int(self.get_parameter('ignore_own_brake_echo_ms').value)

        self.position_poll_period_sec = float(self.get_parameter('position_poll_period_sec').value)

        can_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub = self.create_publisher(Frame, '/CAN/can1/transmit', 10)
        self.sub = self.create_subscription(Frame, '/CAN/can1/receive', self.cb, can_qos)
        self.brake_sub = self.create_subscription(ControladorFloat, '/Brake', self.brake_cmd_cb, 10)

        self.pending = None
        self.sequence = []
        self.busy = False
        self.deadline_ns = None

        self.start_position = None
        self.current_position = None
        self.last_target_counts = None

        self.remote_enabled = True
        self.override_active = False
        self.override_counter = 0
        self.last_real_pedal = None

        self.last_sent_brake_pedal = None
        self.last_sent_brake_time_ns = None

        self.timer = self.create_timer(0.05, self.tick)
        self._init_timer = self.create_timer(0.3, self.start_init_sequence)
        self._poll_timer = self.create_timer(self.position_poll_period_sec, self.auto_read_position)

        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()

        self.get_logger().info('Nodo iniciado. Esperando inicialización del Maxon y comandos /Brake')

    def _make_frame(self, cob_id, data, dlc=None, is_extended=False):
        data = list(data)
        dlc = dlc if dlc is not None else len(data)
        data = (data + [0] * 8)[:8]

        msg = Frame()
        msg.id = int(cob_id)
        msg.is_rtr = False
        msg.is_extended = bool(is_extended)
        msg.is_error = False
        msg.dlc = int(dlc)
        msg.data = data
        return msg

    def format_can_data(self, data, dlc=8):
        n = min(int(dlc), len(data))
        return ' '.join(f'{int(b) & 0xFF:02X}' for b in list(data)[:n])

    def sdo_write_u8(self, index, subindex, value):
        return [0x2F, index & 0xFF, (index >> 8) & 0xFF, subindex & 0xFF,
                value & 0xFF, 0x00, 0x00, 0x00]

    def sdo_write_i8(self, index, subindex, value):
        v = value & 0xFF
        return [0x2F, index & 0xFF, (index >> 8) & 0xFF, subindex & 0xFF,
                v, 0x00, 0x00, 0x00]

    def sdo_write_u16(self, index, subindex, value):
        v = value & 0xFFFF
        return [0x2B, index & 0xFF, (index >> 8) & 0xFF, subindex & 0xFF,
                v & 0xFF, (v >> 8) & 0xFF, 0x00, 0x00]

    def sdo_write_i32(self, index, subindex, value):
        v = value & 0xFFFFFFFF
        return [0x23, index & 0xFF, (index >> 8) & 0xFF, subindex & 0xFF,
                v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]

    def sdo_read(self, index, subindex):
        return [0x40, index & 0xFF, (index >> 8) & 0xFF, subindex & 0xFF,
                0x00, 0x00, 0x00, 0x00]

    def send_request(self, data, tag, timeout_sec=1.0):
        self.pending = tag
        self.deadline_ns = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        self.pub.publish(self._make_frame(0x600 + self.node_id, data))

    def send_brake_can_command(self, pedal_value: int, active: bool = True):
        """Envía la consigna CAN del pedal.

        active=True  -> frame "40 XX C0 FF FF 01 FF F7" (demanda activa,
                         XX=pedal_value). Se usa con enable=True.
        active=False -> frame "00 00 C0 FF FF 01 FF F7" (liberación total,
                         sin demanda). Se usa con enable=False, cuando el
                         actuador vuelve a la posición de reposo real.
        """
        pedal_value = max(0, min(250, int(pedal_value)))

        if active:
            data = [0x40, pedal_value & 0xFF, 0xC0, 0xFF, 0xFF, 0x01, 0xFF, 0xF7]
        else:
            data = [0x00, 0x00, 0xC0, 0xFF, 0xFF, 0x01, 0xFF, 0xF7]
            pedal_value = 0

        msg = self._make_frame(self.BRAKE_CAN_ID, data, dlc=8, is_extended=True)
        self.pub.publish(msg)

        self.last_sent_brake_pedal = pedal_value
        self.last_sent_brake_time_ns = self.get_clock().now().nanoseconds

        self.get_logger().info(
            f'Consigna CAN -> pedal={pedal_value} active={active} | '
            f'frame=ID 0x{self.BRAKE_CAN_ID:08X} DATA {self.format_can_data(data, 8)}'
        )

    def cb(self, msg: Frame):
        if msg.id == 0x580 + self.node_id and not msg.is_extended:
            data = list(msg.data)
            if len(data) < 8:
                return

            cs = data[0]
            idx = data[1] | (data[2] << 8)
            sub = data[3]

            if cs == 0x80:
                abort = int.from_bytes(bytes(data[4:8]), 'little', signed=False)
                self.get_logger().error(
                    f'SDO Abort [{self.pending}] 0x{idx:04X}:{sub:02X} -> 0x{abort:08X}'
                )
                self.pending = None
                self.sequence = []
                self.busy = False
                self.deadline_ns = None
                return

            self.pending = None
            self.deadline_ns = None

            if cs == 0x43 and idx == 0x6064 and sub == 0x00:
                new_position = int.from_bytes(bytes(data[4:8]), 'little', signed=True)
                position_changed = (self.current_position is None) or (new_position != self.current_position)
                self.current_position = new_position

                if self.start_position is None:
                    self.start_position = self.current_position

                if position_changed:
                    rel = self.current_position - self.start_position
                    self.get_logger().info(f'Posición Maxon -> abs={self.current_position} | rel={rel}')

            self.run_next_step()
            return

        pedal_real = self.decode_brake_pedal_frame(msg)
        if pedal_real is None:
            return

        self.last_real_pedal = pedal_real
        self.get_logger().info(
            f'RX pedal frame -> id=0x{msg.id:08X} ext={msg.is_extended} dlc={msg.dlc} '
            f'data={self.format_can_data(msg.data, msg.dlc)} | pedal_real={pedal_real}'
        )

        if self.current_position is None or self.start_position is None:
            self.get_logger().warn('Override no evaluado: posición aún desconocida')
            return

        if self.is_own_brake_echo(pedal_real):
            self.get_logger().info(f'Ignorando eco de consigna propia: pedal_real={pedal_real}')
            return

        if self.remote_enabled and not self.override_active:
            rel_position = self.current_position - self.start_position
            self.check_override(rel_position, pedal_real)

    def decode_brake_pedal_frame(self, msg: Frame):
        if msg.id != self.BRAKE_CAN_ID:
            return None

        data = list(msg.data)
        if len(data) < 2:
            return None

        b0 = int(data[0]) & 0xFF
        b1 = int(data[1]) & 0xFF

        if b0 == 0x00:
            return 0
        if b0 == 0x40:
            return b1
        return None

    def is_own_brake_echo(self, pedal_real: int) -> bool:
        if self.last_sent_brake_pedal is None or self.last_sent_brake_time_ns is None:
            return False

        now_ns = self.get_clock().now().nanoseconds
        dt_ms = (now_ns - self.last_sent_brake_time_ns) / 1e6

        if dt_ms <= self.ignore_own_brake_echo_ms and pedal_real == self.last_sent_brake_pedal:
            return True
        return False

    def tick(self):
        if self.pending is not None and self.deadline_ns is not None:
            if self.get_clock().now().nanoseconds > self.deadline_ns:
                self.get_logger().error(f'Timeout SDO [{self.pending}]')
                self.pending = None
                self.sequence = []
                self.busy = False
                self.deadline_ns = None

    def auto_read_position(self):
        """Sondeo periódico y silencioso de la posición del Maxon.

        No usa queue_sequence (evita spamear el log con "Leyendo
        posición..." cada ciclo) y se salta el ciclo si el nodo está
        ocupado con otra secuencia (init, movimiento, etc.) para no
        interferir. El propio cb() decide si hay que loguear, y solo lo
        hace cuando la posición leída cambia respecto a la anterior.
        """
        if self.busy or self.pending is not None:
            return

        self.sequence = [
            {'kind': 'read_pos'},
            {'kind': 'done'},
        ]
        self.busy = True
        self.run_next_step()

    def queue_sequence(self, label, steps):
        if self.busy:
            self.get_logger().warn('Ocupado')
            return False

        self.sequence = list(steps)
        self.busy = True
        self.get_logger().info(label)
        self.run_next_step()
        return True

    def run_next_step(self):
        if self.pending is not None:
            return

        if not self.sequence:
            self.busy = False
            return

        step = self.sequence.pop(0)
        kind = step['kind']

        if kind == 'write':
            self.send_request(step['data'], step['tag'])
        elif kind == 'read_pos':
            self.send_request(self.sdo_read(0x6064, 0x00), 'read_pos')
        elif kind == 'done':
            self.busy = False

    def start_init_sequence(self):
        if self._init_timer and not self._init_timer.is_canceled():
            self._init_timer.cancel()

        # start_position se fija con la primera lectura de posición tras
        # arrancar el nodo, sea cual sea (no se fuerza homing a abs=0, no
        # requiere confirmación manual). brake_counts_min/max están
        # calibrados como offsets RELATIVOS a ese rel=0 de arranque,
        # siempre que el Maxon no pierda alimentación entre sesiones.
        steps = [
            {'kind': 'write', 'tag': 'fault_reset',
             'data': self.sdo_write_u16(0x6040, 0x00, 0x0080)},
            {'kind': 'write', 'tag': 'set_ppm',
             'data': self.sdo_write_u8(0x6060, 0x00, 0x01)},
            {'kind': 'write', 'tag': 'set_vel',
             'data': self.sdo_write_i32(0x6081, 0x00, self.profile_velocity)},
            {'kind': 'write', 'tag': 'enable_1',
             'data': self.sdo_write_u16(0x6040, 0x00, 0x0006)},
            {'kind': 'write', 'tag': 'enable_2',
             'data': self.sdo_write_u16(0x6040, 0x00, 0x000F)},
            {'kind': 'read_pos'},
            {'kind': 'done'},
        ]
        self.queue_sequence('Inicializando Maxon en Profile Position Mode', steps)

    def disable_maxon_power(self):
        steps = [
            # 0x0000 en el Controlword (0x6040) ejecuta el comando "Disable Voltage"
            # Esto apaga la etapa de potencia instantáneamente y deja el motor libre.
            {'kind': 'write', 'tag': 'disable_voltage',
             'data': self.sdo_write_u16(0x6040, 0x00, 0x0000)},
            {'kind': 'done'},
        ]
        self.queue_sequence('Override activo: Apagando potencia del Maxon (Freewheel)', steps)

    def normalized_to_counts(self, value_01: float) -> int:
        """Devuelve un OFFSET RELATIVO a start_position, usado SOLO cuando
        enable=True.

        Mapea linealmente entre brake_counts_min (punto de tensado, donde
        el CAN empieza a leer "40 00", target_01=0.0) y brake_counts_max
        (límite, "40 FA", target_01=1.0).

        El reposo mecánico total (offset 0, cadena destensada) NO se
        alcanza por aquí: se comanda aparte cuando enable=False, ver
        brake_cmd_cb().
        """
        v = max(0.0, min(1.0, float(value_01)))
        span = self.brake_counts_max - self.brake_counts_min
        return int(round(self.brake_counts_min + v * span))

    def normalized_to_pedal(self, value_01: float) -> int:
        v = max(0.0, min(1.0, float(value_01)))
        return int(round(self.brake_pedal_min + v * (self.brake_pedal_max - self.brake_pedal_min)))

    def counts_to_expected_pedal(self, counts_rel: int) -> int:
        """counts_rel es un offset RELATIVO a start_position (no una
        posición absoluta del encoder)."""
        if counts_rel is None:
            return 0
        if counts_rel <= self.brake_counts_min:
            return 0
        if counts_rel >= self.brake_counts_max:
            return 250

        span_counts = self.brake_counts_max - self.brake_counts_min
        span_pedal = self.brake_pedal_max - self.brake_pedal_min
        pedal = (counts_rel - self.brake_counts_min) * span_pedal / span_counts
        return int(round(max(0.0, min(250.0, pedal))))

    def move_absolute_counts(self, target_counts_rel: int):
        """target_counts_rel es un offset RELATIVO a start_position.

        0                 -> posición inicial real (reposo mecánico total,
                              cadena destensada). Se invoca solo desde el
                              camino enable=False.
        brake_counts_min   -> punto de tensado (CAN "40 00"), target_01=0.0
                              con enable=True.
        brake_counts_max   -> límite (CAN "40 FA"), target_01=1.0

        Internamente se suma start_position para obtener la posición
        absoluta que espera el SDO 0x607A del Maxon.
        """
        if self.start_position is None:
            self.get_logger().warn('No se puede mover: start_position aún desconocida (esperando lectura inicial)')
            return

        target_counts_rel = max(0, min(self.brake_counts_max, int(target_counts_rel)))
        target_counts_abs = self.start_position + target_counts_rel

        steps = [
            {'kind': 'write', 'tag': 'target_pos',
             'data': self.sdo_write_i32(0x607A, 0x00, target_counts_abs)},
            {'kind': 'write', 'tag': 'cw_enable',
             'data': self.sdo_write_u16(0x6040, 0x00, 0x000F)},
            {'kind': 'write', 'tag': 'cw_start_abs',
             'data': self.sdo_write_u16(0x6040, 0x00, 0x003F)},
            {'kind': 'read_pos'},
            {'kind': 'done'},
        ]

        ok = self.queue_sequence(
            f'Moviendo Maxon a offset relativo {target_counts_rel} (abs={target_counts_abs})', steps
        )
        if ok:
            self.last_target_counts = target_counts_rel

    def brake_cmd_cb(self, msg: ControladorFloat):
        if self.override_active:
            self.get_logger().warn('Override activo: comando remoto ignorado')
            return

        if not msg.enable:
            # Reposo: liberar el freno y volver a la posición mecánica
            # inicial real (offset 0), independientemente de msg.target.
            # Frame CAN transmitido: "00 00 C0 FF FF 01 FF F7" (liberación,
            # sin demanda activa) en vez de "40 00" (demanda activa a 0).
            self.remote_enabled = True
            self.get_logger().info('/Brake -> enable=False -> liberando freno (reposo real)')
            self.send_brake_can_command(0, active=False)
            self.move_absolute_counts(0)
            return

        target_01 = max(0.0, min(1.0, float(msg.target)))
        pedal = self.normalized_to_pedal(target_01)
        target_counts_rel = self.normalized_to_counts(target_01)

        self.remote_enabled = True

        self.get_logger().info(
            f'/Brake -> target={target_01:.3f} | pedal={pedal}/250 | offset_rel={target_counts_rel}'
        )

        self.send_brake_can_command(pedal, active=True)
        self.move_absolute_counts(target_counts_rel)

    def check_override(self, current_counts_rel: int, pedal_real: int):
        """current_counts_rel es un offset RELATIVO a start_position."""
        pedal_expected = self.counts_to_expected_pedal(current_counts_rel)
        error = abs(int(pedal_real) - int(pedal_expected))

        self.get_logger().info(
            f'Override check -> rel={current_counts_rel} expected={pedal_expected} '
            f'real={pedal_real} error={error} counter={self.override_counter}'
        )

        if error >= self.override_threshold_pedal:
            self.override_counter += 1
            self.get_logger().warn(
                f'Desviación pedal detectada: expected={pedal_expected}, real={pedal_real}, '
                f'error={error}, counter={self.override_counter}/{self.override_confirm_cycles}'
            )
        else:
            if self.override_counter != 0:
                self.get_logger().info('Desviación dentro de umbral: reseteando contador override')
            self.override_counter = 0

        if self.override_counter >= self.override_confirm_cycles:
            self.override_active = True
            self.remote_enabled = False
            self.get_logger().error(
                f'OVERRIDE detectado: pedal real={pedal_real}, esperado={pedal_expected}. '
                f'Frenado remoto desactivado.'
            )
            self.disable_maxon_power()

    def keyboard_loop(self):
        while True:
            try:
                ch = sys.stdin.read(1)
            except Exception:
                break

            if ch == '5':
                self.queue_sequence('Leyendo posición Maxon (manual)', [
                    {'kind': 'read_pos'},
                    {'kind': 'done'},
                ])
            elif ch == 'r':
                self.override_active = False
                self.remote_enabled = True
                self.override_counter = 0
                self.get_logger().warn('Override reset manual')

                self.start_init_sequence()

            elif ch == 'q':
                rclpy.shutdown()
                break


def main(args=None):
    fd = sys.stdin.fileno()
    old_termios = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)
        rclpy.init(args=args)
        node = BrakeOverrideTest()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_termios)
        sys.stdout.write('\n')
        sys.stdout.flush()


if __name__ == '__main__':
    main()