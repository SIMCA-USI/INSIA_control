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

        # brake_counts_min / brake_counts_max: offsets RELATIVOS a start_position
        # (posición del Maxon leída al arrancar). min = cadena tensada (CAN "40 00"),
        # max = límite (CAN "40 FA").
        self.declare_parameter('brake_counts_min', 102000)
        self.declare_parameter('brake_counts_max', 374000)

        self.declare_parameter('override_threshold_pedal', 20.0)
        self.declare_parameter('override_confirm_cycles', 3)
        self.declare_parameter('ignore_own_brake_echo_ms', 300)
        self.declare_parameter('position_poll_period_sec', 0.2)

        self.node_id = int(self.get_parameter('node_id').value)
        self.profile_velocity = int(self.get_parameter('profile_velocity').value)
        self.brake_counts_min = int(self.get_parameter('brake_counts_min').value)
        self.brake_counts_max = int(self.get_parameter('brake_counts_max').value)
        self.override_threshold_pedal = float(self.get_parameter('override_threshold_pedal').value)
        self.override_confirm_cycles = int(self.get_parameter('override_confirm_cycles').value)
        self.ignore_own_brake_echo_ms = int(self.get_parameter('ignore_own_brake_echo_ms').value)
        self.position_poll_period_sec = float(self.get_parameter('position_poll_period_sec').value)

        can_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(Frame, '/CAN/can1/transmit', 10)
        self.create_subscription(Frame, '/CAN/can1/receive', self.on_can_frame, can_qos)
        self.create_subscription(ControladorFloat, '/Brake', self.on_brake_cmd, 10)

        # Estado de la cola de comandos SDO (uno pendiente cada vez)
        self.pending_tag = None
        self.pending_deadline_ns = None
        self.queue = []
        self.busy = False

        # Posición del Maxon
        self.start_position = None
        self.current_position = None

        # Override / seguridad
        self.override_active = False
        self.override_counter = 0
        self.last_sent_pedal = None
        self.last_sent_time_ns = None

        self.create_timer(0.05, self.check_sdo_timeout)
        self._init_timer = self.create_timer(0.3, self.init_maxon)
        self.create_timer(self.position_poll_period_sec, self.poll_position)

        threading.Thread(target=self.keyboard_loop, daemon=True).start()

        self.get_logger().info('Nodo iniciado. Esperando inicialización del Maxon y comandos /Brake')

    # ---------- CAN / SDO helpers ----------

    def make_frame(self, cob_id, data, is_extended=False):
        data = (list(data) + [0] * 8)[:8]
        f = Frame()
        f.id = int(cob_id)
        f.is_extended = bool(is_extended)
        f.dlc = 8
        f.data = data
        return f

    def fmt(self, data):
        return ' '.join(f'{b & 0xFF:02X}' for b in data)

    def sdo_write_u8(self, index, sub, value):
        return [0x2F, index & 0xFF, index >> 8, sub, value & 0xFF, 0, 0, 0]

    def sdo_write_u16(self, index, sub, value):
        v = value & 0xFFFF
        return [0x2B, index & 0xFF, index >> 8, sub, v & 0xFF, v >> 8, 0, 0]

    def sdo_write_i32(self, index, sub, value):
        v = value & 0xFFFFFFFF
        return [0x23, index & 0xFF, index >> 8, sub, v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]

    def sdo_read(self, index, sub):
        return [0x40, index & 0xFF, index >> 8, sub, 0, 0, 0, 0]

    def send_sdo(self, data, tag, timeout_sec=1.0):
        self.pending_tag = tag
        self.pending_deadline_ns = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        self.pub.publish(self.make_frame(0x600 + self.node_id, data))

    def send_pedal_command(self, pedal_value: int, active: bool):
        """active=True -> "40 XX ..." (consigna activa). active=False -> "00 00 ..." (liberado)."""
        pedal_value = max(0, min(250, int(pedal_value))) if active else 0
        data = [0x40, pedal_value, 0xC0, 0xFF, 0xFF, 0x01, 0xFF, 0xF7] if active \
            else [0x00, 0x00, 0xC0, 0xFF, 0xFF, 0x01, 0xFF, 0xF7]

        self.pub.publish(self.make_frame(self.BRAKE_CAN_ID, data, is_extended=True))
        self.last_sent_pedal = pedal_value
        self.last_sent_time_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(f'Consigna CAN -> pedal={pedal_value} active={active} | DATA {self.fmt(data)}')

    # ---------- Cola simple de comandos SDO ----------

    def run_sequence(self, label, sdo_writes, log=True):
        """sdo_writes: lista de (tag, data). Se envían uno a uno; al terminar se lee la posición."""
        if self.busy:
            self.get_logger().warn('Ocupado, comando ignorado')
            return False
        self.queue = list(sdo_writes) + ['read_pos']
        self.busy = True
        if log:
            self.get_logger().info(label)
        self.advance_queue()
        return True

    def advance_queue(self):
        if self.pending_tag is not None:
            return
        if not self.queue:
            self.busy = False
            return
        step = self.queue.pop(0)
        if step == 'read_pos':
            self.send_sdo(self.sdo_read(0x6064, 0x00), 'read_pos')
        else:
            tag, data = step
            self.send_sdo(data, tag)

    def check_sdo_timeout(self):
        if self.pending_tag and self.get_clock().now().nanoseconds > self.pending_deadline_ns:
            self.get_logger().error(f'Timeout SDO [{self.pending_tag}]')
            self.pending_tag = None
            self.queue = []
            self.busy = False

    # ---------- Inicialización y movimiento ----------

    def init_maxon(self):
        if not self._init_timer.is_canceled():
            self._init_timer.cancel()
        self.run_sequence('Inicializando Maxon en Profile Position Mode', [
            ('fault_reset', self.sdo_write_u16(0x6040, 0x00, 0x0080)),
            ('set_ppm', self.sdo_write_u8(0x6060, 0x00, 0x01)),
            ('set_vel', self.sdo_write_i32(0x6081, 0x00, self.profile_velocity)),
            ('enable_1', self.sdo_write_u16(0x6040, 0x00, 0x0006)),
            ('enable_2', self.sdo_write_u16(0x6040, 0x00, 0x000F)),
        ])

    def disable_maxon(self):
        self.run_sequence('Override: apagando potencia del Maxon (freewheel)', [
            ('disable_voltage', self.sdo_write_u16(0x6040, 0x00, 0x0000)),
        ])

    def move_to_offset(self, offset_rel: int):
        """offset_rel: cuentas relativas a start_position. 0=reposo, min..max=rango de frenado."""
        if self.start_position is None:
            self.get_logger().warn('No se puede mover: aún no se ha leído la posición inicial')
            return
        offset_rel = max(0, min(self.brake_counts_max, int(offset_rel)))
        target_abs = self.start_position + offset_rel

        self.run_sequence(f'Moviendo Maxon a offset {offset_rel} (abs={target_abs})', [
            ('target_pos', self.sdo_write_i32(0x607A, 0x00, target_abs)),
            ('cw_enable', self.sdo_write_u16(0x6040, 0x00, 0x000F)),
            ('cw_start', self.sdo_write_u16(0x6040, 0x00, 0x003F)),
        ])

    def poll_position(self):
        """Lee la posición periódicamente sin generar ruido en el log (solo llena la cola si está libre)."""
        if not self.busy and self.pending_tag is None:
            self.queue = ['read_pos']
            self.busy = True
            self.advance_queue()

    # ---------- Conversión target_01 <-> cuentas / pedal ----------

    def target_to_offset(self, target_01: float) -> int:
        t = max(0.0, min(1.0, target_01))
        return int(round(self.brake_counts_min + t * (self.brake_counts_max - self.brake_counts_min)))

    def target_to_pedal(self, target_01: float) -> int:
        return int(round(max(0.0, min(1.0, target_01)) * 250))

    def offset_to_expected_pedal(self, offset_rel: int) -> int:
        if offset_rel <= self.brake_counts_min:
            return 0
        if offset_rel >= self.brake_counts_max:
            return 250
        span = self.brake_counts_max - self.brake_counts_min
        return int(round((offset_rel - self.brake_counts_min) * 250 / span))

    # ---------- Callbacks ----------

    def on_can_frame(self, msg: Frame):
        if msg.id == 0x580 + self.node_id and not msg.is_extended:
            self.handle_sdo_response(list(msg.data))
            return

        pedal_real = self.decode_pedal_frame(msg)
        if pedal_real is None:
            return

        if self.current_position is None or self.start_position is None:
            return
        if self.is_own_echo(pedal_real):
            return

        if not self.override_active:
            offset_rel = self.current_position - self.start_position
            self.check_override(offset_rel, pedal_real)

    def handle_sdo_response(self, data):
        if len(data) < 8:
            return
        cs, idx, sub = data[0], data[1] | (data[2] << 8), data[3]

        if cs == 0x80:
            abort = int.from_bytes(bytes(data[4:8]), 'little')
            self.get_logger().error(f'SDO Abort [{self.pending_tag}] 0x{idx:04X}:{sub:02X} -> 0x{abort:08X}')
            self.pending_tag = None
            self.queue = []
            self.busy = False
            return

        self.pending_tag = None

        if cs == 0x43 and idx == 0x6064 and sub == 0x00:
            new_pos = int.from_bytes(bytes(data[4:8]), 'little', signed=True)
            changed = new_pos != self.current_position
            self.current_position = new_pos
            if self.start_position is None:
                self.start_position = new_pos
            if changed:
                rel = new_pos - self.start_position
                self.get_logger().info(f'Posición Maxon -> abs={new_pos} | rel={rel}')

        self.advance_queue()

    def decode_pedal_frame(self, msg: Frame):
        if msg.id != self.BRAKE_CAN_ID or len(msg.data) < 2:
            return None
        b0, b1 = msg.data[0] & 0xFF, msg.data[1] & 0xFF
        if b0 == 0x00:
            return 0
        if b0 == 0x40:
            return b1
        return None

    def is_own_echo(self, pedal_real: int) -> bool:
        if self.last_sent_pedal is None:
            return False
        dt_ms = (self.get_clock().now().nanoseconds - self.last_sent_time_ns) / 1e6
        return dt_ms <= self.ignore_own_brake_echo_ms and pedal_real == self.last_sent_pedal

    def check_override(self, offset_rel: int, pedal_real: int):
        expected = self.offset_to_expected_pedal(offset_rel)
        error = abs(pedal_real - expected)

        if error >= self.override_threshold_pedal:
            self.override_counter += 1
            self.get_logger().warn(f'Desviación pedal: expected={expected} real={pedal_real} '
                                    f'counter={self.override_counter}/{self.override_confirm_cycles}')
        else:
            self.override_counter = 0

        if self.override_counter >= self.override_confirm_cycles:
            self.override_active = True
            self.get_logger().error(f'OVERRIDE detectado (real={pedal_real}, esperado={expected}). '
                                     f'Frenado remoto desactivado.')
            self.disable_maxon()

    def on_brake_cmd(self, msg: ControladorFloat):
        if self.override_active:
            self.get_logger().warn('Override activo: comando remoto ignorado')
            return

        if not msg.enable:
            self.get_logger().info('/Brake -> enable=False -> liberando freno (reposo)')
            self.send_pedal_command(0, active=False)
            self.move_to_offset(0)
            return

        target_01 = max(0.0, min(1.0, float(msg.target)))
        pedal = self.target_to_pedal(target_01)
        offset_rel = self.target_to_offset(target_01)

        self.get_logger().info(f'/Brake -> target={target_01:.3f} | pedal={pedal}/250 | offset_rel={offset_rel}')
        self.send_pedal_command(pedal, active=True)
        self.move_to_offset(offset_rel)

    # ---------- Teclado ----------

    def keyboard_loop(self):
        while True:
            try:
                ch = sys.stdin.read(1)
            except Exception:
                break
            if ch == 'r':
                self.override_active = False
                self.override_counter = 0
                self.get_logger().warn('Override reset manual')
                self.init_maxon()
            elif ch == 'q':
                rclpy.shutdown()
                break


def main(args=None):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
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
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        sys.stdout.write('\n')


if __name__ == '__main__':
    main()