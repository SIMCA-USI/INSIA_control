"""
Simulador de actuador para probar el ACK del nodo CANADAC.

Escucha los CANGroup del topic 'can' (por defecto can_control) y devuelve cada frame en 'CAN'
con specifier 0x40.

Parámetros (modificables en caliente con ros2 param set):
  cobid      cobid al que responde (0x605)
  delay      retardo de la respuesta en s (0.0)
  drop_prob  probabilidad de no responder, 0..1 (0.0)
  mute       True = no responde nunca (False)

Uso (lanzar en el mismo namespace que el nodo CANADAC):
  ros2 run <paquete> fake_adac_ack --ros-args -p cobid:=0x605
  ros2 topic echo /CANADAC_Acelerador/Alarm              # ver alarmas
  ros2 param set /FakeADAC mute true                     # actuador muerto -> ALARM
  ros2 param set /FakeADAC mute false                    # vuelve a responder -> RECOVERED
  ros2 param set /FakeADAC drop_prob 0.2                 # pierde 1 de cada 5 respuestas
  ros2 param set /FakeADAC delay 0.15                    # respuestas lentas -> reintentos

Solo para pruebas: no usar con el actuador real conectado al mismo bus/topic.
"""
import random
from collections import deque

import rclpy
from insia_msg.msg import CAN, CANGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class FakeADAC(Node):
    """Nodo que imita la respuesta del actuador ADAC: eco de cada petición con specifier 0x40."""

    def __init__(self):
        super().__init__('FakeADAC')
        self.declare_parameter('can', 'can_control')
        self.declare_parameter('cobid', 0x605)
        self.declare_parameter('delay', 0.0)
        self.declare_parameter('drop_prob', 0.0)
        self.declare_parameter('mute', False)

        self.pub = self.create_publisher(CAN, 'CAN', 50)
        self.create_subscription(CANGroup, self.get_parameter('can').value, self.on_group, 50)
        self.queue = deque()  # (instante_de_salida_ns, frame)
        # Los retardos se gestionan con un timer, sin sleep
        self.create_timer(0.001, self.flush)

    def on_group(self, group: CANGroup):
        """Recibe los comandos del nodo CANADAC y encola el eco de los que van a nuestro cobid."""
        # Lectura de parámetros en cada mensaje para que 'ros2 param set' tenga efecto inmediato
        p = lambda n: self.get_parameter(n).value
        for frame in group.can_frames:
            if frame.cobid != p('cobid') or frame.specifier != 0x22:
                continue
            if p('mute') or random.random() < p('drop_prob'):
                self.get_logger().info(f'Descartado index=0x{frame.index:04X}')
                continue
            echo = CAN(header=frame.header, is_extended=frame.is_extended, cobid=frame.cobid,
                       specifier=0x40, index=frame.index, sub_index=frame.sub_index,
                       data=frame.data, msg_raw=frame.msg_raw)
            # En la trama cruda el specifier ocupa el byte 4 (2 de relleno + 2 de cobid delante)
            raw = list(echo.msg_raw)
            raw[4] = 0x40
            echo.msg_raw = raw
            due = self.get_clock().now().nanoseconds + int(p('delay') * 1e9)
            self.queue.append((due, echo))

    def flush(self):
        """Timer (1 ms): publica los ecos cuyo retardo ya ha vencido."""
        now = self.get_clock().now().nanoseconds
        while self.queue and self.queue[0][0] <= now:
            _, echo = self.queue.popleft()
            echo.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(echo)


def main(args=None):
    rclpy.init(args=args)
    node = FakeADAC()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
