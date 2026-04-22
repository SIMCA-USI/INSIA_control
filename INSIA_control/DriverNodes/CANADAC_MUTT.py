from traceback import format_exc
import struct
import rclpy
from insia_msg.msg import StringStamped, FloatStamped, BoolStamped, Telemetry2
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from can_msgs.msg import Frame

class CANADACNode(Node):
    def __init__(self):
        super().__init__(node_name='CANADAC_MUTT', start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.logger = self.get_logger()
        self.logger.set_level(10)
        
        # --- Variables de estado ---
        self.enabled_cmd = False    # Lo que pide ROS (Enable)
        self.throttle_val = 0.0     # Rango [-100.0, 100.0]
        self.steering_val = 0.0     # Rango [-100.0, 100.0]
        self.shutdown_flag = False
        self.telemetry = Telemetry2(vehicle_ready=False)

        self._init_publishers()
        self._init_subscriptions()
        self._init_timers()
        self.logger.info("CANADAC_MUTT iniciado")

    def _init_publishers(self):
        self.pub_heartbeat = self.create_publisher(StringStamped, 'Heartbeat', qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_rpm = self.create_publisher(FloatStamped, 'RPM', qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_CAN = self.create_publisher(Frame, "CAN/can0/transmit", qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_telemetry = self.create_publisher(msg_type=Telemetry2, topic='Telemetry2', qos_profile=HistoryPolicy.KEEP_LAST)

    def _init_subscriptions(self):
        self.create_subscription(BoolStamped, self.get_name() + '/Enable', self.cb_enable, qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(FloatStamped, self.get_name() + '/Steering', self.cb_steering, qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(FloatStamped, self.get_name() + '/Throttle', self.cb_throttle, qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Frame, "CAN/can0/receive", self.msg_can_receive, qos_profile=HistoryPolicy.KEEP_LAST)

    def _init_timers(self):
        self.timer_heartbeat = self.create_timer(1.0, self.publish_heartbeat)
        # Timer de control a 10Hz: envía ENABLE y CONTROL constantemente
        self.timer_control = self.create_timer(0.05, self.send_can_updates)

        self.timer_control_enable = self.create_timer(0.05, self.send_can_enable)
        self.timer_telemetry = self.create_timer(0.5, self.send_telemetry)

    # --- Callbacks de actualización de estado (ROS -> Nodo) ---
    def cb_enable(self, data: BoolStamped):
        self.enabled_cmd = data.data

    def cb_steering(self, data: FloatStamped):
        self.steering_val = data.data

    def cb_throttle(self, data: FloatStamped):
        self.throttle_val =  data.data

    # --- Recepción de datos del ESP32 (CAN -> ROS) ---
    def msg_can_receive(self, msg: Frame):
        if msg.id == 0x200:
            try:
                # Decodificamos RPM (4 bytes float) e Interlock (1 byte bool)
                rpm = struct.unpack('<f', bytes(msg.data[0:4]))[0]
                interlock_ok = bool(msg.data[4])
                
                now = self.get_clock().now().to_msg()
                self.pub_rpm.publish(FloatStamped(header=Header(stamp=now), data=rpm))

                self.telemetry.vehicle_ready = interlock_ok

            except Exception as e:
                self.logger.error(f'Error en decodificación 0x200: {e}')

    # --- Lógica principal de envío (Nodo -> CAN) ---
    def send_can_updates(self):
        now = self.get_clock().now().to_msg()

        # 1. DETERMINAR VALORES SEGÚN INTERLOCK
        if not self.telemetry.vehicle_ready:
            t_byte = 127
            s_byte = 127
        else:
            t_byte = int(((self.throttle_val + 100.0) / 200.0) * 255.0)
            s_byte = int(((-self.steering_val + 100.0) / 200.0) * 255.0)

        # Asegurar límites de bytes por si acaso
        t_byte = max(0, min(255, t_byte))
        s_byte = max(0, min(255, s_byte))

        # 3. ENVIAR MENSAJE CONTROL (0x150)
        f_control = Frame()
        f_control.header.stamp = now
        f_control.id = 0x150
        f_control.dlc = 2
        f_control.data = [t_byte, s_byte, 0, 0, 0, 0, 0, 0]
        self.pub_CAN.publish(f_control)

    def send_can_enable(self):
        now = self.get_clock().now().to_msg()

        # 2. ENVIAR MENSAJE ENABLE (0x100)
        f_enable = Frame()
        f_enable.header.stamp = now
        f_enable.id = 0x100
        f_enable.dlc = 1
        f_enable.data = [self.enabled_cmd, 0, 0, 0, 0, 0, 0, 0]

        self.pub_CAN.publish(f_enable)

    def publish_heartbeat(self):
        msg = StringStamped(data=self.get_name())
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_heartbeat.publish(msg)
    
    def send_telemetry(self):
        self.pub_telemetry.publish(msg=self.telemetry)

    def shutdown(self):
        self.shutdown_flag = True
        self.timer_control.cancel()
        self.timer_heartbeat.cancel()

def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = CANADACNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'Interrupción por teclado en {manager.get_name()}')
    except Exception as e:
        print(f'Error: {e}\n{format_exc()}')
    finally:
        if manager:
            manager.shutdown()

if __name__ == '__main__':
    main()