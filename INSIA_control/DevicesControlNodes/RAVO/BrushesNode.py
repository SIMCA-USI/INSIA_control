from traceback import format_exc

import rclpy
from insia_msg.msg import StringStamped, Telemetry, IntStamped, BoolStamped, RAVO
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header


class BrushesNode(Node):

    def __init__(self):
        super().__init__(node_name='BrushesNode',
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.telemetry = Telemetry()

        self.create_subscription(msg_type=RAVO, topic=self.get_name(), callback=self.brushes,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_seta_emergencia = self.create_publisher(
            msg_type=BoolStamped,
            topic='BrushesDriver/SetaEmergencia',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_botonera = self.create_publisher(
            msg_type=BoolStamped,
            topic='BrushesDriver/Botonera',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_rutina_encendido = self.create_publisher(
            msg_type=BoolStamped,
            topic='BrushesDriver/RutinaEncendido',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_modo_trabajo = self.create_publisher(
            msg_type=BoolStamped,
            topic='BrushesDriver/ModoTrabajo',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_turbina_activacion = self.create_publisher(
            msg_type=BoolStamped,
            topic='BrushesDriver/Turbina/Activacion',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_turbina_rpm = self.create_publisher(
            msg_type=IntStamped,
            topic='BrushesDriver/Turbina/RPM',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_cepillos_centrales_activacion = self.create_publisher(
            msg_type=BoolStamped,
            topic='BrushesDriver/CepillosCentrales/Activacion',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_cepillos_centrales_movimiento = self.create_publisher(
            msg_type=IntStamped,
            topic='BrushesDriver/CepillosCentrales/Movimiento',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_cepillos_centrales_agua = self.create_publisher(
            msg_type=BoolStamped,
            topic='BrushesDriver/CepillosCentrales/Agua',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_cepillos_centrales_luces = self.create_publisher(
            msg_type=BoolStamped,
            topic='BrushesDriver/CepillosCentrales/Luces',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_cepillos_frontal_activacion = self.create_publisher(
            msg_type=BoolStamped,
            topic='BrushesDriver/CepillosFrontal/Activacion',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_cepillos_frontal_movimiento = self.create_publisher(
            msg_type=IntStamped,
            topic='BrushesDriver/CepillosFrontal/Movimiento',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_cepillos_frontal_agua = self.create_publisher(
            msg_type=BoolStamped,
            topic='BrushesDriver/CepillosFrontal/Agua',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_cepillos_frontal_luces = self.create_publisher(
            msg_type=BoolStamped,
            topic='BrushesDriver/CepillosFrontal/Luces',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_cepillos_frontal_despliegue = self.create_publisher(
            msg_type=BoolStamped,
            topic='BrushesDriver/CepillosFrontal/Despliegue',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_cepillos_frontal_rotacion = self.create_publisher(
            msg_type=IntStamped,
            topic='BrushesDriver/CepillosFrontal/Rotacion',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_cepillos_frontal_angulo = self.create_publisher(
            msg_type=IntStamped,
            topic='BrushesDriver/CepillosFrontal/Angulo',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_suspension = self.create_publisher(
            msg_type=BoolStamped,
            topic='BrushesDriver/Suspension',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_pistolas = self.create_publisher(
            msg_type=BoolStamped,
            topic='BrushesDriver/Pistolas',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_faldon = self.create_publisher(
            msg_type=IntStamped,
            topic='BrushesDriver/Faldon',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_chupon = self.create_publisher(
            msg_type=IntStamped,
            topic='BrushesDriver/Chupon',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.pub_puertas = self.create_publisher(
            msg_type=BoolStamped,
            topic='BrushesDriver/Puertas',
            qos_profile=HistoryPolicy.KEEP_LAST
        )

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def create_bool_stamped_with_header(self, data):
        msg = BoolStamped()
        msg.data = data
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def create_int_stamped_with_header(self, data):
        msg = IntStamped()
        msg.data = data
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def brushes(self, msg: RAVO):
        """
        Callback que gestiona el control de los actuadores de limpieza en función del mensaje RAVO recibido.
        Aplica la lógica de seguridad: si el sistema global o algún grupo está deshabilitado (enable=False),
        se publican valores seguros (False/0) en los tópicos correspondientes.

        Args:
            msg (RAVO): Mensaje recibido con el estado y comandos de los diferentes actuadores.
        """
        try:
            if not msg.enable:
                self.logger.warn("Sistema global deshabilitado: desactivando todos los actuadores.")
                self.pub_seta_emergencia.publish(self.create_bool_stamped_with_header(False))
                self.pub_botonera.publish(self.create_bool_stamped_with_header(False))
                self.pub_rutina_encendido.publish(self.create_bool_stamped_with_header(False))
                self.pub_modo_trabajo.publish(self.create_bool_stamped_with_header(False))
                self.pub_turbina_activacion.publish(self.create_bool_stamped_with_header(False))
                self.pub_turbina_rpm.publish(self.create_int_stamped_with_header(0))
                # Cepillos centrales
                self.pub_cepillos_centrales_activacion.publish(self.create_bool_stamped_with_header(False))
                self.pub_cepillos_centrales_movimiento.publish(self.create_int_stamped_with_header(0))
                self.pub_cepillos_centrales_agua.publish(self.create_bool_stamped_with_header(False))
                self.pub_cepillos_centrales_luces.publish(self.create_bool_stamped_with_header(False))
                # Cepillo frontal
                self.pub_cepillos_frontal_activacion.publish(self.create_bool_stamped_with_header(False))
                self.pub_cepillos_frontal_movimiento.publish(self.create_int_stamped_with_header(0))
                self.pub_cepillos_frontal_agua.publish(self.create_bool_stamped_with_header(False))
                self.pub_cepillos_frontal_luces.publish(self.create_bool_stamped_with_header(False))
                self.pub_cepillos_frontal_despliegue.publish(self.create_bool_stamped_with_header(False))
                self.pub_cepillos_frontal_rotacion.publish(self.create_int_stamped_with_header(0))
                self.pub_cepillos_frontal_angulo.publish(self.create_int_stamped_with_header(0))
                self.pub_suspension.publish(self.create_bool_stamped_with_header(False))
                self.pub_pistolas.publish(self.create_bool_stamped_with_header(False))
                self.pub_faldon.publish(self.create_int_stamped_with_header(0))
                self.pub_chupon.publish(self.create_int_stamped_with_header(0))
                self.pub_puertas.publish(self.create_bool_stamped_with_header(False))
                return

            self.logger.info("Sistema global habilitado: publicando comandos recibidos.")

            # Publicaciones generales con header
            self.pub_seta_emergencia.publish(self.create_bool_stamped_with_header(msg.seta_emergencia))
            self.pub_botonera.publish(self.create_bool_stamped_with_header(msg.botonera))
            self.pub_rutina_encendido.publish(self.create_bool_stamped_with_header(msg.rutina_encendido))
            self.pub_modo_trabajo.publish(self.create_bool_stamped_with_header(msg.modo_trabajo))
            self.pub_turbina_activacion.publish(self.create_bool_stamped_with_header(msg.turbina.enable))
            self.pub_turbina_rpm.publish(self.create_int_stamped_with_header(getattr(msg.turbina, 'rpm', 0)))

            # Cepillos centrales
            cc = msg.cepillos_centrales
            if not cc.enable:
                self.logger.warn("Cepillos centrales deshabilitados: desactivando todos sus actuadores.")
                self.pub_cepillos_centrales_activacion.publish(self.create_bool_stamped_with_header(False))
                self.pub_cepillos_centrales_movimiento.publish(self.create_int_stamped_with_header(0))
                self.pub_cepillos_centrales_agua.publish(self.create_bool_stamped_with_header(False))
                self.pub_cepillos_centrales_luces.publish(self.create_bool_stamped_with_header(False))
            else:
                self.pub_cepillos_centrales_activacion.publish(self.create_bool_stamped_with_header(cc.enable))
                self.pub_cepillos_centrales_movimiento.publish(self.create_int_stamped_with_header(cc.movement))
                self.pub_cepillos_centrales_agua.publish(self.create_bool_stamped_with_header(cc.water))
                self.pub_cepillos_centrales_luces.publish(self.create_bool_stamped_with_header(cc.light))

            # Cepillo frontal
            cf = msg.cepillo_frontal
            if not cf.enable:
                self.logger.warn("Cepillo frontal deshabilitado: desactivando todos sus actuadores.")
                self.pub_cepillos_frontal_activacion.publish(self.create_bool_stamped_with_header(False))
                self.pub_cepillos_frontal_movimiento.publish(self.create_int_stamped_with_header(0))
                self.pub_cepillos_frontal_agua.publish(self.create_bool_stamped_with_header(False))
                self.pub_cepillos_frontal_luces.publish(self.create_bool_stamped_with_header(False))
                self.pub_cepillos_frontal_despliegue.publish(self.create_bool_stamped_with_header(False))
                self.pub_cepillos_frontal_rotacion.publish(self.create_int_stamped_with_header(0))
                self.pub_cepillos_frontal_angulo.publish(self.create_int_stamped_with_header(0))
            else:
                self.pub_cepillos_frontal_activacion.publish(self.create_bool_stamped_with_header(cf.enable))
                self.pub_cepillos_frontal_movimiento.publish(self.create_int_stamped_with_header(cf.movement))
                self.pub_cepillos_frontal_agua.publish(self.create_bool_stamped_with_header(cf.water))
                self.pub_cepillos_frontal_luces.publish(self.create_bool_stamped_with_header(cf.light))
                self.pub_cepillos_frontal_despliegue.publish(self.create_bool_stamped_with_header(cf.extend))
                self.pub_cepillos_frontal_rotacion.publish(self.create_int_stamped_with_header(cf.rotation))
                self.pub_cepillos_frontal_angulo.publish(self.create_int_stamped_with_header(cf.angle))

            self.pub_suspension.publish(self.create_bool_stamped_with_header(msg.suspension))
            self.pub_pistolas.publish(self.create_bool_stamped_with_header(msg.pistolas))
            self.pub_faldon.publish(self.create_int_stamped_with_header(getattr(msg.faldon, 'value', 0)))
            self.pub_chupon.publish(self.create_int_stamped_with_header(getattr(msg.chupon, 'value', 0)))
            self.pub_puertas.publish(self.create_bool_stamped_with_header(msg.door_lock))

            self.logger.debug("Comandos publicados correctamente para todos los actuadores.")

        except Exception as e:
            self.logger.error(f"Error en brushes: {e}")

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
        pass


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = BrushesNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        format_exc()
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
