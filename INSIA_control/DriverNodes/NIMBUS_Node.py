from traceback import format_exc

import rclpy
from INSIA_control.utils.utils import make_can_msg
from insia_msg.msg import CANGroup, StringStamped, BoolStamped, IntStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header


class NIMBUSNode(Node):
    def __init__(self):
        super().__init__(node_name='BrushesControl', start_parameter_services=True,
                         allow_undeclared_parameters=False, automatically_declare_parameters_from_overrides=True)

        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False

        self.cobid = self.get_parameter('cobid').value
        self.can_connected = self.get_parameter('can').value

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_CAN = self.create_publisher(msg_type=CANGroup, topic=self.can_connected,
                                             qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/SetaEmergencia',
                                 callback=self.seta_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/Botonera',
                                 callback=self.botonera_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/RutinaEncendido',
                                 callback=self.RutinaEncendido_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/ModoTrabajo',
                                 callback=self.ModoTrabajo_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/Turbina/Activacion',
                                 callback=self.turbina_activacion_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=IntStamped, topic=self.get_name() + '/Turbina/RPM',
                                 callback=self.turbina_rpm_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/CepillosCentrales/Activacion',
                                 callback=self.cepillos_centrales_activacion_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=IntStamped, topic=self.get_name() + '/CepillosCentrales/Movimiento',
                                 callback=self.cepillos_centrales_movimiento_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/CepillosCentrales/Agua',
                                 callback=self.cepillos_centrales_agua_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/CepillosCentrales/Luces',
                                 callback=self.cepillos_centrales_luces_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/CepillosFrontal/Activacion',
                                 callback=self.cepillos_frontal_activacion_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=IntStamped, topic=self.get_name() + '/CepillosFrontal/Movimiento',
                                 callback=self.cepillos_frontal_movimiento_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/CepillosFrontal/Agua',
                                 callback=self.cepillos_frontal_agua_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/CepillosFrontal/Luces',
                                 callback=self.cepillos_frontal_luces_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/CepillosFrontal/Despliegue',
                                 callback=self.cepillos_frontal_despliegue_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=IntStamped, topic=self.get_name() + '/CepillosFrontal/Rotacion',
                                 callback=self.cepillos_frontal_rotacion_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=IntStamped, topic=self.get_name() + '/CepillosFrontal/Angulo',
                                 callback=self.cepillos_frontal_angulo_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/Suspension',
                                 callback=self.suspension_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/Pistolas',
                                 callback=self.pistolas_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=IntStamped, topic=self.get_name() + '/Faldon', callback=self.faldon_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=IntStamped, topic=self.get_name() + '/Chupon', callback=self.chupon_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic=self.get_name() + '/Puertas',
                                 callback=self.puertas_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def seta_callback(self, data):
        if data.data:
            # Activa la seta de emergencia
            msg = make_can_msg(node=self.cobid, index=0x0000, data=0x01, clock=self.get_clock().now().to_msg())
        else:
            # Desactiva la seta de emergencia
            msg = make_can_msg(node=self.cobid, index=0x0000, data=0x00, clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

    def botonera_callback(self, data):
        if data.data:
            # Control remoto (autonomo)
            msg = make_can_msg(node=self.cobid, index=0x0100, data=0x01, clock=self.get_clock().now().to_msg())
        else:
            # Control manual(botonera original)
            msg = make_can_msg(node=self.cobid, index=0x0100, data=0x00, clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

    def RutinaEncendido_callback(self, data):
        if data.data:
            # Encender rutina encendido
            msg = make_can_msg(node=self.cobid, index=0x0200, data=0x01, clock=self.get_clock().now().to_msg())
        else:
            # Apagar rutina encendido
            msg = make_can_msg(node=self.cobid, index=0x0200, data=0x00, clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

    def ModoTrabajo_callback(self, data):
        if data.data:
            # Encender modo de trabajo
            msg = make_can_msg(node=self.cobid, index=0x0300, data=0x01, clock=self.get_clock().now().to_msg())
        else:
            # Apagar modo de trabajo
            msg = make_can_msg(node=self.cobid, index=0x0300, data=0x00, clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

    def turbina_activacion_callback(self, data):
        if data.data:
            # Encender turbina
            msg = make_can_msg(node=self.cobid, index=0x0400, data=0x01, clock=self.get_clock().now().to_msg())
        else:
            # Apagar turbina
            msg = make_can_msg(node=self.cobid, index=0x0400, data=0x00, clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

    def turbina_rpm_callback(self, data: IntStamped):
        if data.data in [0, 1, 2]:
            if data.data == 0:
                # Dejar de modificar RPM de turbina
                msg = make_can_msg(node=self.cobid, index=0x0400, sub_index=1, data=0x00,
                                   clock=self.get_clock().now().to_msg())
            elif data.data == 1:
                # Disminuir RPM de turbina
                msg = make_can_msg(node=self.cobid, index=0x0400, sub_index=1, data=0x01,
                                   clock=self.get_clock().now().to_msg())
            else:  # data =2
                # Aumentar RPM de turbina
                msg = make_can_msg(node=self.cobid, index=0x0400, sub_index=1, data=0x02,
                                   clock=self.get_clock().now().to_msg())

            self.pub_CAN.publish(CANGroup(
                header=Header(stamp=self.get_clock().now().to_msg()),
                can_frames=[
                    msg
                ]
            ))
        else:
            self.logger.warn(f'Solicitud de RPM de turbina no valido {data.data}')

    def cepillos_centrales_activacion_callback(self, data):
        if data.data:
            # Encender cepillos centrales
            msg = make_can_msg(node=self.cobid, index=0x0500, sub_index=0x00, data=0x01,
                               clock=self.get_clock().now().to_msg())
        else:
            # Apagar cepillos centrales
            msg = make_can_msg(node=self.cobid, index=0x0500, sub_index=0x00, data=0x00,
                               clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

    def cepillos_centrales_movimiento_callback(self, data: IntStamped):
        if data.data in [0, 1, 2, 3, 4]:
            if data.data == 0:
                # Dejar de realizar movimiento
                msg = make_can_msg(node=self.cobid, index=0x0500, sub_index=1, data=0x00,
                                   clock=self.get_clock().now().to_msg())
            elif data.data == 1:
                # Bajar cepillos centrales
                msg = make_can_msg(node=self.cobid, index=0x0500, sub_index=1, data=0x01,
                                   clock=self.get_clock().now().to_msg())
            elif data.data == 2:
                # Subir cepillos centrales
                msg = make_can_msg(node=self.cobid, index=0x0500, sub_index=1, data=0x02,
                                   clock=self.get_clock().now().to_msg())
            elif data.data == 3:
                # Dedsplazar cepillos centrales a la izquierda
                msg = make_can_msg(node=self.cobid, index=0x0500, sub_index=1, data=0x03,
                                   clock=self.get_clock().now().to_msg())
            else:  # data = 4
                # Dedsplazar cepillos centrales a la derecha
                msg = make_can_msg(node=self.cobid, index=0x0500, sub_index=1, data=0x04,
                                   clock=self.get_clock().now().to_msg())

            self.pub_CAN.publish(CANGroup(
                header=Header(stamp=self.get_clock().now().to_msg()),
                can_frames=[
                    msg
                ]
            ))
        else:
            self.logger.warn(f'Solicitud de movimiento de cepillos centrales no valido {data.data}')

    def cepillos_centrales_agua_callback(self, data):
        if data.data:
            # Activar agua de los cepillos centrales
            msg = make_can_msg(node=self.cobid, index=0x0500, sub_index=0x02, data=0x01,
                               clock=self.get_clock().now().to_msg())
        else:
            # Desactivar agua de los cepillos centrales
            msg = make_can_msg(node=self.cobid, index=0x0500, sub_index=0x02, data=0x00,
                               clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

    def cepillos_centrales_luces_callback(self, data):
        if data.data:
            # Activar luz de los cepillos centrales
            msg = make_can_msg(node=self.cobid, index=0x0500, sub_index=0x03, data=0x01,
                               clock=self.get_clock().now().to_msg())
        else:
            # Desactivar luz de los cepillos centrales
            msg = make_can_msg(node=self.cobid, index=0x0500, sub_index=0x03, data=0x00,
                               clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

    def cepillos_frontal_activacion_callback(self, data: IntStamped):
        if data.data in [0, 1, 2]:
            if data.data == 0:
                # Apagar cepillo frontal
                msg = make_can_msg(node=self.cobid, index=0x0600, data=0x00,
                                   clock=self.get_clock().now().to_msg())
            elif data.data == 1:
                # Rotacion del cepillo frontal en sentido horario
                msg = make_can_msg(node=self.cobid, index=0x0600, data=0x01,
                                   clock=self.get_clock().now().to_msg())
            else:  # data = 2
                # Rotacion del cepillo frontal en sentido horario
                msg = make_can_msg(node=self.cobid, index=0x0600, data=0x02,
                                   clock=self.get_clock().now().to_msg())

            self.pub_CAN.publish(CANGroup(
                header=Header(stamp=self.get_clock().now().to_msg()),
                can_frames=[
                    msg
                ]
            ))
        else:
            self.logger.warn(f'Solicitud de movimiento de cepillos frontal no valido {data.data}')

    def cepillos_frontal_movimiento_callback(self, data: IntStamped):
        if data.data in [0, 1, 2, 3, 4]:
            if data.data == 0:
                # Dejar de realizar movimiento
                msg = make_can_msg(node=self.cobid, index=0x0600, sub_index=1, data=0x00,
                                   clock=self.get_clock().now().to_msg())
            elif data.data == 1:
                # Bajar cepillos frontal
                msg = make_can_msg(node=self.cobid, index=0x0600, sub_index=1, data=0x01,
                                   clock=self.get_clock().now().to_msg())
            elif data.data == 2:
                # Subir cepillos frontal
                msg = make_can_msg(node=self.cobid, index=0x0600, sub_index=1, data=0x02,
                                   clock=self.get_clock().now().to_msg())
            elif data.data == 3:
                # Dedsplazar cepillos v a la izquierda
                msg = make_can_msg(node=self.cobid, index=0x0600, sub_index=1, data=0x03,
                                   clock=self.get_clock().now().to_msg())
            else:  # data = 4
                # Dedsplazar cepillos frontal a la derecha
                msg = make_can_msg(node=self.cobid, index=0x0600, sub_index=1, data=0x04,
                                   clock=self.get_clock().now().to_msg())

            self.pub_CAN.publish(CANGroup(
                header=Header(stamp=self.get_clock().now().to_msg()),
                can_frames=[
                    msg
                ]
            ))
        else:
            self.logger.warn(f'Solicitud de movimiento de cepillos centrales no valido {data.data}')

    def cepillos_frontal_agua_callback(self, data):
        if data.data:
            # Activar agua del cepillo frontal
            msg = make_can_msg(node=self.cobid, index=0x0600, sub_index=0x02, data=0x01,
                               clock=self.get_clock().now().to_msg())
        else:
            # Desactivar agua del cepillo frontal
            msg = make_can_msg(node=self.cobid, index=0x0600, sub_index=0x02, data=0x00,
                               clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

    def cepillos_frontal_luces_callback(self, data):
        if data.data:
            # Activar luz del cepillo frontal
            msg = make_can_msg(node=self.cobid, index=0x0600, sub_index=0x03, data=0x01,
                               clock=self.get_clock().now().to_msg())
        else:
            # Desactivar luz del cepillo frontal
            msg = make_can_msg(node=self.cobid, index=0x0600, sub_index=0x03, data=0x00,
                               clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

    def cepillos_frontal_despliegue_callback(self, data):
        if data.data:
            # Desplegar/replegar cepillo frontal
            msg = make_can_msg(node=self.cobid, index=0x0600, sub_index=0x04, data=0x01,
                               clock=self.get_clock().now().to_msg())
        else:
            # Dejar de desplegar/replegar cepillo frontal
            msg = make_can_msg(node=self.cobid, index=0x0600, sub_index=0x04, data=0x00,
                               clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

    def cepillos_frontal_rotacion_callback(self, data: IntStamped):
        if data.data in [0, 1, 2]:
            if data.data == 0:
                # No rotar
                msg = make_can_msg(node=self.cobid, index=0x0600, sub_index=0x05, data=0x00,
                                   clock=self.get_clock().now().to_msg())
            elif data.data == 1:
                # Rotar en sentido horario
                msg = make_can_msg(node=self.cobid, index=0x0600, sub_index=0x05, data=0x01,
                                   clock=self.get_clock().now().to_msg())
            else:  # data = 2
                # Rotar en sentido antihorario
                msg = make_can_msg(node=self.cobid, index=0x0600, sub_index=0x05, data=0x02,
                                   clock=self.get_clock().now().to_msg())

            self.pub_CAN.publish(CANGroup(
                header=Header(stamp=self.get_clock().now().to_msg()),
                can_frames=[
                    msg
                ]
            ))
        else:
            self.logger.warn(f'Solicitud de rotacion de cepillos frontal no valido {data.data}')

    def cepillos_frontal_angulo_callback(self, data: IntStamped):
        if data.data in [0, 1, 2]:
            if data.data == 0:
                # No rotar
                msg = make_can_msg(node=self.cobid, index=0x0600, sub_index=0x06, data=0x00,
                                   clock=self.get_clock().now().to_msg())
            elif data.data == 1:
                # Rotar angulo en sentido horario
                msg = make_can_msg(node=self.cobid, index=0x0600, sub_index=0x06, data=0x01,
                                   clock=self.get_clock().now().to_msg())
            else:  # data = 2
                # Rotar angulo en sentido antihorario
                msg = make_can_msg(node=self.cobid, index=0x0600, sub_index=0x06, data=0x02,
                                   clock=self.get_clock().now().to_msg())

            self.pub_CAN.publish(CANGroup(
                header=Header(stamp=self.get_clock().now().to_msg()),
                can_frames=[
                    msg
                ]
            ))
        else:
            self.logger.warn(f'Solicitud de angulo de cepillos frontal no valido {data.data}')

    def suspension_callback(self, data):
        if data.data:
            # Subir suspension
            msg = make_can_msg(node=self.cobid, index=0x0700, data=0x01,
                               clock=self.get_clock().now().to_msg())
        else:
            # Dejar de subir suspension
            msg = make_can_msg(node=self.cobid, index=0x0700, data=0x00,
                               clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

    def pistolas_callback(self, data):
        if data.data:
            # Activar pitolas de agua
            msg = make_can_msg(node=self.cobid, index=0x0800, data=0x01,
                               clock=self.get_clock().now().to_msg())
        else:
            # Desactivar pitolas de agua
            msg = make_can_msg(node=self.cobid, index=0x0800, data=0x00,
                               clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

    def faldon_callback(self, data: IntStamped):
        if data.data in [0, 1, 2]:
            if data.data == 0:
                # Dejar de cambiar el faldon
                msg = make_can_msg(node=self.cobid, index=0x0900, data=0x00,
                                   clock=self.get_clock().now().to_msg())
            elif data.data == 1:
                # Bajar faldon
                msg = make_can_msg(node=self.cobid, index=0x0900, data=0x01,
                                   clock=self.get_clock().now().to_msg())
            else:  # data = 2
                # Subir faldon
                msg = make_can_msg(node=self.cobid, index=0x0900, data=0x02,
                                   clock=self.get_clock().now().to_msg())

            self.pub_CAN.publish(CANGroup(
                header=Header(stamp=self.get_clock().now().to_msg()),
                can_frames=[
                    msg
                ]
            ))
        else:
            self.logger.warn(f'Solicitud de faldon no valido {data.data}')

    def chupon_callback(self, data: IntStamped):
        if data.data in [0, 1, 2]:
            if data.data == 0:
                # Dejar de cambiar el chupon
                msg = make_can_msg(node=self.cobid, index=0x0A00, data=0x00,
                                   clock=self.get_clock().now().to_msg())
            elif data.data == 1:
                # Bajar chupon
                msg = make_can_msg(node=self.cobid, index=0x0A00, data=0x01,
                                   clock=self.get_clock().now().to_msg())
            else:  # data = 2
                # Subir chupon
                msg = make_can_msg(node=self.cobid, index=0x0A00, data=0x02,
                                   clock=self.get_clock().now().to_msg())

            self.pub_CAN.publish(CANGroup(
                header=Header(stamp=self.get_clock().now().to_msg()),
                can_frames=[
                    msg
                ]
            ))
        else:
            self.logger.warn(f'Solicitud de chupon no valido {data.data}')

    def puertas_callback(self, data):
        if data.data:
            # Dejar de pulsar bloqueo de puertas
            msg = make_can_msg(node=self.cobid, index=0x0B00, data=0x01,
                               clock=self.get_clock().now().to_msg())
        else:
            # Bloquear/Desbloquear puertas
            msg = make_can_msg(node=self.cobid, index=0x0B00, data=0x00,
                               clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

    """
    def enable(self, data):
        if data.data:
            msg = make_can_msg(node=self.cobid, index=0x0003, data=0x01, clock=self.get_clock().now().to_msg())
        else:
            msg = make_can_msg(node=self.cobid, index=0x0003, data=0x00, clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))

    def consigna(self, data):
        msg = make_can_msg(node=self.cobid, index=0x0001, data=int(data.data * 100),
                           clock=self.get_clock().now().to_msg())
        self.pub_CAN.publish(CANGroup(
            header=Header(stamp=self.get_clock().now().to_msg()),
            can_frames=[
                msg
            ]
        ))
        
    """

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
            self.timer_heartbeat.cancel()
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = NIMBUSNode()
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
