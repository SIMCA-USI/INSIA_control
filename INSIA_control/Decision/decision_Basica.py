from traceback import format_exc
from numpy import interp

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy

from std_msgs.msg import Float64, Bool, String, UInt8
from mdef_a2sat.msg import RespConduccion, PetConduccion, EstadoMision, ModoMision


# noinspection PyBroadException
class Decision(Node):
    def __init__(self, name: str = 'unknown_control'):
        super().__init__(node_name=name, namespace='control', start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        # Logging configuration
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)

        self.id_plataforma = self.get_parameter('id_platform').value

        self.TeleOperacion = None
        self.wp = None
        self.mode = 2  # Teleoperacion
        self.estado_wp = 0
        self.rango_volante = self.get_parameter('rango_volante').value

        self.lidar_obs = 0

        self.create_subscription(msg_type=PetConduccion, topic='/' + self.id_plataforma + '/Decision',
                                 callback=self.sub_pet_conduccion_wp, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=PetConduccion, topic='/TeleOperacion',
                                 callback=self.sub_pet_conduccion, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=ModoMision, topic='/Misiones/Modo',
                                 callback=self.modo_mision, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=UInt8, topic='/' + self.id_plataforma + '/Lidar/Obstaculo',
                                 callback=self.sub_lidar_obs, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=UInt8, topic='/' + self.id_plataforma + '/WP_Status',
                                 callback=self.sub_wp_status, qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_decision = self.create_publisher(msg_type=PetConduccion,
                                                  topic='/' + self.id_plataforma + '/TeleOperacion',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_estado = self.create_publisher(msg_type=EstadoMision,
                                                topic='/Misiones/Estado',
                                                qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_control = self.create_timer(1 / 10, self.decision)
        self.timer_estado_mision = self.create_timer(1, self.estado_mision)
        self.parada_emergencia = False

    # Subscribers test

    def sub_lidar_obs(self, data):
        self.lidar_obs = data.data

    def sub_wp_status(self, data):
        self.estado_wp = data.data

    def modo_mision(self, data):
        if data.id_plataforma == self.id_plataforma:
            self.logger.debug(f'Modo {data.modo_mision}')
            self.mode = int(data.modo_mision)
        else:
            self.logger.debug(f'ID incorrecto: {data.id_plataforma}')

    def sub_pet_conduccion(self, data):
        if data.id_plataforma == self.id_plataforma:
            if self.get_parameter('100_volante_conduccion').value:
                data.direccion = interp(data.direccion, (-100, 100), (-self.rango_volante, self.rango_volante))
            else:
                if not self.get_parameter('operador_angulo_volante-rueda').value:
                    data.direccion *= self.get_parameter('steering_wheels_conversion').value
            if self.TeleOperacion != data:
                self.TeleOperacion = data
                if self.mode == 2:
                    self.timer_control.reset()
                    self.decision()
        else:
            self.logger.debug(f'Mensaje TeleOperación con destino: {self.TeleOperacion.id_plataforma}')

    def sub_pet_conduccion_wp(self, data):
        if self.wp != data:
            self.wp = data
            if self.mode == 1:
                self.timer_control.reset()
                self.decision()

    def estado_mision(self):
        self.pub_estado.publish(
            EstadoMision(
                id_plataforma=self.id_plataforma,
                modo_mision=self.mode,
                estado_mision=self.estado_wp
            )
        )

    def decision(self):
        if self.mode == 0:  # Manual
            self.logger.debug(f'Modo manual')
            self.pub_decision.publish(
                PetConduccion(
                    id_plataforma=self.id_plataforma,
                    b_velocidad=False,
                    b_direccion=False,
                    b_marchas=False
                )
            )
        elif self.mode == 1:  # Waypoints
            self.logger.debug(f'Modo Waypoints')
            if self.wp is not None:
                if self.lidar_obs == 2:
                    self.parada_emergencia = True
                    self.pub_decision.publish(
                        PetConduccion(id_plataforma=self.TeleOperacion.id_plataforma,
                                      direccion=self.TeleOperacion.direccion,
                                      velocidad=self.TeleOperacion.velocidad,
                                      marchas=self.TeleOperacion.marchas,
                                      desact_parada_emergencia=True,
                                      override=self.TeleOperacion.override,
                                      b_velocidad=self.TeleOperacion.b_velocidad,
                                      b_direccion=self.TeleOperacion.b_direccion,
                                      b_marchas=self.TeleOperacion.b_marchas)
                    )
                else:
                    self.pub_decision.publish(self.wp)
            else:
                self.pub_decision.publish(
                    PetConduccion(id_plataforma="No teleOperacion", direccion=0., velocidad=0., marchas="N",
                                  desact_parada_emergencia=self.parada_emergencia, override=False,
                                  b_velocidad=False,
                                  b_direccion=False, b_marchas=False))

        elif self.mode == 2:  # Teleoperado
            self.logger.debug(f'Modo Teleoperado')
            if self.TeleOperacion is not None:
                if self.lidar_obs == 2 and not self.TeleOperacion.override:
                    self.parada_emergencia = True
                    self.pub_decision.publish(
                        PetConduccion(id_plataforma=self.TeleOperacion.id_plataforma,
                                      direccion=self.TeleOperacion.direccion,
                                      velocidad=self.TeleOperacion.velocidad,
                                      marchas=self.TeleOperacion.marchas,
                                      desact_parada_emergencia=True,
                                      override=self.TeleOperacion.override,
                                      b_velocidad=self.TeleOperacion.b_velocidad,
                                      b_direccion=self.TeleOperacion.b_direccion,
                                      b_marchas=self.TeleOperacion.b_marchas))
                else:
                    teleoperacion = self.TeleOperacion
                    if self.TeleOperacion.desact_parada_emergencia:
                        teleoperacion.desact_parada_emergencia = False
                        self.parada_emergencia = False
                    else:
                        teleoperacion.desact_parada_emergencia = self.parada_emergencia
                    self.pub_decision.publish(teleoperacion)
            else:
                if self.lidar_obs:
                    self.parada_emergencia = True
                    self.pub_decision.publish(
                        PetConduccion(id_plataforma="No teleOperacion", direccion=0., velocidad=0., marchas="N",
                                      desact_parada_emergencia=True, override=False, b_velocidad=False,
                                      b_direccion=False,
                                      b_marchas=False))
                else:
                    self.pub_decision.publish(
                        PetConduccion(id_plataforma="No teleOperacion", direccion=0., velocidad=0., marchas="N",
                                      desact_parada_emergencia=self.parada_emergencia, override=False,
                                      b_velocidad=False,
                                      b_direccion=False, b_marchas=False))
        else:
            self.logger.error(f'Error in mode: {self.mode}')

    def shutdown(self):
        self.timer_control.cancel()


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = Decision(name='Decision')
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print('Decision: Keyboard interrupt')
        manager.shutdown()
        manager.destroy_node()
    except Exception:
        print(format_exc())


if __name__ == '__main__':
    main()
