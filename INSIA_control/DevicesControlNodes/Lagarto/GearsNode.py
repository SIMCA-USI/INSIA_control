import os

import rclpy
import yaml
from insia_msg.msg import ControladorStr, StringStamped, Telemetry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader


class Gears_Lagarto(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='Node', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        self.id_plataforma = vehicle_parameters['id_vehicle']
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False
        self.telemetry = Telemetry()
        self.current_gear = 'N'  # Eliminar cuando se recojan las marchas del vehiculo
        with open(os.getenv('ROS_WS') + '/src/INSIA_control/INSIA_control/diccionarios/' + self.get_parameter(
                'dictionary').value) as f:
            self.avaliable_gears = yaml.load(f, Loader=yaml.FullLoader)

        self.create_subscription(msg_type=Telemetry, topic='Telemetry', callback=self.telemetry_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_gear = self.create_publisher(msg_type=StringStamped, topic='Arduino_Gears/Consigna',
                                              qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=ControladorStr, topic=self.get_name(), callback=self.control,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def telemetry_callback(self, telemetry: Telemetry):
        self.telemetry = telemetry

    def control(self, controlador: ControladorStr):
        try:
            if controlador.enable:
                self.logger.debug(f"Request update gear value to {controlador.target}")
                if controlador.target in self.avaliable_gears.keys():
                    target_gear = controlador.target
                    if (self.current_gear in ['D1', 'D2', 'D3', 'D'] and controlador.target in ['R']) \
                            or (self.current_gear in ['R'] and controlador.target in ['D1', 'D2', 'D3', 'D']) \
                            or (self.current_gear in ['M'] and controlador.target in ['D1', 'D2', 'D3', 'D', 'R']):
                        target_gear = 'N'
                    if self.telemetry.speed == 0 or (self.telemetry.speed != 0 and (target_gear in ['N', 'M'] or (
                            self.current_gear in ['D1', 'D2', 'D3', 'D'] and target_gear in ['D1', 'D2', 'D3', 'D']))):
                        self.current_gear = target_gear
                        self.pub_gear.publish(
                            StringStamped(
                                header=Header(stamp=self.get_clock().now().to_msg()),
                                data=target_gear
                            )
                        )
                else:
                    self.logger.error(f"Invalid gear value {controlador.target}")
            else:
                self.pub_gear.publish(
                    StringStamped(
                        header=Header(stamp=self.get_clock().now().to_msg()),
                        data='M'
                    )
                )
        except Exception as e:
            self.logger.error(f'Error in gears control: {e}')
        # Cambiar a este modelo si se consigue sacar la marcha del vehiculo
        # try:
        #     self.logger.debug(f"Request update gear value to {controlador.target}V")
        #     if controlador.target in self.avaliable_gears.keys():
        #         target_gear = controlador.target
        #         if (self.current_gear in ['D1', 'D2', 'D3', 'D'] and controlador.target in ['R']) \
        #                 or (self.current_gear in ['R'] and controlador.target in ['D1', 'D2', 'D3', 'D']) \
        #                 or (self.current_gear in ['M'] and controlador.target in ['D1', 'D2', 'D3', 'D', 'R']):
        #             target_gear = 'N'
        #         if self.telemetry.speed == 0 or (self.telemetry.speed != 0 and (target_gear in ['N', 'M'] or (
        #                 self.current_gear in ['D1', 'D2', 'D3', 'D'] and target_gear in ['D1', 'D2', 'D3', 'D']))):
        #             self.pub_gear.publish(
        #                 StringStamped(
        #                     header=Header(stamp=self.get_clock().now().to_msg()),
        #                     data=target_gear
        #                 )
        #             )
        #     else:
        #         self.logger.error(f" invalid gear value {controlador.target}V")
        # except Exception as e:
        #     self.logger.error(f'Error in gears control: {e}')

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
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = Gears_Lagarto()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
