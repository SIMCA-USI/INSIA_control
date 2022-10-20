import os

import rclpy
import yaml
from insia_msg.msg import Telemetry, StringStamped, PetConduccion, ControladorStr
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader


class GearsControlNode(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='GearsControlNode', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.id_plataforma = vehicle_parameters['id_vehicle']
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False
        self.control_msg: PetConduccion = PetConduccion()
        self.current_speed = 0.
        self.steering_range = vehicle_parameters['steering']['wheel_range']
        self.default_gear = self.get_parameters_by_prefix('default')

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_gears = self.create_publisher(msg_type=ControladorStr, topic='Gears',
                                               qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=Telemetry, topic='Telemetry', callback=self.telemetry_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=PetConduccion, topic='Decision/Output', callback=self.decision,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)
        self.timer_control = self.create_timer(1 / 2, self.control_loop)

    def telemetry_callback(self, telemetry: Telemetry):
        self.current_speed = telemetry.speed

    def control_loop(self):
        if self.control_msg.b_gear and self.control_msg.gear != '':
            self.logger.debug('Gears enabled')
            if self.current_speed < 0.5:
                self.pub_gears.publish(ControladorStr(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    enable=self.control_msg.b_gear,
                    target=self.control_msg.gear
                ))
        else:
            self.logger.debug('Gears disabled')
            self.pub_gears.publish(ControladorStr(
                header=Header(stamp=self.get_clock().now().to_msg()),
                enable=self.control_msg.b_gear,
            ))

    def decision(self, decision):
        self.control_msg = decision

    def publish_heartbeat(self):
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
        manager = GearsControlNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
