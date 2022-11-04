import os
from traceback import format_exc

import rclpy
import yaml
from insia_msg.msg import StringStamped, PetConduccion, BoolStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from yaml.loader import SafeLoader


class PathPlanning(Node):

    def parameters_callback(self, params):
        for param in params:
            if param.name == "log_level":
                self.logger.set_level(param.value)
        return SetParametersResult(successful=True)

    def __init__(self, name: str = 'unknown_control'):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='Node', namespace=vehicle_parameters['id_vehicle'], start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        # Logging configuration
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)

        self.add_on_set_parameters_callback(self.parameters_callback)
        self.flag = False

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped,
                                                   topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_PathPlanning = self.create_publisher(msg_type=PetConduccion,
                                                      topic='PathPlanning',
                                                      qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=PetConduccion, topic='Lidar',
                                 callback=self.sub_lidar, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=PetConduccion, topic='WP',
                                 callback=self.sub_wp, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic='PFlag',
                                 callback=self.sub_flag, qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def sub_lidar(self, data: PetConduccion):
        if self.flag:
            self.pub_PathPlanning.publish(data)

    def sub_wp(self, data: PetConduccion):
        if not self.flag:
            self.pub_PathPlanning.publish(data)

    def sub_flag(self, data: BoolStamped):
        if self.flag != data.data:
            self.flag = data.data
            if self.flag:
                self.logger.debug(f'Change to use lidar')
            else:
                self.logger.debug(f'Change to use WayPoints')

    def publish_heartbeat(self):
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
        manager = PathPlanning(name='PathPlanning')
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print('PathPlanning: Keyboard interrupt')
        manager.shutdown()
        manager.destroy_node()
    except Exception:
        print(format_exc())


if __name__ == '__main__':
    main()
