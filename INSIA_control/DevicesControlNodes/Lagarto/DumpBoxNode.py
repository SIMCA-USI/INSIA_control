import os

import rclpy
import yaml
from insia_msg.msg import ControladorStr, StringStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from yaml.loader import SafeLoader
from std_msgs.msg import Header


class DumpBoxLagarto(Node):
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
        self.target_DumpBox = None

        self.pub_heartbit = self.create_publisher(msg_type=StringStamped,
                                                  topic='/' + vehicle_parameters['id_vehicle'] + '/Heartbit',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_dump_box = self.create_publisher(msg_type=StringStamped, topic='/' + vehicle_parameters[
            'id_vehicle'] + '/Arduino_DumpBox/Consigna', qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=ControladorStr,
                                 topic='/' + vehicle_parameters['id_vehicle'] + '/' + self.get_name(),
                                 callback=self.control, qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbit = self.create_timer(1, self.publish_heartbit)
        self.timer_Dump_box = self.create_timer(1, self.publish_DumpBox)

    def control(self, controlador):
        if self.target_DumpBox != controlador.target:
            self.target_DumpBox = controlador.target
            self.publish_DumpBox()

    def publish_DumpBox(self):
        if self.target_DumpBox is not None:
            self.pub_dump_box.publish(
                StringStamped(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    data=self.target_DumpBox
                )
            )

    def publish_heartbit(self):
        msg = StringStamped(
            data=self.get_name()
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_heartbit.publish(msg)

    def shutdown(self):
        try:
            self.shutdown_flag = True
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = DumpBoxLagarto()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print('Node: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
