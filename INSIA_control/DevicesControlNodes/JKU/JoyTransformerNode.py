import os
from traceback import format_exc

import rclpy
import yaml
from insia_msg.msg import StringStamped, Joy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from sensor_msgs.msg import Joy as JoyController
from std_msgs.msg import Header
from yaml.loader import SafeLoader


class JoyTransformerNode(Node):

    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='SteeringNode', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False

        self.create_subscription(msg_type=JoyController,
                                 topic=self.get_name(),
                                 callback=self.controller_update, qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_joy = self.create_publisher(msg_type=Joy, topic='Joy_transformed', qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def controller_update(self, msg: JoyController):
        try:
            self.pub_joy.publish(Joy(
                header=Header(stamp=self.get_clock().now().to_msg()),
                acceleration=float(msg.axes[1]),
                steering=float(msg.axes[3])
            ))
        except Exception:
            # In case that msg is empty or any other problem
            self.pub_joy.publish(
                Joy(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    acceleration=-1.,
                    steering=0.
                )
            )

    def publish_heartbeat(self):
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
        manager = JoyTransformerNode()
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
