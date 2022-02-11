import os

import numpy as np
import rclpy
import yaml
from insia_msg.msg import CANGroup, StringStamped, EPOSConsigna, EPOSDigital, BoolStamped, IntStamped, Telemetry, ControllerFloat
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader

from src.utils.utils import make_can_msg
from traceback import format_exc
from src.utils.pid import PIDF


class SteeringNode(Node):

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

        self.steering_sensor_error = vehicle_parameters['steering_sensor_error']
        self.steering_conversion = vehicle_parameters['steering_conversion']
        self.telemetry = Telemetry()
        self.controller = None

        self.create_subscription(msg_type=ControllerFloat,
                                 topic='/' + vehicle_parameters['id_vehicle'] + self.get_name(),
                                 callback=self.controller_update, qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbit = self.create_publisher(msg_type=StringStamped,
                                                  topic='/' + vehicle_parameters['id_vehicle'] + '/Heartbit',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_enable = self.create_publisher(msg_type=BoolStamped,
                                                topic='/' + vehicle_parameters['id_vehicle'] + '/EPOS4_Volante/Enable',
                                                qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_target = self.create_publisher(msg_type=IntStamped, topic='/' + vehicle_parameters[
            'id_vehicle'] + '/EPOS4_Volante/TargetTorque', qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbit = self.create_timer(1, self.publish_heartbit)

    def controller_update(self, data):
        self.controller = data.data
        self.pub_enable.publish(BoolStamped(
            header=Header(stamp=self.get_clock().now().to_msg()),
            data=self.controller.enable
        ))
        if self.controller.enable:
            self.puh_target.publish(IntStamped(
                header=Header(stamp=self.get_clock().now().to_msg()),
                data=(self.controller.target-self.steering_sensor_error)*self.steering_conversion#Alpha volante
            ))

    def publish_heartbit(self):
        msg = StringStamped(
            data=self.get_name()
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_heartbit.publish(msg)

    def shutdown(self):
        try:
            self.shutdown_flag = True
            self.timer_heartbit.cancel()
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = SteeringNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print('CANADAC: Keyboard interrupt')
    except Exception as e:
        format_exc()
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
