import os
from traceback import format_exc

import rclpy
import yaml
from insia_msg.msg import StringStamped, BoolStamped, Telemetry, ControladorFloat, EPOSConsigna
from numpy import interp
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader


class BrakeNode(Node):

    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='BrakeNode', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False
        params = vehicle_parameters.get('brake')
        self.device_range = params['range']
        self.telemetry = Telemetry()
        self.controller = None

        self.create_subscription(msg_type=ControladorFloat,
                                 topic=self.get_name(),
                                 callback=self.controller_update, qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped,
                                                   topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_enable = self.create_publisher(msg_type=BoolStamped,
                                                topic='MCD60_Freno/Enable',
                                                qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_target = self.create_publisher(msg_type=EPOSConsigna, topic='MCD60_Freno/TargetPosition',
                                                qos_profile=HistoryPolicy.KEEP_LAST)

        # Servicio para la calibración del freno
        # self.srv_brake_calibration = self.create_service(BrakeCalibration, 'brake_calibration', self.enable_calibration)
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def controller_update(self, data):
        self.controller = data
        self.pub_enable.publish(BoolStamped(
            header=Header(stamp=self.get_clock().now().to_msg()),
            data=True
        ))
        if self.controller.enable:
            self.pub_target.publish(EPOSConsigna(
                header=Header(stamp=self.get_clock().now().to_msg()),
                position=int(interp(self.controller.target, (0, 1), self.device_range)),
                mode=EPOSConsigna.ABSOLUTO
            ))
        else:
            self.pub_target.publish(EPOSConsigna(
                header=Header(stamp=self.get_clock().now().to_msg()),
                position=0,
                mode=EPOSConsigna.ABSOLUTO
            ))

    def enable_calibration(self, request, response):
        if not request.bool.data:
            response.data = 0
            pass
        else:
            # self.calibration()
            response.data = 1
        return response

    def calibration(self):
        # TODO: Crear proceso de calibracion
        pass

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
            self.pub_target.publish(EPOSConsigna(
                header=Header(stamp=self.get_clock().now().to_msg()),
                position=0,
                mode=EPOSConsigna.ABSOLUTO
            ))
            # self.pub_enable.publish(BoolStamped(
            #     header=Header(stamp=self.get_clock().now().to_msg()),
            #     data=False
            # ))
            self.timer_heartbeat.cancel()
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = BrakeNode()
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
