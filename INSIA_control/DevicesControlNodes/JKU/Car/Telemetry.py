import os

import rclpy
import yaml
from insia_msg.msg import Telemetry, StringStamped
from numpy import interp
from openpilot_msgs.msg import CarState
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from yaml.loader import SafeLoader
from std_msgs.msg import Float64

dict_gears = {
    0: 'U',
    1: 'P',
    2: 'D',
    3: 'N',
    4: 'R',
    5: 'S',
    6: 'L',
    7: 'B',
    8: 'E',
    9: 'M'
}


class VehicleNode(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='VehicleDecoder', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        self.id_plataforma = vehicle_parameters['id_vehicle']
        self.steering_sensor_error = vehicle_parameters['steering']['sensor_error']
        self.steering_sensor_inverted = vehicle_parameters['steering']['inverted']
        self.steering_wheel_conversion = vehicle_parameters['steering']['steering_wheel_conversion']
        self.throttle_range = vehicle_parameters['throttle']['range']
        self.brake_range = vehicle_parameters['brake']['range']
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False
        self.velocidad = 0.
        self.volante = 0.

        self.create_subscription(msg_type=CarState, topic='carState', callback=self.publish_telemetry,
                                 qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=Float64, topic='Velocity', callback=self.publish_telemetry_fake_speed,
                                 qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=Float64, topic='Volante', callback=self.publish_telemetry_fake_steering,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_telemetry = self.create_publisher(msg_type=Telemetry, topic='Telemetry',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

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

    def publish_telemetry_fake_speed(self, data: Float64):
        """
        Receive openpilot msg and translate it to telemetry msg
        :param data: openpilot/CarState
        :return: Publish on Telemetry
        """
        self.velocidad = data.data * 3.6
        self.pub_telemetry.publish(Telemetry(
            id_plataforma=self.id_plataforma,
            speed=self.velocidad,  # TODO: Revisar si realmente esta en ms o kmh
            steering=self.volante,
        ))

    def publish_telemetry_fake_steering(self, data: Float64):
        """
        Receive openpilot msg and translate it to telemetry msg
        :param data: openpilot/CarState
        :return: Publish on Telemetry
        """
        self.volante = (data.data - self.steering_sensor_error) * (
            -1 if self.steering_sensor_inverted else 1)
        self.pub_telemetry.publish(Telemetry(
            id_plataforma=self.id_plataforma,
            speed=self.velocidad,  # TODO: Revisar si realmente esta en ms o kmh
            steering=self.volante,
        ))

    def publish_telemetry(self, data: CarState):
        """
        Receive openpilot msg and translate it to telemetry msg
        :param data: openpilot/CarState
        :return: Publish on Telemetry
        """
        steer = (data.steeringangledeg - self.steering_sensor_error) * (
            -1 if self.steering_sensor_inverted else 1)
        self.pub_telemetry.publish(Telemetry(
            id_plataforma=self.id_plataforma,
            speed=data.vegoraw * 3.6,  # TODO: Revisar si realmente esta en ms o kmh
            steering=steer,
            steering_deg=steer / self.steering_wheel_conversion,
            throttle=int(interp(data.gas, self.throttle_range, (0, 100))),
            brake=int(interp(data.brake, self.brake_range, (0, 100))),
            gears=dict_gears.get(data.gearshifter, 'U')
        ))

    def shutdown(self):
        try:
            self.shutdown_flag = True
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = VehicleNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
