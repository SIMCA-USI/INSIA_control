import os

import rclpy
import yaml
from insia_msg.msg import Telemetry, StringStamped, PetConduccion, ControladorFloat
from numpy import interp
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader


class LateralControlNode(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='Node', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)
        self.id_plataforma = vehicle_parameters['id_vehicle']
        self.wheel_range = vehicle_parameters['steering']['wheel_range']
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False
        self.telemetry = Telemetry()
        self.pet_conduccion = PetConduccion()

        self.create_subscription(msg_type=PetConduccion, topic='Decision/Output', callback=self.decision_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=Telemetry, topic='Telemetry', callback=self.telemetry_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)
        self.pub_steering = self.create_publisher(msg_type=ControladorFloat, topic='Steering',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_control = self.create_timer(1 / 10, self.control)
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def control(self):
        if self.pet_conduccion.b_steering:
            target_steering = self.pet_conduccion.steering  # Angulo de volante
            current_steering = self.telemetry.steering  # Angulo de volante
            error = min(max(target_steering, self.wheel_range[0]), self.wheel_range[1]) - current_steering

            self.pub_steering.publish(
                ControladorFloat(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    enable=True,
                    target=interp(error, self.wheel_range, (-1., 1.))
                )
            )
        else:
            self.pub_steering.publish(
                ControladorFloat(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    enable=False,
                    target=0.
                )
            )

    def decision_callback(self, decision: PetConduccion):
        self.pet_conduccion = decision

    def telemetry_callback(self, telemetry: Telemetry):
        self.telemetry = telemetry

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
        manager = LateralControlNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
