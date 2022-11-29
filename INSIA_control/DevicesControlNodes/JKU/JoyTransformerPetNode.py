import os
from traceback import format_exc

import rclpy
import yaml
from insia_msg.msg import StringStamped, PetConduccion
from numpy import interp
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from sensor_msgs.msg import Joy as JoyController
from std_msgs.msg import Header
from yaml.loader import SafeLoader

dict_buttons_sony = {
    'x': 0,
    'circle': 1,
    'triangle': 2,
    'square': 3,
    'l1': 4,
    'r1': 5,
    'l2': 6,
    'r2': 7,
    'lb': 8,
    'rb': 9,
    'l3': 10,
    'r3': 11,
}


class JoyTransformerPetNode(Node):

    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='JoyTransformerPetNode', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True, allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)
        self.shutdown_flag = False
        params = vehicle_parameters.get('steering')
        self.device_range_steering = params['wheel_range']
        params = vehicle_parameters.get('speed')
        self.device_range_speed = params['range']
        self.b_steering = False
        self.b_brake = False
        self.b_throttle = False
        self.b_gears = False
        self.last_msg = None

        self.create_subscription(msg_type=JoyController, topic=self.get_name(), callback=self.controller_update,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_joy = self.create_publisher(msg_type=PetConduccion, topic='Joy_transformed_Pet',
                                             qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def check_button(self, msg: JoyController, button: str, master: bool = False) -> bool:
        if button in dict_buttons_sony.keys():
            b = dict_buttons_sony[button]
            if master:
                return msg.buttons[b]
            return msg.buttons[b] != self.last_msg.buttons[b] and msg.buttons[b]
        else:
            self.logger.error(f'Wrong def in code button: {button}')
            return False

    def controller_update(self, msg: JoyController):
        if self.last_msg is None:
            self.last_msg = msg
        try:
            # Desactivar all
            if self.check_button(msg, 'l1', master=True):
                print('entering')
                self.b_steering = False
                self.b_throttle = False
                self.b_brake = False
                self.b_gears = False
            else:
                # Activar all
                if self.check_button(msg, 'r1', master=True):
                    self.b_steering = True
                    self.b_throttle = True
                    self.b_brake = True
                    self.b_gears = True
                else:
                    # Cambiar de estado cada b al pulsar el boton
                    if self.check_button(msg, 'circle'):
                        self.b_steering = not self.b_steering
                    if self.check_button(msg, 'x'):
                        self.b_throttle = not self.b_throttle
                    if self.check_button(msg, 'square'):
                        self.b_brake = not self.b_brake
                    if self.check_button(msg, 'triangle'):
                        self.b_gears = not self.b_gears

            self.pub_joy.publish(PetConduccion(
                header=Header(stamp=self.get_clock().now().to_msg()),
                b_steering=self.b_steering,
                b_throttle=self.b_throttle,
                b_brake=self.b_brake,
                b_gear=self.b_gears,
                speed=interp(msg.axes[1], (0, 1), self.device_range_speed),
                steering=interp(msg.axes[3], (-1, 1), self.device_range_steering)
            ))
            self.last_msg = msg
        except Exception as e:
            self.logger.error(f'{e}')
            self.pub_joy.publish(PetConduccion(
                header=Header(stamp=self.get_clock().now().to_msg()),
                b_steering=False,
                b_throttle=False,
                b_brake=False,
                speed=0.,
                steering=0.
            ))

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
            self.timer_heartbeat.cancel()
        except Exception as e:
            self.logger.error(f'Exception in shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = JoyTransformerPetNode()
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
