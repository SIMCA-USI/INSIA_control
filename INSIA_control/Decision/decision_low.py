import os
import time
from copy import deepcopy
from traceback import format_exc

import rclpy
import yaml
from insia_msg.msg import StringStamped, PetConduccion, MasterSwitch, ModoMision
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader


class Decision(Node):

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

        self.id_plataforma = vehicle_parameters['id_vehicle']
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.master_switch = MasterSwitch()
        self.tele_msg = None
        self.tele_ttl = self.get_parameter_or('tele_ttl', Parameter(name='tele_ttl', value=1))
        self.wp_msg = None
        self.wp_ttl = self.get_parameter_or('wp_ttl', Parameter(name='wp_ttl', value=1))
        self.mode = ModoMision.TELE_OPERADO

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped,
                                                   topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_decision = self.create_publisher(msg_type=PetConduccion,
                                                  topic='Decision/Output',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=MasterSwitch,
                                 topic='MasterSwitch',
                                 callback=self.master_switch_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=PetConduccion, topic='PathPlanning',
                                 callback=self.sub_wp, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=PetConduccion, topic='TeleOperacion',
                                 callback=self.sub_tele, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=ModoMision, topic='Mode',
                                 callback=self.modo_mision, qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_control = self.create_timer(1 / 10, self.decision)
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def master_switch_callback(self, data: MasterSwitch):
        if self.master_switch.b_gear != data.b_gear or self.master_switch.b_brake != data.b_brake or \
                self.master_switch.b_steering != data.b_steering or self.master_switch.b_throttle != data.b_throttle:
            self.master_switch = data
            self.logger.debug(
                f'Changed MS: Brake: {data.b_brake} Throttle: {data.b_throttle} Steering: {data.b_steering} Gears: {data.b_gear}')

    def modo_mision(self, data):
        self.logger.debug(f'Modo {data.modo_mision}')
        self.mode = data.modo_mision

    def sub_tele(self, data):
        self.tele_msg = data
        if self.mode == ModoMision.TELE_OPERADO:
            self.timer_control.reset()
            self.decision()

    def sub_wp(self, data):
        self.wp_msg = data
        if self.mode == ModoMision.AUTONOMO:
            self.timer_control.reset()
            self.decision()

    def manual(self) -> PetConduccion:
        return PetConduccion(
            header=Header(stamp=self.get_clock().now().to_msg()),
            id_plataforma=self.id_plataforma,
            b_throttle=False,
            b_brake=False,
            b_steering=False,
            b_gear=False,
            steering=0.,
            speed=0.
        )

    def is_valid(self, msg: PetConduccion, ttl) -> bool:
        if msg is not None:
            t = msg.header.stamp.sec
            if t != 0:
                t_alive = (time.time() - t)
                if t_alive > ttl:
                    return False
                else:
                    return True
            else:
                self.logger.debug(f'Msg with stamp = 0 {msg}')
                return True
        else:
            return False

    def decision(self):
        msg = None
        if self.mode == ModoMision.MANUAL:  # Manual

            self.logger.debug(f'Modo manual')
            msg = self.manual()

        elif self.mode == ModoMision.AUTONOMO:  # Waypoints

            self.logger.debug(f'Modo Waypoints')
            if self.is_valid(self.wp_msg, self.wp_ttl):
                msg = self.wp_msg
            else:
                self.logger.debug(f'Msg wp is not valid, change to manual')
                msg = self.manual()

        elif self.mode == ModoMision.TELE_OPERADO:  # Teleoperado

            self.logger.debug(f'Modo Teleoperado')
            if self.is_valid(self.tele_msg, self.tele_ttl):
                msg = self.tele_msg
            else:
                self.logger.debug(f'Msg tele is not valid, change to manual')
                msg = self.manual()
        else:
            self.logger.error(f'Error in mode: {self.mode}')
            msg = self.manual()

        msg_final = deepcopy(msg)
        # Se hace la comprobación del master switch
        msg_final.header.stamp = self.get_clock().now().to_msg()
        msg_final.b_brake = msg_final.b_brake and self.master_switch.b_brake
        msg_final.b_throttle = msg_final.b_throttle and self.master_switch.b_throttle
        msg_final.b_steering = msg_final.b_steering and self.master_switch.b_steering
        msg_final.b_gear = msg_final.b_gear and self.master_switch.b_gear
        self.pub_decision.publish(msg_final)

    def publish_heartbeat(self):
        msg = StringStamped(
            data=self.get_name()
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_heartbeat.publish(msg)

    def shutdown(self):
        self.timer_control.cancel()


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = Decision(name='Decision')
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print('Decision: Keyboard interrupt')
        manager.shutdown()
        manager.destroy_node()
    except Exception:
        print(format_exc())


if __name__ == '__main__':
    main()
