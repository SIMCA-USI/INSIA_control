import os
import time
from copy import deepcopy
from traceback import format_exc

import rclpy
import yaml
from std_msgs.msg import Bool
from insia_msg.msg import StringStamped, PetConduccion, MasterSwitch, ModoMision, Override, BoolStamped, Telemetry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy import time as t_rclpy
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Header
from yaml.loader import SafeLoader


class Decision(Node):

    def parameters_callback(self, params):
        for param in params:
            if param.name == "log_level":
                self.logger.set_level(param.value)
        return SetParametersResult(successful=True)

    def __init__(self):
        """
        Decision node that receives
        MasterSwitch: To invalidate orders from high level
        PathPlanning: Targets from wp/lidar
        TeleOperacion: Targets from keyboard/Controller/tablet
        Mode: To select operation mode
        """
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
        self.steering_wheel_conversion = vehicle_parameters['steering']['steering_wheel_conversion']
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Systems enabled by default
        self.master_switch = MasterSwitch(b_steering=True, b_throttle=True, b_brake=True, b_gear=True)
        self.tele_msg = None
        self.wp_msg = None
        self.emergency_stop_msg = False
        self.override = Override()
        self.telemetry = Telemetry()
        # self.emergency_stop_msg = False
        # self.wp_ttl = self.get_parameter_or('wp_ttl', Parameter(name='wp_ttl', value=1))

        self.wp_ttl, self.wp_mode = self.get_p(self.get_parameters_by_prefix('wp'))
        self.tele_ttl, self.tele_mode = self.get_p(self.get_parameters_by_prefix('tele'))

        # Manual mode by default, TODO: When everything will be working perfectly default teleoperation
        self.mode = ModoMision.MANUAL

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped, topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_decision = self.create_publisher(msg_type=PetConduccion, topic='Decision/Output',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped, topic='EmergencyStop', callback=self.emergency_stop_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=MasterSwitch, topic='MasterSwitch', callback=self.master_switch_callback,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=Override, topic='Override',
                                 callback=self.override_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=PetConduccion, topic='PathPlanning',
                                 callback=self.pathplanning_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=PetConduccion, topic='TeleOperacion',
                                 callback=self.teleoperation_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=ModoMision, topic='Mode',
                                 callback=self.modo_mision_callback, qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=Telemetry, topic='Telemetry', callback=self.callback_telemetry,
                                 qos_profile=HistoryPolicy.KEEP_LAST)

        self.timer_control = self.create_timer(1 / 10, self.decision)
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)

    def callback_telemetry(self, data: Telemetry):
        self.telemetry = data

    def get_p(self, d_params: dict):
        """
        Function to get parameters
        :param d_params: Dictionary of parameters
        :type d_params: dict
        :return: ttl and steering data mode of this param
        :rtype: (float, str)
        """
        if 'ttl' in d_params.keys():
            ttl = d_params['ttl'].value
        else:
            ttl = 1  # Default ttl 1s
        if 'mode' in d_params.keys():
            mode = d_params['mode'].value
            if mode not in ['wheels', 'steering_wheel']:
                self.logger.error(f'Error getting params , not valid: {mode}')
                mode = 'wheels'
        else:
            mode = 'wheels'
        return ttl, mode

    def emergency_stop_callback(self, data):
        self.emergency_stop_msg = data.data

    def override_callback(self, data):
        self.override = data

    def emergency_stop_callback(self, data: BoolStamped):
        self.emergency_stop_msg = data.data

    def master_switch_callback(self, data: MasterSwitch):
        if self.master_switch.b_gear != data.b_gear or self.master_switch.b_brake != data.b_brake or \
                self.master_switch.b_steering != data.b_steering or self.master_switch.b_throttle != data.b_throttle:
            self.master_switch = data
            self.logger.debug(
                f'Changed MS: Brake: {data.b_brake} Throttle: {data.b_throttle} Steering: {data.b_steering} Gears: {data.b_gear}')

    def modo_mision_callback(self, data: ModoMision):
        self.logger.debug(f'Modo {data.modo_mision}')
        self.mode = data.modo_mision

    def teleoperation_callback(self, data: PetConduccion):
        if self.tele_mode == 'wheels':
            data.steering *= self.steering_wheel_conversion
        self.tele_msg = data
        if self.mode == ModoMision.TELE_OPERADO:
            self.timer_control.reset()
            self.decision()

    def pathplanning_callback(self, data: PetConduccion):
        if self.wp_mode == 'wheels':
            data.steering *= self.steering_wheel_conversion
        self.wp_msg = data
        if self.mode == ModoMision.AUTONOMO:
            self.timer_control.reset()
            self.decision()

    def manual(self) -> PetConduccion:
        """
        Function to create PetConduccion msg
        :return: PetConduccion
        :rtype: PetConduccion
        """
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
        """
        Function to check if msg is valid depending of it's ttl and it's header
        :param msg: PetConduccion msg
        :type msg: PetConduccion
        :param ttl: Time To Live of a msg
        :type ttl: float
        :return: If msg is valid or not
        :rtype: bool
        """
        if msg is not None:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 10 ** -9
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
        """
        Decision function to select correct msg and publish it
        :return: Pub on Decision Result
        """
        if self.mode == ModoMision.MANUAL:  # Manual

            self.logger.debug(f'Modo manual')
            msg = self.manual()

        elif self.mode == ModoMision.AUTONOMO:  # Waypoints

            self.logger.debug(f'Modo Waypoints')
            if self.is_valid(self.wp_msg, self.wp_ttl):
                msg = self.wp_msg
                if self.override.b_steering:
                    self.logger.info(
                        f'Overriding steering from {msg.steering:.2f} to {self.override.steering:.2f}')
                    msg.steering = self.override.steering
                if self.override.b_speed and msg.speed > 5:
                    self.logger.info(f'Overriding speed from {msg.speed:.2f} to {self.override.speed:.2f}')
                    msg.speed = self.override.speed
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
        # MasterSwitch override
        msg_final.header.stamp = self.get_clock().now().to_msg()
        msg_final.b_brake = msg_final.b_brake and self.master_switch.b_brake
        msg_final.b_throttle = msg_final.b_throttle and self.master_switch.b_throttle
        msg_final.b_steering = msg_final.b_steering and self.master_switch.b_steering
        msg_final.b_gear = msg_final.b_gear and self.master_switch.b_gear
        if self.emergency_stop_msg:
            msg_final.speed = 0.
        self.pub_decision.publish(msg_final)

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
        self.timer_control.cancel()


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = Decision()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print('Decision: Keyboard interrupt')
        manager.shutdown()
        manager.destroy_node()
    except Exception:
        print(format_exc())


if __name__ == '__main__':
    main()
