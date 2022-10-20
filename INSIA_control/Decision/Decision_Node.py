import os

import rclpy
import yaml
from insia_msg.msg import StringStamped, PetConduccion, BoolStamped, MasterSwitch
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from yaml.loader import SafeLoader
import time


class DecisionNode(Node):
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
        self.default_ttl = 1
        self.lidar_priority = self.get_parameter_or('lidar_priority', Parameter(name='lidar_priority', value=-1)).value
        self.force_use = None
        self.danger_obstacle = False
        self.override = False
        self.last_decision = None
        self.master_switch = MasterSwitch()

        self.pub_heartbeat = self.create_publisher(msg_type=StringStamped,
                                                   topic='Heartbeat',
                                                   qos_profile=HistoryPolicy.KEEP_LAST)

        self.pub_decision = self.create_publisher(msg_type=PetConduccion,
                                                  topic='Decision/Output',
                                                  qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=MasterSwitch,
                                 topic='MasterSwitch',
                                 callback=self.master_switch_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped,
                                 topic='DangerObstacle',
                                 callback=self.darger_obs_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped,
                                 topic='Decision/Override',
                                 callback=self.override_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=BoolStamped,
                                 topic='Decision/ForceLidar',
                                 callback=self.force_use_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        self.create_subscription(msg_type=PetConduccion,
                                 topic='Decision/Output',
                                 callback=self.decision_callback, qos_profile=HistoryPolicy.KEEP_LAST)

        subscribers = self.get_parameters_by_prefix('subscribers')
        print(subscribers)
        self.ttl = {}
        self.create_subscribers(subscribers)
        self.dict_PetConduccion = {}

        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat)
        self.timer_decision = self.create_timer(0.1, self.publish_decision)

    def master_switch_callback(self, data: MasterSwitch):
        if self.master_switch.b_gear != data.b_gear or self.master_switch.b_brake != data.b_brake or self.master_switch.b_steering != data.b_steering or self.master_switch.b_throttle != data.b_throttle:
            self.master_switch = data
            self.logger.debug(
                f'Changed MS: Brake: {data.b_brake} Throttle: {data.b_throttle} Steering: {data.b_steering} Gears: {data.b_gear}')

    def darger_obs_callback(self, data):
        self.danger_obstacle = data.data

    def override_callback(self, data):
        self.override = data.data

    def decision_callback(self, data):
        self.last_decision = data

    def create_subscribers(self, subscribers: dict):
        for sub in subscribers:
            if 'topic' in sub:
                priority = sub.split('.')[0]
                self.create_subscription(msg_type=PetConduccion,
                                         topic='/' + self.id_plataforma + '/' + self.get_name() + subscribers[
                                             sub].value,
                                         callback=lambda pet, priority_l=priority: self.update_PetConduccion(priority_l,
                                                                                                             pet),
                                         qos_profile=HistoryPolicy.KEEP_LAST)
                self.logger.debug(
                    f'Created sub /{self.id_plataforma}/{self.get_name()}{subscribers[sub].value} with priority {sub[0]}')
            elif 'ttl' in sub:
                priority = sub.split('.')[0]
                self.ttl.update({priority: subscribers[sub]})
                self.logger.debug(f'Modified default_ttl for priority {priority}: {subscribers[sub].value} sec')

    def update_PetConduccion(self, priority, pet):
        self.dict_PetConduccion.update({priority: pet})
        self.publish_decision()
        self.timer_decision.reset()

    def is_alive(self, key):
        try:
            msg: PetConduccion = self.dict_PetConduccion.get(key)
            t = msg.header.stamp.sec
            if t != 0:
                ttl = self.ttl.get(key).value if key in self.ttl else self.default_ttl
                t_alive = (time.time() - t)
                if t_alive > ttl:
                    return False
            return True
        except Exception as e:
            self.logger.error(e)

    def publish_decision(self):
        msg_final = None
        if '0' in self.dict_PetConduccion.keys() and self.is_alive('0'):  # Caso de teclado
            msg_final = self.dict_PetConduccion.get('0')
        elif self.danger_obstacle and not self.override:
            msg_final = self.brake_decision(self.last_decision)  # Valores por defecto
        elif self.force_use is not None:  # Caso en el que se solicita forzar el uso de cierta prioridad
            if self.force_use in self.dict_PetConduccion.keys() and self.is_alive(self.force_use):
                msg_final = self.dict_PetConduccion.get(str(self.force_use))
        elif msg_final is None:  # Otro caso
            msg_final = self.brake_decision(self.last_decision)
            for key in sorted(self.dict_PetConduccion.keys()):
                if key != '0':
                    msg: PetConduccion = self.dict_PetConduccion.get(key)
                    if self.is_alive(key):
                        msg_final = msg
                        self.logger.debug(f'Priority {key} will be published')
                        break
                    else:
                        self.dict_PetConduccion.pop(key)
                        self.logger.debug(f'Popped PetConduccion {key}')
        msg_final.header.stamp = self.get_clock().now().to_msg()
        msg_final.b_brake = msg_final.b_brake and self.master_switch.b_brake
        msg_final.b_throttle = msg_final.b_throttle and self.master_switch.b_throttle
        msg_final.b_steering = msg_final.b_steering and self.master_switch.b_steering
        msg_final.b_gear = msg_final.b_gear and self.master_switch.b_gear
        self.pub_decision.publish(msg_final)

    def brake_decision(self, last_decision: PetConduccion):
        if last_decision is None:
            return PetConduccion()
        else:
            last_decision.speed = 0.
            last_decision.steering = 0.
            return last_decision

    def force_use_callback(self, data):
        if self.lidar_priority > 0:
            if data.data:
                self.force_use = 2
            else:
                self.force_use = None

    def publish_heartbeat(self):
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
        manager = DecisionNode()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print(f'{manager.get_name()}: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
