import os
import struct

import rclpy
import yaml
from insia_msg.msg import CAN, CANGroup
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from yaml.loader import SafeLoader


class test_CAN_Node(Node):
    def __init__(self):
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='CAN_test', namespace=vehicle_parameters['id_vehicle'],
                         start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        self.pub = self.create_publisher(msg_type=CANGroup, topic='/' + vehicle_parameters['id_vehicle'] + '/CAN_7',
                                         qos_profile=HistoryPolicy.KEEP_LAST)
        frame = CAN(
            data_raw=bytearray(struct.pack('<IIIB', 4, 65, 7, 4))
        )
        self.pub.publish(
            CANGroup(
                can_frames=[frame]
            )
        )

    def shutdown(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = test_CAN_Node()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print('CAN: Keyboard interrupt')
    except Exception as e:
        print(e)
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
