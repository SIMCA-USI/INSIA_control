import rclpy
import numpy as np
from rclpy.node import Node
from can_msgs.msg import Frame
from insia_msg.msg import ControladorFloat


class ThrottleNode(Node):
    def __init__(self):
        super().__init__('throttle_node')

        self.publisher_ = self.create_publisher(Frame, '/CAN/can0/transmit', 10)
        self.create_subscription(ControladorFloat, '/Throttle', self.command_callback, 10)

        self.enabled = False
        self.last_target = 0.0
        self.heartbeat_counter = 0

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Throttle node started')

    def command_callback(self, msg):
        self.enabled = msg.enable
        self.last_target = msg.target

    def timer_callback(self):
        frame = Frame()
        frame.id = 0x0CF00300
        frame.is_extended = True
        frame.dlc = 8

        data = [0xFF] * 8
        if self.enabled:
            target= max(0.0, min(1.0, self.last_target))
            can_value = int(np.interp(target, [0.0, 1.0], [0, 250]))
            data[0] = 0xF0
            data[1] = can_value
        else:
            data[0] = 0xF1
            data[1] = 0x00

        frame.data = data  
        self.publisher_.publish(frame)

        hb_frame = Frame()
        hb_frame.id = 0x0CEF0082
        hb_frame.is_extended = True
        hb_frame.dlc = 8
        hb_data = [0xFF] * 8
        
        hb_values = [0xFC, 0xFD, 0xFE, 0xFF]

        hb_data[0] = hb_values[self.heartbeat_counter]
        hb_frame.data = hb_data
        self.publisher_.publish(hb_frame)
        self.heartbeat_counter = (self.heartbeat_counter + 1) % len(hb_values)


def main(args=None):
    rclpy.init(args=args)
    node = ThrottleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()