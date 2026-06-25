#!/usr/bin/env python3
"""Subscribe to /joint_states and print joint angles."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePrinter(Node):
    def __init__(self):
        super().__init__('joint_state_printer')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10)

    def callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.get_logger().info(f'{name}: {pos:.2f} rad')

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
