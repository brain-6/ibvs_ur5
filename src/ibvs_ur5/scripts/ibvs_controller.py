#!/usr/bin/env python3
"""
IBVS Controller Node (Phase 3: 开环误差验证版)
订阅红方块和绿球的像素坐标，计算图像空间的误差 e = s - s*
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np

class IBVSController(Node):
    def __init__(self):
        super().__init__('ibvs_controller')

        self.target_sub = self.create_subscription(
            Point, '/target_centroid', self.target_callback, 10)

        self.feature_sub = self.create_subscription(
            Point, '/feature_centroid', self.feature_callback, 10)

        self.s_star = np.array([0.0, 0.0], dtype=np.float64)
        self.s = np.array([0.0, 0.0], dtype=np.float64)

        self.has_target = False
        self.has_feature = False

        self.last_target_time = self.get_clock().now()
        self.target_timeout_sec = 0.5

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('IBVS Controller initialized (Open-loop mode).')

    def target_callback(self, msg):
        self.s_star = np.array([msg.x, msg.y], dtype=np.float64)
        self.has_target = True
        self.last_target_time = self.get_clock().now()

    def feature_callback(self, msg):
        self.s = np.array([msg.x, msg.y], dtype=np.float64)
        self.has_feature = True

    def control_loop(self):
        if not (self.has_target and self.has_feature):

            time_since_last_target = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9

            if time_since_last_target > self.target_timeout_sec:
                self.get_logger().warn(f'Target lost for {time_since_last_target:.1f}s! Emergency stop.')
                return

            self.get_logger().info('Waiting for image features...')
            return

        e = self.s - self.s_star
        fx = 554.38
        fy = 554.38
        Z = 2.0

        L_obj = np.array([[fx / Z, 0.0],
                          [0.0, fy / Z]])
        L_obj_inv = np.linalg.inv(L_obj)

        lam = 1.0

        v = -lam * (L_obj_inv @ e)
        max_vel = 0.05
        v = np.clip(v, -max_vel, max_vel)

        if np.linalg.norm(e) < 15.0:
            v = np.array([0.0, 0.0])
            self.get_logger().info('Target reached! Holding position.')

        self.get_logger().info(
            f'Error: e_u={e[0]:6.1f}, e_v={e[1]:6.1f} | '
            f'Cmd Vel: v_x={v[0]:6.3f}, v_y={v[1]:6.3f} m/s'
        )

def main(args=None):
    rclpy.init(args=args)
    node = IBVSController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
