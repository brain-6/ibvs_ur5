#!/usr/bin/env python3
"""
IBVS Controller Node (开环误差验证 + 速度级 IK)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np

class IBVSController(Node):
    def __init__(self):
        super().__init__('ibvs_controller')

        # 订阅视觉特征
        self.target_sub = self.create_subscription(Point, '/target_centroid', self.target_callback, 10)
        self.feature_sub = self.create_subscription(Point, '/feature_centroid', self.feature_callback, 10)

        self.s_star = np.array([0.0, 0.0], dtype=np.float64)
        self.s = np.array([0.0, 0.0], dtype=np.float64)
        self.has_target = False
        self.has_feature = False

        # 订阅关节状态
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.current_q = np.zeros(6, dtype=np.float64)
        self.has_joints = False

        # 超时熔断机制
        self.last_target_time = self.get_clock().now()
        self.target_timeout_sec = 0.5

        # 定时器 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('IBVS Controller initialized.')

    def target_callback(self, msg):
        self.s_star = np.array([msg.x, msg.y], dtype=np.float64)
        self.has_target = True
        self.last_target_time = self.get_clock().now()

    def feature_callback(self, msg):
        self.s = np.array([msg.x, msg.y], dtype=np.float64)
        self.has_feature = True

    def joint_callback(self, msg):
        joint_order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        q_dict = dict(zip(msg.name, msg.position))
        self.current_q = np.array([q_dict[name] for name in joint_order], dtype=np.float64)
        self.has_joints = True

    def compute_jacobian_xy(self, q):
        a2, a3 = -0.42500, -0.39225
        d4, d5, d6 = 0.10915, 0.09465, 0.0823
        q1, q2, q3, q4 = q[0], q[1], q[2], q[3]

        s1, c1 = np.sin(q1), np.cos(q1)
        s2, c2 = np.sin(q2), np.cos(q2)
        s23, c23 = np.sin(q2+q3), np.cos(q2+q3)
        s234, c234 = np.sin(q2+q3+q4), np.cos(q2+q3+q4)

        # 正向运动学 (仅 X, Y)
        px = c1*(a2*c2 + a3*c23 + d4*s234 + d6*c234) - d5*s1
        py = s1*(a2*c2 + a3*c23 + d4*s234 + d6*c234) + d5*c1

        J = np.zeros((2, 6))
        J[0, 0] = -py
        J[1, 0] = px
        J[0, 1] = -s1*(a2*s2 + a3*s23 - d4*c234 + d6*s234)
        J[1, 1] =  c1*(a2*s2 + a3*s23 - d4*c234 + d6*s234)
        J[0, 2] = -s1*(a3*s23 - d4*c234 + d6*s234)
        J[1, 2] =  c1*(a3*s23 - d4*c234 + d6*s234)
        J[0, 3] = -s1*(-d4*c234 + d6*s234)
        J[1, 3] =  c1*(-d4*c234 + d6*s234)
        # J 的第 4, 5 列对 XY 平移影响极小，近似为 0
        return J

    def control_loop(self):
        # 超时熔断检查
        time_since_last_target = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
        if time_since_last_target > self.target_timeout_sec:
            self.get_logger().warn(f'Target lost for {time_since_last_target:.1f}s! Emergency stop.')
            return

        if not (self.has_target and self.has_feature):
            self.get_logger().info('Waiting for image features...')
            return

        e = self.s - self.s_star

        fx = fy = 554.38
        Z = 2.0
        L_obj = np.array([[fx/Z, 0.0], [0.0, fy/Z]])
        L_obj_inv = np.linalg.inv(L_obj)
        lam = 1.0
        v = -lam * (L_obj_inv @ e)

        # 安全锁：速度限幅与死区
        max_vel = 0.05
        v = np.clip(v, -max_vel, max_vel)
        if np.linalg.norm(e) < 15.0:
            v = np.array([0.0, 0.0])
            self.get_logger().info('Target reached! Holding position.')

        # Velocity IK
        if self.has_joints:
            J_xy = self.compute_jacobian_xy(self.current_q)
            # rcond 用于过滤极小奇异值，防止机械臂在奇异位形附近速度爆炸
            J_xy_pinv = np.linalg.pinv(J_xy, rcond=1e-3)
            q_dot = J_xy_pinv @ v

            self.get_logger().info(
                f'Error: e_u={e[0]:6.1f}, e_v={e[1]:6.1f} | '
                f'Cmd Vel: v_x={v[0]:6.3f}, v_y={v[1]:6.3f} | '
                f'Joint Vel: q1={q_dot[0]:6.3f}, q2={q_dot[1]:6.3f}, q3={q_dot[2]:6.3f}'
            )
        else:
            self.get_logger().info('Waiting for joint states...')

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
