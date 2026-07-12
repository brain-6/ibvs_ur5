#!/usr/bin/env python3
"""
IBVS Controller Node (Phase 3: Closed-loop Control v2)
修复: 坐标系映射 / 增益 / 关节限位 / 速度限幅
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
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

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.current_q = np.zeros(6, dtype=np.float64)
        self.has_joints = False

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)

        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        # 超时熔断
        self.last_target_time = self.get_clock().now()
        self.target_timeout_sec = 0.5

        # 控制参数
        self.lambda_gain = 0.2
        self.max_vel = 0.05
        self.max_q_dot = 0.1
        self.deadzone_px = 15.0

        # 关节硬限位 (防 Z 轴失控 / 翻倒)
        self.q_lower = np.array(
            [-6.28, -2.0, -6.28, -6.28, -6.28, -6.28], dtype=np.float64)
        self.q_upper = np.array(
            [ 6.28,  0.5,  6.28,  6.28,  6.28,  6.28], dtype=np.float64)

        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('IBVS Controller v2 initialized.')

    #  回调
    def target_callback(self, msg):
        self.s_star = np.array([msg.x, msg.y], dtype=np.float64)
        self.has_target = True
        self.last_target_time = self.get_clock().now()

    def feature_callback(self, msg):
        self.s = np.array([msg.x, msg.y], dtype=np.float64)
        self.has_feature = True

    def joint_callback(self, msg):
        q_dict = dict(zip(msg.name, msg.position))
        self.current_q = np.array(
            [q_dict[name] for name in self.joint_names], dtype=np.float64)
        self.has_joints = True

    #  UR5 几何雅可比 (2x6, 仅末端 XY)
    def compute_jacobian_xy(self, q):
        a2, a3 = -0.42500, -0.39225
        d4, d5, d6 = 0.10915, 0.09465, 0.0823
        q1, q2, q3, q4 = q[0], q[1], q[2], q[3]

        s1, c1 = np.sin(q1), np.cos(q1)
        s2, c2 = np.sin(q2), np.cos(q2)
        s23, c23 = np.sin(q2 + q3), np.cos(q2 + q3)
        s234, c234 = np.sin(q2 + q3 + q4), np.cos(q2 + q3 + q4)

        J = np.zeros((2, 6))

        J[0, 0] = -(s1 * (a2*c2 + a3*c23 + d4*s234 + d6*c234) + d5*c1)
        J[1, 0] =  (c1 * (a2*c2 + a3*c23 + d4*s234 + d6*c234) - d5*s1)

        f2 = -a2*s2 - a3*s23 + d4*c234 - d6*s234
        J[0, 1] = c1 * f2
        J[1, 1] = s1 * f2

        f3 = -a3*s23 + d4*c234 - d6*s234
        J[0, 2] = c1 * f3
        J[1, 2] = s1 * f3

        f4 = d4*c234 - d6*s234
        J[0, 3] = c1 * f4
        J[1, 3] = s1 * f4

        return J

    #  主控制循环 10 Hz
    def control_loop(self):
        # 1. 超时熔断
        elapsed = (self.get_clock().now() -
                   self.last_target_time).nanoseconds / 1e9
        if elapsed > self.target_timeout_sec:
            self.get_logger().warn(
                'Target lost! Emergency stop.',
                throttle_duration_sec=1.0)
            return

        if not (self.has_target and self.has_feature):
            self.get_logger().info(
                'Waiting for image features...',
                throttle_duration_sec=2.0)
            return

        if not self.has_joints:
            self.get_logger().info(
                'Waiting for joint states...',
                throttle_duration_sec=2.0)
            return

        # 2. 图像误差
        e = self.s - self.s_star               # [e_u, e_v] 单位 px

        # 3. 死区检查 —— 在死区内不发指令
        err_norm = np.linalg.norm(e)
        if err_norm < self.deadzone_px:
            self.get_logger().info(
                f'Dead zone (err={err_norm:.1f}px). Holding.',
                throttle_duration_sec=2.0)
            return

        # 4. IBVS 控制律: v_cam = -lambda * L^-1 * e
        fx = fy = 554.38
        Z = 2.0
        L = np.array([[fx / Z, 0.0],
                      [0.0,     fy / Z]])
        v_cam = -self.lambda_gain * np.linalg.solve(L, e)

        # 5. 坐标系映射: 相机帧 -> 基座帧
        v_base = np.array([v_cam[1], -v_cam[0]], dtype=np.float64)

        # 6. 笛卡尔速度限幅
        v_base = np.clip(v_base, -self.max_vel, self.max_vel)

        # 7. 速度级 IK: q_dot = J^+ @ v_base
        J_xy = self.compute_jacobian_xy(self.current_q)
        J_pinv = np.linalg.pinv(J_xy, rcond=1e-3)
        q_dot = J_pinv @ v_base

        # 8. 关节速度限幅 (防振荡)
        q_dot = np.clip(q_dot, -self.max_q_dot, self.max_q_dot)

        # 9. 积分
        q_next = self.current_q + q_dot * self.dt

        # 10. 关节硬限位 (防 Z 轴失控)
        q_next = np.clip(q_next, self.q_lower, self.q_upper)

        # 11. 发送 JTC
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = q_next.tolist()
        point.time_from_start = Duration(sec=0, nanosec=110000000)

        traj_msg.points.append(point)
        self.traj_pub.publish(traj_msg)

        # 12. 调试日志
        self.get_logger().info(
            f'err=[{e[0]:7.1f},{e[1]:7.1f}] '
            f'v=[{v_base[0]:+.4f},{v_base[1]:+.4f}] '
            f'qd=[{q_dot[0]:+.4f},{q_dot[1]:+.4f},{q_dot[2]:+.4f},{q_dot[3]:+.4f}]'
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
