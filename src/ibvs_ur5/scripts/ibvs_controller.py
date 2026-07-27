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
from ibvs_math import (
    feature_position_and_jacobian,
    scale_to_max_abs,
    scale_to_norm,
    damped_pseudoinverse,
)


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

        self.lambda_gain = float(
            self.declare_parameter('lambda_gain', 0.2).value)
        self.estimated_depth = float(
            self.declare_parameter('estimated_depth', 2.0).value)
        self.fx = float(self.declare_parameter('fx', 554.38).value)
        self.fy = float(self.declare_parameter('fy', 554.38).value)
        self.max_cartesian_speed = float(
            self.declare_parameter('max_cartesian_speed', 0.05).value)
        self.max_joint_speed = float(
            self.declare_parameter('max_joint_speed', 0.1).value)
        self.damping = float(
            self.declare_parameter('damping', 0.02).value)
        self.deadzone_px = float(
            self.declare_parameter('deadzone_px', 15.0).value)
        self.feature_timeout_sec = float(
            self.declare_parameter('feature_timeout_sec', 0.5).value)
        self.control_rate = float(
            self.declare_parameter('control_rate', 10.0).value)
        self.trajectory_duration = float(
            self.declare_parameter('trajectory_duration', 0.11).value)

        if self.control_rate <= 0.0:
            raise ValueError('control_rate must be positive')

        if self.damping < 0.0:
            raise ValueError('damping must not be negative')

        self.dt = 1.0 / self.control_rate
        now = self.get_clock().now()
        self.last_target_time = now
        self.last_feature_time = now

        # 关节硬限位 (防 Z 轴失控 / 翻倒)
        self.q_lower = np.array(
            [-6.28, -2.0, -6.28, -6.28, -6.28, -6.28], dtype=np.float64)
        self.q_upper = np.array(
            [ 6.28,  0.5,  6.28,  6.28,  6.28,  6.28], dtype=np.float64)

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
        self.last_feature_time = self.get_clock().now()

    def joint_callback(self, msg):
        q_dict = dict(zip(msg.name, msg.position))
        self.current_q = np.array(
            [q_dict[name] for name in self.joint_names], dtype=np.float64)
        self.has_joints = True

    def features_are_fresh(self):
        now = self.get_clock().now()
        target_age = (now - self.last_target_time).nanoseconds / 1e9
        feature_age = (now - self.last_feature_time).nanoseconds / 1e9
        return (
            self.has_target
            and self.has_feature
            and target_age <= self.feature_timeout_sec
            and feature_age <= self.feature_timeout_sec
        )


    #  主控制循环
    def control_loop(self):
        # 1. 超时熔断
        if not self.features_are_fresh():
            self.get_logger().warn(
                'Image feature stale or missing; holding position.',
                throttle_duration_sec=1.0)
            return

        if not self.has_joints:
            self.get_logger().info(
                'Waiting for joint states...',
                throttle_duration_sec=2.0)
            return

        # 2. 图像误差
        e = self.s - self.s_star

        # 3. 死区检查
        err_norm = np.linalg.norm(e)
        if err_norm < self.deadzone_px:
            self.get_logger().info(
                f'Dead zone (err={err_norm:.1f}px). Holding.',
                throttle_duration_sec=2.0)
            return

        # 4. IBVS 控制律
        L = np.array([
            [self.fx / self.estimated_depth, 0.0],
            [0.0, self.fy / self.estimated_depth],
        ])
        v_cam = -self.lambda_gain * np.linalg.solve(L, e)

        # 5. 坐标系映射: 相机帧 -> 基座帧
        v_base = np.array(
            [v_cam[1], -v_cam[0], 0.0], dtype=np.float64)

        # 6. 笛卡尔限速
        v_base = scale_to_norm(
            v_base, self.max_cartesian_speed)

        # 7. 速度级 IK
        _, jacobian = feature_position_and_jacobian(self.current_q)
        jacobian_pinv = damped_pseudoinverse(
            jacobian, damping=self.damping)
        q_dot = jacobian_pinv @ v_base

        # 8. 关节限速
        q_dot = scale_to_max_abs(
            q_dot, self.max_joint_speed)

        # 9. 积分
        q_next = self.current_q + q_dot * self.dt

        # 10. 关节硬限位 (防 Z 轴失控)
        q_next = np.clip(q_next, self.q_lower, self.q_upper)

        # 11. 发送 JTC
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        duration_ns = max(
            1, int(round(self.trajectory_duration * 1e9)))

        point = JointTrajectoryPoint()
        point.positions = q_next.tolist()
        point.time_from_start = Duration(
            sec=duration_ns // 1_000_000_000,
            nanosec=duration_ns % 1_000_000_000)

        traj_msg.points.append(point)
        self.traj_pub.publish(traj_msg)

        # 12. 调试日志
        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        achieved_velocity = jacobian @ q_dot
        self.get_logger().info(
            f'err=[{e[0]:7.1f},{e[1]:7.1f}] '
            f'v_cmd=[{v_base[0]:+.4f},'
            f'{v_base[1]:+.4f},{v_base[2]:+.4f}] '
            f'v_act=[{achieved_velocity[0]:+.4f},'
            f'{achieved_velocity[1]:+.4f},{achieved_velocity[2]:+.4f}] '
            f'qd_max={np.max(np.abs(q_dot)):.3f} '
            f'sigma_min={singular_values[-1]:.4f}')


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
