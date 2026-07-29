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
    damped_pseudoinverse,
    feature_position_and_jacobian,
    image_error_to_base_velocity,
    combine_prioritized_velocities,
    shortest_angular_difference,
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
        self.posture_gain = float(
            self.declare_parameter('posture_gain', 0.25).value)
        self.preferred_q = np.asarray(
            self.declare_parameter(
                'preferred_q',
                [0.0, -1.2, 1.2, -1.57, -1.57, 0.0]).value,
            dtype=np.float64)
        self.deadzone_px = float(
            self.declare_parameter('deadzone_px', 15.0).value)
        self.feature_timeout_sec = float(
            self.declare_parameter('feature_timeout_sec', 0.5).value)
        self.control_rate = float(
            self.declare_parameter('control_rate', 10.0).value)
        self.trajectory_duration = float(
            self.declare_parameter('trajectory_duration', 0.11).value)
        self.tool_offset = float(
            self.declare_parameter('tool_offset', 0.05).value)

        if self.control_rate <= 0.0:
            raise ValueError('control_rate must be positive')

        if self.damping < 0.0:
            raise ValueError('damping must not be negative')
        if self.posture_gain < 0.0:
            raise ValueError('posture_gain must not be negative')
        if self.preferred_q.shape != (6,):
            raise ValueError('preferred_q must contain six joint angles')

        self.dt = 1.0 / self.control_rate
        now = self.get_clock().now()
        self.last_target_time = now
        self.last_feature_time = now

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            'IBVS controller initialized: '
            f'{self.control_rate:.0f} Hz, '
            f'lambda={self.lambda_gain:.2f}, '
            f'v_max={self.max_cartesian_speed:.2f} m/s, '
            f'qd_max={self.max_joint_speed:.2f} rad/s, '
            f'damping={self.damping:.3f}, '
            f'posture_gain={self.posture_gain:.2f}, '
            f'tool_offset={self.tool_offset:.3f} m')

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
        q_by_name = dict(zip(msg.name, msg.position))
        missing = [
            name for name in self.joint_names if name not in q_by_name
        ]
        if missing:
            self.get_logger().warn(
                f'JointState is missing: {", ".join(missing)}',
                throttle_duration_sec=2.0)
            return

        self.current_q = np.array(
            [q_by_name[name] for name in self.joint_names],
            dtype=np.float64)
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
        # 超时熔断
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

        # 图像误差
        e = self.s - self.s_star

        # 死区检查
        err_norm = np.linalg.norm(e)
        if err_norm < self.deadzone_px:
            self.get_logger().info(
                f'Dead zone (err={err_norm:.1f}px). Holding.',
                throttle_duration_sec=2.0)
            return

        # IBVS控制律
        desired_velocity = image_error_to_base_velocity(
            e,
            gain=self.lambda_gain,
            depth=self.estimated_depth,
            fx=self.fx,
            fy=self.fy,
            max_speed=self.max_cartesian_speed)


        # 速度级 IK
        _, jacobian = feature_position_and_jacobian(
            self.current_q, tool_offset=self.tool_offset)
        jacobian_pinv = damped_pseudoinverse(
            jacobian, damping=self.damping)
        q_dot_task = jacobian_pinv @ desired_velocity

        posture_error = shortest_angular_difference(
            self.preferred_q, self.current_q)
        nullspace = np.eye(6) - jacobian_pinv @ jacobian
        q_dot_posture = nullspace @ (
            self.posture_gain * posture_error)

        # 保留视觉任务，余速用于姿态
        q_dot, posture_scale = combine_prioritized_velocities(
            q_dot_task,
            q_dot_posture,
            self.max_joint_speed)

        # 积分
        q_next = self.current_q + q_dot * self.dt

        # 发送 JTC
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

        # 调试日志
        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        achieved_velocity = jacobian @ q_dot
        self.get_logger().info(
            f'err=[{e[0]:7.1f},{e[1]:7.1f}] '
            f'v_cmd=[{desired_velocity[0]:+.4f},'
            f'{desired_velocity[1]:+.4f},'
            f'{desired_velocity[2]:+.4f}] '
            f'v_act=[{achieved_velocity[0]:+.4f},'
            f'{achieved_velocity[1]:+.4f},{achieved_velocity[2]:+.4f}] '
            f'qd_task={np.max(np.abs(q_dot_task)):.3f} '
            f'qd_max={np.max(np.abs(q_dot)):.3f} '
            f'qd_null={np.linalg.norm(q_dot_posture):.3f} '
            f'posture_scale={posture_scale:.3f} '
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
