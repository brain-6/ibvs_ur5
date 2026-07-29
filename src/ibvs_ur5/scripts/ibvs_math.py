#!/usr/bin/env python3
"""Pure NumPy kinematics helpers for the UR5 IBVS controller."""

import numpy as np


UR5_A = np.array([0.0, -0.42500, -0.39225, 0.0, 0.0, 0.0])
UR5_D = np.array([0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823])
UR5_ALPHA = np.array([
    np.pi / 2.0, 0.0, 0.0, np.pi / 2.0, -np.pi / 2.0, 0.0,
])

ROS_BASE_FROM_DH_BASE = np.diag([-1.0, -1.0, 1.0])


def dh_transform(theta, d, a, alpha):
    #返回标准的D-H变换
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=np.float64)

def feature_position_and_jacobian(q, tool_offset=0.05):
    #返回特征位置及3x6平移雅可比矩阵
    q = np.asarray(q, dtype=np.float64)
    if q.shape != (6,):
        raise ValueError('q must contain exactly six joint angles')

    transform = np.eye(4, dtype=np.float64)
    joint_origins = []
    joint_axes = []

    for index in range(6):
        joint_origins.append(transform[:3, 3].copy())
        joint_axes.append(transform[:3, 2].copy())
        transform = transform @ dh_transform(
            q[index], UR5_D[index], UR5_A[index], UR5_ALPHA[index])

    feature_in_tool = np.array(
        [0.0, 0.0, tool_offset, 1.0], dtype=np.float64)
    feature_position_dh = (transform @ feature_in_tool)[:3]

    jacobian_dh = np.empty((3, 6), dtype=np.float64)
    for index, (origin, axis) in enumerate(zip(joint_origins, joint_axes)):
        jacobian_dh[:, index] = np.cross(
            axis, feature_position_dh - origin)

    feature_position = ROS_BASE_FROM_DH_BASE @ feature_position_dh
    jacobian = ROS_BASE_FROM_DH_BASE @ jacobian_dh
    return feature_position, jacobian

def scale_to_norm(vector, max_norm):
    #限制向量模，方向不变
    vector = np.asarray(vector, dtype=np.float64)
    if max_norm <= 0.0:
        raise ValueError('max_norm must be positive')

    norm = float(np.linalg.norm(vector))
    if norm <= max_norm or norm == 0.0:
        return vector.copy()
    return vector * (max_norm / norm)


def scale_to_max_abs(vector, max_abs):
    #等比缩放限制分量
    vector = np.asarray(vector, dtype=np.float64)
    if max_abs <= 0.0:
        raise ValueError('max_abs must be positive')

    peak = float(np.max(np.abs(vector)))
    if peak <= max_abs or peak == 0.0:
        return vector.copy()
    return vector * (max_abs / peak)

def image_error_to_base_velocity(error, gain, depth, fx, fy, max_speed):
    #固定相机：像素误差映射基座XYZ 速度
    error = np.asarray(error, dtype=np.float64)
    if error.shape != (2,):
        raise ValueError('error must contain [e_u, e_v]')
    if gain <= 0.0 or depth <= 0.0 or fx <= 0.0 or fy <= 0.0:
        raise ValueError('gain, depth, fx, and fy must be positive')

    image_plane_velocity = -gain * np.array([
        depth * error[0] / fx,
        depth * error[1] / fy,
    ], dtype=np.float64)

    base_velocity = np.array([
        -image_plane_velocity[1],
        -image_plane_velocity[0],
        0.0,
    ], dtype=np.float64)
    return scale_to_norm(base_velocity, max_speed)

def damped_pseudoinverse(jacobian, damping):
    #返回J^T (J J^T + 阻尼^2 I)^-1
    jacobian = np.asarray(jacobian, dtype=np.float64)
    if jacobian.ndim != 2:
        raise ValueError('jacobian must be a matrix')
    if damping < 0.0:
        raise ValueError('damping must not be negative')

    regularized = (
        jacobian @ jacobian.T
        + (damping ** 2) * np.eye(jacobian.shape[0], dtype=np.float64)
    )
    return jacobian.T @ np.linalg.solve(
        regularized, np.eye(jacobian.shape[0], dtype=np.float64))


def damped_least_squares(jacobian, target_velocity, damping):
    #求解阻尼速度级逆运动学
    jacobian = np.asarray(jacobian, dtype=np.float64)
    target_velocity = np.asarray(target_velocity, dtype=np.float64)
    if jacobian.ndim != 2:
        raise ValueError('jacobian must be a matrix')
    if target_velocity.shape != (jacobian.shape[0],):
        raise ValueError('target_velocity size must match jacobian rows')
    return damped_pseudoinverse(jacobian, damping) @ target_velocity

def shortest_angular_difference(target, current):
    #最短角度误差
    target = np.asarray(target, dtype=np.float64)
    current = np.asarray(current, dtype=np.float64)
    if target.shape != current.shape:
        raise ValueError('target and current must have the same shape')
    return (target - current + np.pi) % (2.0 * np.pi) - np.pi

def combine_prioritized_velocities(primary, secondary, max_abs):
    #保留主速度，次速度拟合至剩余限制
    primary = np.asarray(primary, dtype=np.float64)
    secondary = np.asarray(secondary, dtype=np.float64)

    if primary.ndim != 1 or primary.shape != secondary.shape:
        raise ValueError('primary and secondary must have the same vector shape')
    if max_abs <= 0.0:
        raise ValueError('max_abs must be positive')

    primary_limited = scale_to_max_abs(primary, max_abs)
    secondary_scale = 1.0

    for primary_value, secondary_value in zip(
            primary_limited, secondary):
        if secondary_value > 0.0:
            bound = (
                max_abs - primary_value) / secondary_value
        elif secondary_value < 0.0:
            bound = (
                -max_abs - primary_value) / secondary_value
        else:
            continue

        secondary_scale = min(secondary_scale, bound)

    secondary_scale = float(np.clip(secondary_scale, 0.0, 1.0))
    combined = primary_limited + secondary_scale * secondary
    return combined, secondary_scale
