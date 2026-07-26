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
    """Limit a vector norm without changing its direction."""
    vector = np.asarray(vector, dtype=np.float64)
    if max_norm <= 0.0:
        raise ValueError('max_norm must be positive')

    norm = float(np.linalg.norm(vector))
    if norm <= max_norm or norm == 0.0:
        return vector.copy()
    return vector * (max_norm / norm)


def scale_to_max_abs(vector, max_abs):
    """Limit all components by one scale factor, preserving direction."""
    vector = np.asarray(vector, dtype=np.float64)
    if max_abs <= 0.0:
        raise ValueError('max_abs must be positive')

    peak = float(np.max(np.abs(vector)))
    if peak <= max_abs or peak == 0.0:
        return vector.copy()
    return vector * (max_abs / peak)
