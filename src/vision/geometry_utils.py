#!/usr/bin/env python3
"""共享几何工具：旋转矩阵↔四元数、坐标变换"""
import numpy as np


def rotmat_to_quat(R):
    """3x3 旋转矩阵 → [x, y, z, w] 四元数"""
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        return np.array([
            (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s
        ])
    elif m00 > m11 and m00 > m22:
        s = np.sqrt(1.0 + m00 - m11 - m22) * 2
        return np.array([
            0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s
        ])
    elif m11 > m22:
        s = np.sqrt(1.0 + m11 - m00 - m22) * 2
        return np.array([
            (m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s
        ])
    else:
        s = np.sqrt(1.0 + m22 - m00 - m11) * 2
        return np.array([
            (m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s
        ])


def quat_to_rotmat(x, y, z, w):
    """四元数 → 3x3 旋转矩阵"""
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ])


def optical_to_world(ox, oy, oz, cam_x, cam_y, cam_z, z_offset=0.0):
    """camera_left_optical → world 坐标变换"""
    wx = cam_x - oy
    wy = cam_y + ox
    wz = cam_z - oz + z_offset
    return wx, wy, wz
