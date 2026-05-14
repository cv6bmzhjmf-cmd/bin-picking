#!/usr/bin/env python3
"""几何工具单元测试"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'vision'))

import numpy as np
from geometry_utils import rotmat_to_quat, quat_to_rotmat, optical_to_world

PASS, FAIL = 0, 0


def check(name, actual, expected=None, tol=0.01):
    global PASS, FAIL
    if expected is not None:
        ok = abs(actual - expected) < tol
    elif isinstance(actual, bool):
        ok = actual
    else:
        ok = bool(actual)
    if ok:
        PASS += 1
        print(f'  [PASS] {name}')
    else:
        FAIL += 1
        print(f'  [FAIL] {name} — got {actual}, expected {expected}')


def test_rotmat_to_quat_identity():
    print('\n=== rotmat_to_quat: Identity ===')
    R = np.eye(3)
    q = rotmat_to_quat(R)
    check('w ≈ 1.0', abs(q[3] - 1.0), 0.0, tol=0.001)
    check('x ≈ 0', abs(q[0]), 0.0, tol=0.001)
    check('y ≈ 0', abs(q[1]), 0.0, tol=0.001)
    check('z ≈ 0', abs(q[2]), 0.0, tol=0.001)


def test_rotmat_to_quat_90deg_z():
    print('\n=== rotmat_to_quat: 90° around Z ===')
    theta = np.pi / 2
    R = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0, 0, 1],
    ])
    q = rotmat_to_quat(R)
    check('w ≈ 0.707', q[3], np.cos(np.pi/4), tol=0.01)
    check('z ≈ 0.707', q[2], np.sin(np.pi/4), tol=0.01)


def test_quat_to_rotmat_identity():
    print('\n=== quat_to_rotmat: Identity ===')
    R = quat_to_rotmat(0, 0, 0, 1)
    check('is identity', np.allclose(R, np.eye(3), atol=0.001))


def test_quat_to_rotmat_roundtrip():
    print('\n=== quat <-> rotmat roundtrip ===')
    R_orig = np.array([
        [0, -1, 0],
        [1, 0, 0],
        [0, 0, 1],
    ])
    q = rotmat_to_quat(R_orig)
    R_back = quat_to_rotmat(q[0], q[1], q[2], q[3])
    check('roundtrip matches', np.allclose(R_orig, R_back, atol=0.001))


def test_optical_to_world_origin():
    print('\n=== optical_to_world: origin ===')
    cam = (0.47, 0.0, 0.5)
    wx, wy, wz = optical_to_world(0.0, 0.0, 0.5, *cam)
    check('wx ≈ 0.47', wx, 0.47)
    check('wy ≈ 0.0', wy, 0.0)
    check('wz ≈ 0.0', wz, 0.0)


def test_optical_to_world_with_z_offset():
    print('\n=== optical_to_world: with z_offset ===')
    cam = (0.47, 0.0, 0.5)
    wx, wy, wz = optical_to_world(0.0, 0.0, 0.45, *cam, z_offset=0.33)
    check('wz with offset', wz, 0.38)


if __name__ == '__main__':
    print('Geometry Utils Tests')
    print('====================')
    test_rotmat_to_quat_identity()
    test_rotmat_to_quat_90deg_z()
    test_quat_to_rotmat_identity()
    test_quat_to_rotmat_roundtrip()
    test_optical_to_world_origin()
    test_optical_to_world_with_z_offset()
    print(f'\n{"="*40}')
    print(f'Results: {PASS} passed, {FAIL} failed')
    if FAIL > 0:
        sys.exit(1)
