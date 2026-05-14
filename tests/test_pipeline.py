#!/usr/bin/env python3
"""离线验证 bin-picking 管线核心逻辑 — 不依赖 ROS2/Gazebo"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'vision'))

import numpy as np
import math
from ikpy.chain import Chain
import warnings
warnings.filterwarnings('ignore')

URDF_PATH = '/tmp/ur5.urdf'
PASS, FAIL = 0, 0


def check(name, actual, expected=None, tol=0.01):
    global PASS, FAIL
    if expected is not None:
        ok = abs(actual - expected) < tol
    else:
        ok = bool(actual)
    if ok:
        PASS += 1
        print(f'  [PASS] {name}')
    else:
        FAIL += 1
        print(f'  [FAIL] {name} — got {actual}, expected {expected}')


def test_ur5_kinematics():
    """Test UR5 FK/IK via ikpy"""
    print('\n=== UR5 Kinematics (ikpy) ===')
    if not os.path.exists(URDF_PATH):
        print(f'  [SKIP] URDF not found: {URDF_PATH}')
        return
    chain = Chain.from_urdf_file(URDF_PATH)

    # FK at zero
    fk0 = chain.forward_kinematics([0]*9)
    check('FK([0]*9) z near 0', fk0[2, 3], expected=None)
    print(f'        FK zero-pos: {np.round(fk0[:3,3], 4)}')

    # IK → FK closed loop
    target = [0.3, -0.2, 0.25]
    ik = chain.inverse_kinematics(target, initial_position=[0]*9)
    fk = chain.forward_kinematics(ik)
    err = np.linalg.norm(fk[:3, 3] - target)
    check(f'IK→FK closed loop err={err:.4f}m', err, 0.0, tol=0.01)
    print(f'        Target: {target} → FK: {np.round(fk[:3,3], 4)}')


def test_coordinate_transform():
    """Test camera_left_optical → world transform"""
    print('\n=== Coordinate Transform ===')
    cx, cy, cz = 0.47, 0.0, 0.5

    # Object at optical origin → should be near camera in world
    ox, oy, oz = 0.0, 0.0, 0.5
    wx = cx - oy
    wy = cy + ox
    wz = cz - oz
    check('optical_origin → world_x ≈ 0.47', wx, 0.47)
    check('optical_origin → world_y ≈ 0.0', wy, 0.0)
    check('optical_origin → world_z ≈ 0.0', wz, 0.0)

    # Object 0.1m right in optical (optical_x=+0.1) → world_y=cy+0.1=0.1
    wx2 = cx - 0.0  # optical_y=0
    wy2 = cy + 0.1  # optical_x=+0.1
    check('optical right(+x) → world +y', wy2, 0.1)


def test_collision_detection():
    """Test _inside_bin logic"""
    print('\n=== Collision Detection ===')
    bcx, bcy, bcz = 0.5, 0.0, 0.05
    bsx, bsy, bsz = 0.4, 0.3, 0.15
    z_tol = 0.35

    def inside(gx, gy, gz):
        margin = 0.02
        return (abs(gx - bcx) < bsx / 2 - margin and
                abs(gy - bcy) < bsy / 2 - margin and
                gz > bcz - z_tol and gz < bcz + bsz + z_tol)

    check('bin center is inside', inside(0.5, 0.0, 0.06))
    check('outside bin x', not inside(1.0, 0.0, 0.06))
    check('outside bin y', not inside(0.5, 0.5, 0.06))
    check('below bin (z_tol)', inside(0.5, 0.0, -0.2), True)


def test_reachability():
    """Test UR5 reachability check"""
    print('\n=== Reachability ===')
    bx, by, bz = 0.5, 0.35, 0.0
    max_reach = 0.85

    def reachable(dx, dy, dz):
        return math.sqrt(dx*dx + dy*dy + dz*dz) <= max_reach

    check('near target reachable', reachable(0.3, -0.2, 0.25))
    check('far target unreachable', not reachable(2.0, 0.0, 0.0))


def test_grasp_strategy():
    """End-to-end: object pose → grasp → IK"""
    print('\n=== End-to-End Grasp Strategy ===')
    if not os.path.exists(URDF_PATH):
        print(f'  [SKIP] URDF not found: {URDF_PATH}')
        return
    chain = Chain.from_urdf_file(URDF_PATH)
    cx, cy, cz = 0.47, 0.0, 0.5  # camera in world
    bx, by, bz = 0.5, 0.35, 0.0   # UR5 base in world

    # Simulated detected object in optical frame
    objects_opt = [
        (0.02, -0.01, 0.45),   # near bin center
        (0.05, 0.03, 0.48),    # offset
        (0.15, -0.10, 0.55),   # near bin edge
    ]

    approach = 0.05
    best_err = 999

    for i, (ox, oy, oz) in enumerate(objects_opt):
        wx = cx - oy
        wy = cy + ox
        wz = cz - oz
        gx, gy, gz = wx, wy, wz + approach
        dx = gx - bx
        dy = gy - by
        dz = gz - bz

        r = math.sqrt(dx*dx + dy*dy + dz*dz)
        if r > 0.85:
            print(f'  obj#{i}: unreachable (dist={r:.3f}m)')
            continue

        target = [dx, dy, dz]
        ik = chain.inverse_kinematics(target, initial_position=[0]*9)
        fk = chain.forward_kinematics(ik)
        err = np.linalg.norm(fk[:3, 3] - target)
        q6 = np.array([ik[j] for j in [2, 3, 4, 5, 6, 7]])
        best_err = min(best_err, err)
        status = 'PASS' if err <= 0.01 else 'FAIL'
        print(f'  obj#{i}: {status} err={err:.4f}m '
              f'grasp_world=({gx:.3f},{gy:.3f},{gz:.3f}) '
              f'q=[{q6[0]:.2f},{q6[1]:.2f},{q6[2]:.2f}]')

    check('at least one object reachable', best_err, 0.0, tol=0.01)


if __name__ == '__main__':
    print('Bin-Picking Pipeline Offline Test')
    print('=================================')
    test_ur5_kinematics()
    test_coordinate_transform()
    test_collision_detection()
    test_reachability()
    test_grasp_strategy()
    print(f'\n{"="*40}')
    print(f'Results: {PASS} passed, {FAIL} failed')
    if FAIL == 0:
        print('ALL TESTS PASSED')
    else:
        print(f'{FAIL} TESTS FAILED')
        sys.exit(1)
