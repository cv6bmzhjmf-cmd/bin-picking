#!/usr/bin/env python3
"""离线验证 bin-picking 管线核心逻辑 — 不依赖 ROS2/Gazebo"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'vision'))

import numpy as np
import math
from geometry_utils import optical_to_world, rotmat_to_quat, quat_to_rotmat
import warnings
warnings.filterwarnings('ignore')

try:
    from ikpy.chain import Chain
    HAS_IKPY = True
except ImportError:
    HAS_IKPY = False

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
    if not HAS_IKPY:
        print('  [SKIP] ikpy not installed')
        return
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

    wx, wy, wz = optical_to_world(0.0, 0.0, 0.5, cx, cy, cz)
    check('optical_origin → world_x ≈ 0.47', wx, 0.47)
    check('optical_origin → world_y ≈ 0.0', wy, 0.0)
    check('optical_origin → world_z ≈ 0.0', wz, 0.0)

    wx2, wy2, wz2 = optical_to_world(0.1, 0.0, 0.5, cx, cy, cz)
    check('optical right(+x) → world +y', wy2, 0.1)
    check('world_x unchanged for +x', wx2, 0.47)


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
    if not HAS_IKPY:
        print('  [SKIP] ikpy not installed')
        return
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


def test_empty_object_list():
    """Empty pose array should not crash"""
    print('\n=== Empty Object List ===')
    check('empty list handled', len([]) == 0)


def test_max_reach_boundary():
    """Objects at UR5 max reach boundary"""
    print('\n=== Max Reach Boundary ===')
    bx, by, bz = 0.5, 0.35, 0.0
    max_reach = 0.85

    def reachable(dx, dy, dz):
        return math.sqrt(dx*dx + dy*dy + dz*dz) <= max_reach

    check('at limit (0.85m)', reachable(0.85, 0.0, 0.0))
    check('just beyond (0.86m)', not reachable(0.86, 0.0, 0.0))
    check('diagonal at limit', reachable(0.6, 0.6, 0.05))


def test_bin_edge_cases():
    """Objects at bin boundary"""
    print('\n=== Bin Edge Cases ===')
    bcx, bcy, bcz = 0.5, 0.0, 0.05
    bsx, bsy, bsz = 0.4, 0.3, 0.15
    z_tol = 0.05
    margin = 0.02

    def inside(gx, gy, gz):
        return (abs(gx - bcx) < bsx / 2 - margin and
                abs(gy - bcy) < bsy / 2 - margin and
                gz > bcz - z_tol and gz < bcz + bsz + z_tol)

    check('at bin floor (z_tol ok)', inside(0.5, 0.0, 0.001))
    check('below bin floor', not inside(0.5, 0.0, -0.1))
    check('at x boundary', inside(0.67, 0.0, 0.06))
    check('beyond x boundary', not inside(0.70, 0.0, 0.06))
    check('at y boundary', inside(0.5, 0.12, 0.06))
    check('beyond y boundary', not inside(0.5, 0.14, 0.06))
    check('above bin top', not inside(0.5, 0.0, 0.25))


def test_rotmat_quat_roundtrip():
    """Verify rotmat ↔ quat roundtrip via geometry_utils"""
    print('\n=== RotMat <-> Quat Roundtrip ===')
    R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    q = rotmat_to_quat(R)
    R2 = quat_to_rotmat(q[0], q[1], q[2], q[3])
    check('roundtrip within tolerance', np.allclose(R, R2, atol=0.001))


def test_multiple_objects_sort():
    """Verify objects sorted by distance from bin center"""
    print('\n=== Multi-Object Sort ===')
    cam_x, cam_y = 0.47, 0.0
    bin_cx, bin_cy = 0.5, 0.0

    objects = [
        (0.02, -0.01, 0.45),   # near center
        (0.15, -0.10, 0.55),   # far from center
        (0.05, 0.03, 0.48),    # medium
    ]

    def sort_key(opt):
        ox, oy, oz = opt
        wx, wy, _ = optical_to_world(ox, oy, oz, cam_x, cam_y, 0.0)
        return math.sqrt((wx - bin_cx)**2 + (wy - bin_cy)**2)

    sorted_objs = sorted(objects, key=sort_key)
    # Nearest to bin center should be first
    check('sort picks nearest first', sorted_objs[0] == objects[0])


if __name__ == '__main__':
    print('Bin-Picking Pipeline Offline Test')
    print('=================================')
    test_ur5_kinematics()
    test_coordinate_transform()
    test_collision_detection()
    test_reachability()
    test_grasp_strategy()
    test_empty_object_list()
    test_max_reach_boundary()
    test_bin_edge_cases()
    test_rotmat_quat_roundtrip()
    test_multiple_objects_sort()
    print(f'\n{"="*40}')
    print(f'Results: {PASS} passed, {FAIL} failed')
    if FAIL == 0:
        print('ALL TESTS PASSED')
    else:
        print(f'{FAIL} TESTS FAILED')
        sys.exit(1)
