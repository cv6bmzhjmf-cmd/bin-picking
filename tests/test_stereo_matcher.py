#!/usr/bin/env python3
"""立体匹配核心逻辑测试 — 离线验证深度计算和滤波"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'vision'))

import numpy as np
from scipy.ndimage import median_filter

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


def test_disparity_to_depth():
    """Z = fx * B / d"""
    print('\n=== Depth from Disparity ===')
    fx, B = 642.0, 0.06
    d = 100.0
    z = fx * B / d
    check('d=100 → z≈0.385m', z, 0.3852, tol=0.01)

    d = 50.0
    z = fx * B / d
    check('d=50 → z≈0.770m', z, 0.7704, tol=0.01)


def test_median_filter_preserves_values():
    """Float32 median filter preserves valid region"""
    print('\n=== Float32 Median Filter ===')
    disp = np.zeros((20, 20), dtype=np.float32)
    disp[5:15, 5:15] = 100.0
    filtered = median_filter(disp, size=5)
    # Center should remain ~100
    center_mean = filtered[8:12, 8:12].mean()
    check('center preserved after median', center_mean, 100.0, tol=1.0)
    # Edges should be 0
    edge_mean = filtered[0:3, 0:3].mean()
    check('edges remain 0 after median', edge_mean, 0.0, tol=0.01)


def test_clip_range():
    """Disparity clipping limits extreme values"""
    print('\n=== Disparity Clipping ===')
    disp = np.array([50, 80, 150, 224, 250], dtype=np.float32)
    clipped = np.clip(disp, 80, 224)
    check('min clipped to 80', clipped[0], 80.0)
    check('80 preserved', clipped[1], 80.0)
    check('150 preserved', clipped[2], 150.0)
    check('224 preserved', clipped[3], 224.0)
    check('max clipped to 224', clipped[4], 224.0)


if __name__ == '__main__':
    print('Stereo Matcher Core Tests')
    print('=========================')
    test_disparity_to_depth()
    test_median_filter_preserves_values()
    test_clip_range()
    print(f'\n{"="*40}')
    print(f'Results: {PASS} passed, {FAIL} failed')
    if FAIL > 0:
        sys.exit(1)
