#!/usr/bin/env python3
"""
独立演示：UR5 从 [0,0,0,0,0,0] 平滑移动到 [0,0,0,0,pi/2,0]
纯 set_model_configuration + cubic spline 插值，不依赖 ros2_control
用法: python3 demo_trajectory.py
"""
import math
import time
import sys
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelConfiguration

_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
_MODEL = 'ur5'


def shortest_angle_diff(a, b):
    """a→b 的最短角度差，结果在 [-pi, +pi)"""
    return (b - a + math.pi) % (2 * math.pi) - math.pi


def cubic_ease(t):
    """cubic ease-in-out: t∈[0,1] → [0,1], 两端速度为零"""
    return 3 * t * t - 2 * t * t * t


def build_cubic_spline_trajectory(q_start, q_end, duration, steps):
    """
    对每个关节独立构建 cubic ease-in-out 轨迹。
    返回 numpy 数组 shape=(steps+1, 6)
    """
    import numpy as np

    q_start = np.array(q_start, dtype=np.float64)
    q_end = np.array(q_end, dtype=np.float64)

    # 处理旋转关节：走最短弧
    for i in range(6):
        diff = shortest_angle_diff(q_start[i], q_end[i])
        q_end_adj = q_start[i] + diff
        q_end[i] = q_end_adj

    traj = np.zeros((steps + 1, 6), dtype=np.float64)
    for k in range(steps + 1):
        t = k / steps               # 0..1
        alpha = cubic_ease(t)       # ease-in-out
        traj[k] = q_start + alpha * (q_end - q_start)

    return traj


def main():
    rclpy.init(args=sys.argv)
    node = Node('demo_trajectory')
    client = node.create_client(SetModelConfiguration, '/gazebo/set_model_configuration')

    node.get_logger().info('等待 Gazebo 服务...')
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error('Gazebo 服务不可用，请先启动 Gazebo')
        return

    # ── 轨迹参数 ──
    Q_START = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]          # 零位
    Q_END   = [0.0, 0.0, 0.0, 0.0, math.pi / 2, 0.0]  # wrist_3 转 90°
    DURATION = 2.0   # 秒
    RATE = 100        # Hz

    steps = int(DURATION * RATE)
    traj = build_cubic_spline_trajectory(Q_START, Q_END, DURATION, steps)

    node.get_logger().info(
        f'轨迹就绪: {steps+1} 个路径点 @ {RATE}Hz, 时长 {DURATION}s')
    node.get_logger().info(
        f'起点: {[f"{a:.3f}" for a in Q_START]}')
    node.get_logger().info(
        f'终点: {[f"{a:.3f}" for a in Q_END]}')

    # ── 发送路径点 ──
    dt = 1.0 / RATE
    t0 = time.time()

    for k, q in enumerate(traj):
        # 重新包回 [-pi, pi)
        q_wrapped = [(float(a) + math.pi) % (2 * math.pi) - math.pi for a in q]

        req = SetModelConfiguration.Request()
        req.model_name = _MODEL
        req.joint_names = list(_JOINTS)
        req.joint_positions = q_wrapped
        client.call_async(req)

        # 按节拍等待
        target_time = t0 + k * dt
        now = time.time()
        sleep_t = target_time - now
        if sleep_t > 0:
            time.sleep(sleep_t)

    elapsed = time.time() - t0
    node.get_logger().info(f'轨迹完成！实际耗时 {elapsed:.2f}s')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
