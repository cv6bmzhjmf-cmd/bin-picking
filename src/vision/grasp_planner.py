#!/usr/bin/env python3
"""物体位姿 → 抓取姿态生成 + 坐标变换 + UR5 IK (ikpy)"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, JointState
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
import numpy as np
import math
import warnings
warnings.filterwarnings('ignore', category=UserWarning, module='ikpy')
from ikpy.chain import Chain

_UR5_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# ikpy chain layout (from ur5 URDF):
#   idx 0: Base link (fixed)
#   idx 1: base_link-base_link_inertia (fixed)
#   idx 2: shoulder_pan_joint  ← UR5[0]
#   idx 3: shoulder_lift_joint ← UR5[1]
#   idx 4: elbow_joint         ← UR5[2]
#   idx 5: wrist_1_joint       ← UR5[3]
#   idx 6: wrist_2_joint       ← UR5[4]
#   idx 7: wrist_3_joint       ← UR5[5]
#   idx 8: wrist_3_link-ft_frame (fixed)
_IKPY_SIZE = 9
_UR5_IDX = [2, 3, 4, 5, 6, 7]

def _q6_to_ikpy(q6):
    arr = [0.0] * _IKPY_SIZE
    for i, idx in enumerate(_UR5_IDX):
        arr[idx] = float(q6[i])
    return arr

def _ikpy_to_q6(arr):
    return np.array([arr[i] for i in _UR5_IDX])


def rotmat_to_quat(R):
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        return np.array([(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s])
    elif m00 > m11 and m00 > m22:
        s = np.sqrt(1.0 + m00 - m11 - m22) * 2
        return np.array([0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s])
    elif m11 > m22:
        s = np.sqrt(1.0 + m11 - m00 - m22) * 2
        return np.array([(m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s])
    else:
        s = np.sqrt(1.0 + m22 - m00 - m11) * 2
        return np.array([(m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s])


class GraspPlanner(Node):
    def __init__(self):
        super().__init__('grasp_planner')
        self.object_poses = None
        self.object_cloud = None

        self.declare_parameter('urdf_path', '/tmp/ur5.urdf')
        self.declare_parameter('approach_height', 0.05)
        self.declare_parameter('cam_world_x', 0.47)
        self.declare_parameter('cam_world_y', 0.0)
        self.declare_parameter('cam_world_z', 0.5)
        self.declare_parameter('ur5_base_x', 0.5)
        self.declare_parameter('ur5_base_y', 0.35)
        self.declare_parameter('ur5_base_z', 0.0)

        urdf = self.get_parameter('urdf_path').value
        self.chain = Chain.from_urdf_file(urdf)
        self.get_logger().info(f'Loaded UR5 kinematics: {urdf}')

        self.poses_sub = self.create_subscription(
            PoseArray, '/stereo/object_poses', self.poses_cb, 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/stereo/objects_cloud', self.cloud_cb, 10)

        self.target_pub = self.create_publisher(PoseStamped, '/grasp_target', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, '/grasp_marker', 10)
        self.q_current = np.zeros(6)
        self.timer = self.create_timer(1.0, self.process)

    def poses_cb(self, msg):
        self.object_poses = msg

    def cloud_cb(self, msg):
        self.object_cloud = msg

    def process(self):
        if self.object_poses is None or self.object_cloud is None:
            return
        if not self.object_poses.poses:
            return

        # 1. Pick object closest to world (0.5, 0) — bin center
        best_idx = 0
        best_dist = np.inf
        for i, p in enumerate(self.object_poses.poses):
            ox, oy = p.position.x, p.position.y
            wx = self.get_parameter('cam_world_x').value - oy
            wy = self.get_parameter('cam_world_y').value + ox
            d = math.sqrt((wx - 0.5)**2 + (wy - 0.0)**2)
            if d < best_dist:
                best_dist = d
                best_idx = i

        target_pose_opt = self.object_poses.poses[best_idx]

        # 2. camera_left_optical → world coordinate transform
        # Camera at (0.47,0,0.5) looking down (pitch=90°).
        # optical_x(right)→world -y, optical_y(down)→world -x, optical_z(fwd)→world -z
        cx = self.get_parameter('cam_world_x').value
        cy = self.get_parameter('cam_world_y').value
        cz = self.get_parameter('cam_world_z').value
        ox = target_pose_opt.position.x
        oy = target_pose_opt.position.y
        oz = target_pose_opt.position.z

        world_x = cx - oy
        world_y = cy + ox
        world_z = cz - oz

        # optical → world rotation
        qo = target_pose_opt.orientation
        R_opt = self._quat_to_rotmat(qo.x, qo.y, qo.z, qo.w)
        R_opt2world = np.array([[0, -1, 0], [1, 0, 0], [0, 0, -1]], dtype=np.float64)
        R_world = R_opt2world @ R_opt

        # 3. Grasp pose (approach height above object center)
        approach = self.get_parameter('approach_height').value
        grasp_x = world_x
        grasp_y = world_y
        grasp_z = world_z + approach

        # Publish grasp target in world frame
        msg = PoseStamped()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='world')
        msg.pose.position.x = grasp_x
        msg.pose.position.y = grasp_y
        msg.pose.position.z = grasp_z
        q = rotmat_to_quat(R_world)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.target_pub.publish(msg)

        # 4. UR5 IK: target in UR5 base frame
        bx = self.get_parameter('ur5_base_x').value
        by = self.get_parameter('ur5_base_y').value
        bz = self.get_parameter('ur5_base_z').value
        dx = grasp_x - bx
        dy = grasp_y - by
        dz = grasp_z - bz

        # Push target outward if too close horizontally (UR5 wrist offset ~0.11m)
        r = math.sqrt(dx*dx + dy*dy)
        if r < 0.3:
            scale = 0.3 / r if r > 0.01 else 1.0
            dx *= scale
            dy *= scale
        # Keep z within reasonable reach; don't flip sign
        if dz < -0.5:
            dz = -0.5

        # ikpy position-only IK
        target = [dx, dy, dz]
        init = _q6_to_ikpy(self.q_current)
        try:
            ik_result = self.chain.inverse_kinematics(target, initial_position=init)
            q6 = _ikpy_to_q6(ik_result)

            # Verify with FK
            fk_result = self.chain.forward_kinematics(ik_result)
            fk_pos = fk_result[:3, 3]
        except Exception as e:
            self.get_logger().warn(f'IK failed: {e}')
            return

        self.q_current = q6

        # Publish joint states
        js = JointState()
        js.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='')
        js.name = _UR5_JOINTS
        js.position = [float(x) for x in q6]
        self.joint_pub.publish(js)

        # Marker at grasp target (world frame coordinates)
        mk = Marker()
        mk.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='world')
        mk.ns = 'grasp_target'
        mk.id = 0
        mk.type = Marker.SPHERE
        mk.action = Marker.ADD
        mk.pose.position = Point(x=grasp_x, y=grasp_y, z=grasp_z)
        mk.pose.orientation.w = 1.0
        mk.scale = Vector3(x=0.03, y=0.03, z=0.03)
        mk.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        mk.lifetime.sec = 2
        self.marker_pub.publish(mk)

        # End-effector actual position (green) — FK in UR5 base → world
        ee_wx = bx + fk_pos[0]
        ee_wy = by + fk_pos[1]
        ee_wz = bz + fk_pos[2]
        mk2 = Marker()
        mk2.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='world')
        mk2.ns = 'ee_actual'
        mk2.id = 0
        mk2.type = Marker.SPHERE
        mk2.action = Marker.ADD
        mk2.pose.position = Point(x=ee_wx, y=ee_wy, z=ee_wz)
        mk2.pose.orientation.w = 1.0
        mk2.scale = Vector3(x=0.03, y=0.03, z=0.03)
        mk2.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        mk2.lifetime.sec = 2
        self.marker_pub.publish(mk2)

        self.get_logger().info(
            f'grasp_world=({grasp_x:.3f},{grasp_y:.3f},{grasp_z:.3f}) '
            f'target_base=({dx:.3f},{dy:.3f},{dz:.3f}) '
            f'FK=({fk_pos[0]:.3f},{fk_pos[1]:.3f},{fk_pos[2]:.3f}) '
            f'err={np.linalg.norm(fk_pos - target):.3f}m '
            f'q=[{q6[0]:.2f},{q6[1]:.2f},{q6[2]:.2f},{q6[3]:.2f},{q6[4]:.2f},{q6[5]:.2f}]',
            throttle_duration_sec=2.0)

    @staticmethod
    def _quat_to_rotmat(x, y, z, w):
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.array([
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ])


def main():
    rclpy.init()
    rclpy.spin(GraspPlanner())


if __name__ == '__main__':
    main()
