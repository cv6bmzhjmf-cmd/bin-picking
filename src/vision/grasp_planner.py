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
        self.declare_parameter('bin_center_x', 0.5)
        self.declare_parameter('bin_center_y', 0.0)
        self.declare_parameter('bin_center_z', 0.05)
        self.declare_parameter('bin_size_x', 0.4)
        self.declare_parameter('bin_size_y', 0.3)
        self.declare_parameter('bin_size_z', 0.15)
        self.declare_parameter('bin_z_tolerance', 0.35)
        self.declare_parameter('pre_grasp_height', 0.10)
        self.declare_parameter('ur5_max_reach', 0.85)

        urdf = self.get_parameter('urdf_path').value
        self.chain = Chain.from_urdf_file(urdf)
        self.get_logger().info(f'Loaded UR5 kinematics: {urdf}')

        self.poses_sub = self.create_subscription(
            PoseArray, '/stereo/object_poses', self.poses_cb, 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/stereo/objects_cloud', self.cloud_cb, 10)

        self.target_pub = self.create_publisher(PoseStamped, '/grasp_target', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.approach_pub = self.create_publisher(JointState, '/approach_joints', 10)
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

        cx = self.get_parameter('cam_world_x').value
        cy = self.get_parameter('cam_world_y').value
        cz = self.get_parameter('cam_world_z').value
        bx = self.get_parameter('ur5_base_x').value
        by = self.get_parameter('ur5_base_y').value
        bz = self.get_parameter('ur5_base_z').value
        bcx = self.get_parameter('bin_center_x').value
        bcy = self.get_parameter('bin_center_y').value
        bcz = self.get_parameter('bin_center_z').value
        bsx = self.get_parameter('bin_size_x').value
        bsy = self.get_parameter('bin_size_y').value
        bsz = self.get_parameter('bin_size_z').value
        max_reach = self.get_parameter('ur5_max_reach').value
        approach = self.get_parameter('approach_height').value
        z_tol = self.get_parameter('bin_z_tolerance').value

        # 1. Sort objects by distance from bin center (world frame)
        objects = []
        for i, p in enumerate(self.object_poses.poses):
            wy = cx - p.position.y
            wx = cy + p.position.x
            d = math.sqrt((wx - bcx)**2 + (wy - bcy)**2)
            objects.append((d, i, p))
        objects.sort(key=lambda x: x[0])

        best_result = None

        failed_collision = 0
        failed_reach = 0
        failed_ik = 0

        for dist, idx, p in objects:
            # camera_left_optical → world
            wx = cx - p.position.y
            wy = cy + p.position.x
            wz = cz - p.position.z
            gx, gy, gz = wx, wy, wz + approach

            # Collision: grasp point must be inside bin
            if not self._inside_bin(gx, gy, gz, bcx, bcy, bcz, bsx, bsy, bsz, z_tol):
                failed_collision += 1
                continue

            # Reachability: distance from UR5 base
            dx = gx - bx
            dy = gy - by
            dz = gz - bz
            if math.sqrt(dx*dx + dy*dy + dz*dz) > max_reach:
                failed_reach += 1
                continue

            # Enforce minimum horizontal distance
            r = math.sqrt(dx*dx + dy*dy)
            if r < 0.3:
                scale = 0.3 / r if r > 0.01 else 1.0
                dx *= scale
                dy *= scale
            if dz < -0.5:
                dz = -0.5

            # IK (try oriented first, fall back to position-only if error > 0.01)
            target = [dx, dy, dz]
            init = _q6_to_ikpy(self.q_current)
            R_down = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
            q6 = None
            for method in ['oriented', 'position']:
                try:
                    if method == 'oriented':
                        ik_result = self.chain.inverse_kinematics(
                            target, target_orientation=R_down, initial_position=init)
                    else:
                        ik_result = self.chain.inverse_kinematics(target, initial_position=init)
                    q6 = _ikpy_to_q6(ik_result)
                    fk_result = self.chain.forward_kinematics(ik_result)
                    fk_pos = fk_result[:3, 3]
                    if np.linalg.norm(fk_pos - target) <= 0.01:
                        ik_method = method
                        break
                except Exception:
                    continue
            if q6 is None:
                failed_ik += 1
                continue

            # optical → world rotation
            qo = p.orientation
            R_opt = self._quat_to_rotmat(qo.x, qo.y, qo.z, qo.w)
            R_opt2world = np.array([[0, -1, 0], [1, 0, 0], [0, 0, -1]], dtype=np.float64)
            R_world = R_opt2world @ R_opt

            best_result = (idx, gx, gy, gz, dx, dy, dz, q6, ik_method, fk_pos, qo, R_world)
            break

        if best_result is None:
            self.get_logger().warn(
                f'No reachable object: {failed_collision} collision '
                f'{failed_reach} reach {failed_ik} ik — {len(objects)} total',
                throttle_duration_sec=2.0)
            return

        obj_idx, gx, gy, gz, dx, dy, dz, q6, ik_method, fk_pos, qo, R_world = best_result
        self.q_current = q6

        # Publish grasp target
        msg = PoseStamped()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='world')
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.position.z = gz
        q = rotmat_to_quat(R_world)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.target_pub.publish(msg)

        # Pre-grasp waypoint (above object to avoid bin walls)
        pre_gz = gz + self.get_parameter('pre_grasp_height').value
        pre_dz = pre_gz - bz
        pre_r = math.sqrt(dx*dx + dy*dy)
        if pre_r < 0.3:
            scale = 0.3 / pre_r if pre_r > 0.01 else 1.0
            pre_dx = dx * scale
            pre_dy = dy * scale
        else:
            pre_dx, pre_dy = dx, dy
        pre_target = [pre_dx, pre_dy, pre_dz]
        pre_init = _q6_to_ikpy(self.q_current)
        try:
            pre_ik = self.chain.inverse_kinematics(pre_target, initial_position=pre_init)
            pre_q6 = _ikpy_to_q6(pre_ik)
            pre_fk = self.chain.forward_kinematics(pre_ik)
            pre_err = np.linalg.norm(pre_fk[:3, 3] - pre_target)
        except Exception:
            pre_q6, pre_err = None, 999

        # Publish grasp joint states
        js = JointState()
        js.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='')
        js.name = _UR5_JOINTS
        js.position = [float(x) for x in q6]
        self.joint_pub.publish(js)

        # Publish pre-grasp approach joints
        if pre_q6 is not None:
            ajs = JointState()
            ajs.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='')
            ajs.name = _UR5_JOINTS
            ajs.position = [float(x) for x in pre_q6]
            self.approach_pub.publish(ajs)

        # Blue marker: pre-grasp approach point
        if pre_q6 is not None:
            mk3 = Marker()
            mk3.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='world')
            mk3.ns = 'pre_grasp'
            mk3.id = 0
            mk3.type = Marker.SPHERE
            mk3.action = Marker.ADD
            mk3.pose.position = Point(x=gx, y=gy, z=pre_gz)
            mk3.pose.orientation.w = 1.0
            mk3.scale = Vector3(x=0.025, y=0.025, z=0.025)
            mk3.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
            mk3.lifetime.sec = 2
            self.marker_pub.publish(mk3)

        # Red marker: grasp target
        mk = Marker()
        mk.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='world')
        mk.ns = 'grasp_target'
        mk.id = 0
        mk.type = Marker.SPHERE
        mk.action = Marker.ADD
        mk.pose.position = Point(x=gx, y=gy, z=gz)
        mk.pose.orientation.w = 1.0
        mk.scale = Vector3(x=0.03, y=0.03, z=0.03)
        mk.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        mk.lifetime.sec = 2
        self.marker_pub.publish(mk)

        # Green marker: end-effector actual
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

        pre_info = f'pre-grasp err={pre_err:.3f}m' if pre_q6 is not None else 'pre-grasp FAIL'
        self.get_logger().info(
            f'obj#{obj_idx} grasp_world=({gx:.3f},{gy:.3f},{gz:.3f}) '
            f'target_base=({dx:.3f},{dy:.3f},{dz:.3f}) '
            f'FK=({fk_pos[0]:.3f},{fk_pos[1]:.3f},{fk_pos[2]:.3f}) '
            f'err={np.linalg.norm(fk_pos - np.array([dx,dy,dz])):.3f}m '
            f'method={ik_method} {pre_info} '
            f'q=[{q6[0]:.2f},{q6[1]:.2f},{q6[2]:.2f},{q6[3]:.2f},{q6[4]:.2f},{q6[5]:.2f}]',
            throttle_duration_sec=2.0)

    @staticmethod
    def _inside_bin(gx, gy, gz, bcx, bcy, bcz, bsx, bsy, bsz, z_tol):
        """Check if grasp point is within the bin bounding box."""
        margin = 0.02
        return (abs(gx - bcx) < bsx / 2 - margin and
                abs(gy - bcy) < bsy / 2 - margin and
                gz > bcz - z_tol and gz < bcz + bsz + z_tol)

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
