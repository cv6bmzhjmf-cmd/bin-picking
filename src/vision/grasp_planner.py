#!/usr/bin/env python3
"""物体位姿 → 抓取姿态生成 + 坐标变换 + UR5 IK (ikpy)"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA, Float64
import numpy as np
import math
import warnings
warnings.filterwarnings('ignore', category=UserWarning, module='ikpy')
from ikpy.chain import Chain
from geometry_utils import rotmat_to_quat, quat_to_rotmat, optical_to_world
from gazebo_msgs.srv import SetModelConfiguration

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


class GraspPlanner(Node):
    def __init__(self):
        super().__init__('grasp_planner')
        self.object_poses = None

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
        self.declare_parameter('bin_z_tolerance', 0.05)
        self.declare_parameter('pre_grasp_height', 0.10)
        self.declare_parameter('ur5_max_reach', 0.85)
        self.declare_parameter('use_gazebo_joints', True)

        urdf = self.get_parameter('urdf_path').value
        self.chain = Chain.from_urdf_file(urdf)
        self.get_logger().info(f'Loaded UR5 kinematics: {urdf}')

        self.poses_sub = self.create_subscription(
            PoseArray, '/stereo/object_poses', self.poses_cb, 10)

        self.target_pub = self.create_publisher(PoseStamped, '/grasp_target', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.approach_pub = self.create_publisher(JointState, '/approach_joints', 10)
        self.marker_pub = self.create_publisher(Marker, '/grasp_marker', 10)
        self.gripper_pub = self.create_publisher(Float64, '/gripper_cmd', 10)

        self.gazebo_client = self.create_client(
            SetModelConfiguration, '/gazebo/set_model_configuration')

        self.q_current = np.zeros(6)
        self.grasp_phase = 'approach'
        self.timer = self.create_timer(1.0, self.process)

    def poses_cb(self, msg):
        self.object_poses = msg

    def process(self):
        if self.object_poses is None:
            return
        if not self.object_poses.poses:
            return

        params = {p: self.get_parameter(p).value for p in [
            'cam_world_x', 'cam_world_y', 'cam_world_z', 'ur5_base_x', 'ur5_base_y',
            'ur5_base_z', 'bin_center_x', 'bin_center_y', 'bin_center_z',
            'bin_size_x', 'bin_size_y', 'bin_size_z', 'ur5_max_reach',
            'approach_height', 'bin_z_tolerance', 'pre_grasp_height']}

        objects = self._sort_objects(params['cam_world_x'], params['cam_world_y'],
                                     params['bin_center_x'], params['bin_center_y'])
        best = self._find_best_grasp(objects, params)
        if best is None:
            return

        self.q_current = best['q6']
        self._publish_grasp_target(best)

        pre_q6, pre_gz = self._compute_pre_grasp(best, params)
        self._execute_phase(best['q6'], pre_q6)
        self._publish_markers(best, pre_gz, pre_q6, params)

        grip = 'open' if self.grasp_phase == 'approach' else 'close'
        self.get_logger().info(
            f'obj#{best["idx"]} {self.grasp_phase} gripper={grip} '
            f'optical=({best["ox"]:.3f},{best["oy"]:.3f},{best["oz"]:.3f}) '
            f'world=({best["gx"]:.3f},{best["gy"]:.3f},{best["gz"]:.3f}) '
            f'err={np.linalg.norm(best["fk_pos"] - np.array([best["dx"],best["dy"],best["dz"]])):.3f}m '
            f'method={best["ik_method"]} '
            f'q=[{best["q6"][0]:.2f},{best["q6"][1]:.2f},{best["q6"][2]:.2f},'
            f'{best["q6"][3]:.2f},{best["q6"][4]:.2f},{best["q6"][5]:.2f}]',
            throttle_duration_sec=2.0)

    @staticmethod
    def _inside_bin(gx, gy, gz, bcx, bcy, bcz, bsx, bsy, bsz, z_tol):
        margin = 0.02
        return (abs(gx - bcx) < bsx / 2 - margin and
                abs(gy - bcy) < bsy / 2 - margin and
                gz > bcz - z_tol and gz < bcz + bsz + z_tol)

    def _sort_objects(self, cam_x, cam_y, bin_cx, bin_cy):
        objects = []
        for i, p in enumerate(self.object_poses.poses):
            wx, wy, _ = optical_to_world(p.position.x, p.position.y, p.position.z,
                                         cam_x, cam_y, 0.0)
            d = math.sqrt((wx - bin_cx)**2 + (wy - bin_cy)**2)
            objects.append((d, i, p))
        objects.sort(key=lambda x: x[0])
        return objects

    def _find_best_grasp(self, objects, params):
        cx, cy, cz = params['cam_world_x'], params['cam_world_y'], params['cam_world_z']
        bx, by, bz = params['ur5_base_x'], params['ur5_base_y'], params['ur5_base_z']
        bcx, bcy, bcz = params['bin_center_x'], params['bin_center_y'], params['bin_center_z']
        bsx, bsy, bsz = params['bin_size_x'], params['bin_size_y'], params['bin_size_z']
        max_reach = params['ur5_max_reach']
        approach = params['approach_height']
        z_tol = params['bin_z_tolerance']

        failed_collision, failed_reach, failed_ik = 0, 0, 0

        for dist, idx, p in objects:
            ox, oy, oz = p.position.x, p.position.y, p.position.z
            wx, wy, wz = optical_to_world(ox, oy, oz, cx, cy, cz)
            gx, gy, gz = wx, wy, wz + approach

            if not self._inside_bin(gx, gy, gz, bcx, bcy, bcz, bsx, bsy, bsz, z_tol):
                failed_collision += 1
                continue

            dx, dy, dz = gx - bx, gy - by, gz - bz
            if math.sqrt(dx*dx + dy*dy + dz*dz) > max_reach:
                failed_reach += 1
                continue

            r = math.sqrt(dx*dx + dy*dy)
            if r < 0.3:
                scale = 0.3 / r if r > 0.01 else 1.0
                dx *= scale; dy *= scale
            if dz < -0.5:
                dz = -0.5

            target = [dx, dy, dz]
            init = _q6_to_ikpy(self.q_current)
            R_down = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
            q6 = None; ik_method = None
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

            qo = p.orientation
            R_opt = quat_to_rotmat(qo.x, qo.y, qo.z, qo.w)
            R_opt2world = np.array([[0, -1, 0], [1, 0, 0], [0, 0, -1]], dtype=np.float64)
            R_world = R_opt2world @ R_opt

            return {'idx': idx, 'gx': gx, 'gy': gy, 'gz': gz,
                    'dx': dx, 'dy': dy, 'dz': dz,
                    'q6': q6, 'ik_method': ik_method, 'fk_pos': fk_pos,
                    'qo': qo, 'R_world': R_world, 'ox': ox, 'oy': oy, 'oz': oz}

        self.get_logger().warn(
            f'No reachable object: {failed_collision} collision '
            f'{failed_reach} reach {failed_ik} ik — {len(objects)} total',
            throttle_duration_sec=2.0)
        return None

    def _publish_grasp_target(self, best):
        msg = PoseStamped()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='world')
        msg.pose.position.x = best['gx']
        msg.pose.position.y = best['gy']
        msg.pose.position.z = best['gz']
        q = rotmat_to_quat(best['R_world'])
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.target_pub.publish(msg)

    def _compute_pre_grasp(self, best, params):
        bz = params['ur5_base_z']
        pre_gz = best['gz'] + params['pre_grasp_height']
        pre_dz = pre_gz - bz
        pre_r = math.sqrt(best['dx']*best['dx'] + best['dy']*best['dy'])
        if pre_r < 0.3:
            scale = 0.3 / pre_r if pre_r > 0.01 else 1.0
            pre_dx = best['dx'] * scale
            pre_dy = best['dy'] * scale
        else:
            pre_dx, pre_dy = best['dx'], best['dy']
        pre_target = [pre_dx, pre_dy, pre_dz]
        pre_init = _q6_to_ikpy(self.q_current)
        try:
            pre_ik = self.chain.inverse_kinematics(pre_target, initial_position=pre_init)
            pre_q6 = _ikpy_to_q6(pre_ik)
            pre_fk = self.chain.forward_kinematics(pre_ik)
            if np.linalg.norm(pre_fk[:3, 3] - pre_target) > 0.01:
                pre_q6 = None
        except Exception:
            pre_q6 = None
        return pre_q6, pre_gz

    def _execute_phase(self, q6, pre_q6):
        self.grasp_phase = 'grasp' if self.grasp_phase == 'approach' else 'approach'
        if self.grasp_phase == 'approach' and pre_q6 is not None:
            angles = pre_q6
            self.gripper_pub.publish(Float64(data=1.0))
        else:
            angles = q6
            self.gripper_pub.publish(Float64(data=0.0))
        js = JointState()
        js.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='')
        js.name = _UR5_JOINTS
        js.position = [float(x) for x in angles]
        self.joint_pub.publish(js)

        if self.get_parameter('use_gazebo_joints').value:
            self._set_gazebo_joints(angles)

    def _set_gazebo_joints(self, angles):
        if not self.gazebo_client.service_is_ready():
            self.get_logger().debug('Gazebo service not ready', throttle_duration_sec=5.0)
            return
        req = SetModelConfiguration.Request()
        req.model_name = 'ur5'
        req.joint_names = list(_UR5_JOINTS)
        req.joint_positions = [float(a) for a in angles]
        self.gazebo_client.call_async(req)

    def _publish_markers(self, best, pre_gz, pre_q6, params):
        bx, by, bz = params['ur5_base_x'], params['ur5_base_y'], params['ur5_base_z']
        # Blue: pre-grasp approach point
        if pre_q6 is not None:
            mk3 = Marker()
            mk3.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='world')
            mk3.ns = 'pre_grasp'; mk3.id = 0
            mk3.type = Marker.SPHERE; mk3.action = Marker.ADD
            mk3.pose.position = Point(x=best['gx'], y=best['gy'], z=pre_gz)
            mk3.pose.orientation.w = 1.0
            mk3.scale = Vector3(x=0.025, y=0.025, z=0.025)
            mk3.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
            mk3.lifetime.sec = 2
            self.marker_pub.publish(mk3)
        # Red: grasp target
        mk = Marker()
        mk.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='world')
        mk.ns = 'grasp_target'; mk.id = 0
        mk.type = Marker.SPHERE; mk.action = Marker.ADD
        mk.pose.position = Point(x=best['gx'], y=best['gy'], z=best['gz'])
        mk.pose.orientation.w = 1.0
        mk.scale = Vector3(x=0.03, y=0.03, z=0.03)
        mk.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        mk.lifetime.sec = 2
        self.marker_pub.publish(mk)
        # Green: end-effector actual
        fk = best['fk_pos']
        mk2 = Marker()
        mk2.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='world')
        mk2.ns = 'ee_actual'; mk2.id = 0
        mk2.type = Marker.SPHERE; mk2.action = Marker.ADD
        mk2.pose.position = Point(x=bx + fk[0], y=by + fk[1], z=bz + fk[2])
        mk2.pose.orientation.w = 1.0
        mk2.scale = Vector3(x=0.03, y=0.03, z=0.03)
        mk2.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        mk2.lifetime.sec = 2
        self.marker_pub.publish(mk2)


def main():
    rclpy.init()
    rclpy.spin(GraspPlanner())


if __name__ == '__main__':
    main()
