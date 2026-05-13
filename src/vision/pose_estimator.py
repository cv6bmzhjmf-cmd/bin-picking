#!/usr/bin/env python3
"""视差→点云→分割→聚类→6D位姿估计"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d
from std_msgs.msg import Header


class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        self.bridge = CvBridge()
        self.disp = None
        self.K = None

        self.declare_parameter('voxel_size', 0.002)
        self.declare_parameter('plane_distance_threshold', 0.008)
        self.declare_parameter('dbscan_eps', 0.02)
        self.declare_parameter('dbscan_min_points', 30)
        self.declare_parameter('icp_enabled', False)
        self.declare_parameter('icp_max_iterations', 50)
        self.declare_parameter('baseline', 0.06)
        self.declare_parameter('max_depth', 2.0)
        self.declare_parameter('min_cluster_points', 300)

        self.disp_sub = self.create_subscription(
            Image, '/stereo/disparity_raw', self.disp_cb, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/stereo_camera/left/camera_info', self.info_cb, 10)

        self.cloud_pub = self.create_publisher(PointCloud2, '/stereo/point_cloud', 10)
        self.objects_pub = self.create_publisher(PointCloud2, '/stereo/objects_cloud', 10)
        self.poses_pub = self.create_publisher(PoseArray, '/stereo/object_poses', 10)

        self.timer = self.create_timer(1.0, self.process)

    def info_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)

    def disp_cb(self, msg):
        self.disp = self.bridge.imgmsg_to_cv2(msg, '32FC1')

    def process(self):
        if self.disp is None or self.K is None:
            return

        baseline = self.get_parameter('baseline').value
        max_depth = self.get_parameter('max_depth').value
        fx = self.K[0, 0]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        # 1. 视差 → 3D 点云（手动计算，避免 Q 矩阵符号问题）
        d = self.disp.astype(np.float32)
        valid = d > 0
        if not valid.any():
            return

        Z = fx * baseline / d[valid]
        u, v = np.meshgrid(np.arange(d.shape[1]), np.arange(d.shape[0]))
        X = (u[valid] - cx) * Z / fx
        Y = (v[valid] - cy) * Z / fx
        pts = np.stack([X, Y, Z], axis=-1)

        # 裁切深度
        pts = pts[pts[:, 2] < max_depth]
        if len(pts) == 0:
            return

        # → Open3D
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        self.publish_cloud(pcd, self.cloud_pub, 'camera_left_optical')

        # 2. 体素降采样
        voxel_size = self.get_parameter('voxel_size').value
        pcd = pcd.voxel_down_sample(voxel_size)

        # 3. RANSAC 平面分割 → 去底板
        plane_thresh = self.get_parameter('plane_distance_threshold').value
        plane_model, inliers = pcd.segment_plane(plane_thresh, 3, 1000)
        objects_cloud = pcd.select_by_index(inliers, invert=True)

        if len(objects_cloud.points) == 0:
            return

        # 4. DBSCAN 聚类
        eps = self.get_parameter('dbscan_eps').value
        min_points = self.get_parameter('dbscan_min_points').value
        labels = np.array(objects_cloud.cluster_dbscan(eps, min_points, print_progress=False))
        min_cluster = self.get_parameter('min_cluster_points').value

        all_object_points = []
        poses = PoseArray()
        poses.header = Header(stamp=self.get_clock().now().to_msg(),
                              frame_id='camera_left_optical')

        for label in set(labels):
            if label < 0:
                continue
            cluster_idx = np.where(labels == label)[0]
            if len(cluster_idx) < min_cluster:
                continue
            cluster = objects_cloud.select_by_index(cluster_idx.tolist())
            all_object_points.append(cluster)
            pose = self.estimate_pose(cluster)
            poses.poses.append(pose)

        # 发布物体合并点云
        if all_object_points:
            merged = all_object_points[0]
            for p in all_object_points[1:]:
                merged += p
            self.publish_cloud(merged, self.objects_pub, 'camera_left_optical')

        # 发布位姿
        if poses.poses:
            self.poses_pub.publish(poses)
            self.get_logger().info(
                f'检测到 {len(poses.poses)} 个物体', throttle_duration_sec=2.0)

    def estimate_pose(self, cluster: o3d.geometry.PointCloud) -> Pose:
        """质心 + PCA → 6D 位姿初值"""
        pts = np.asarray(cluster.points)
        center = pts.mean(axis=0)  # 质心

        # PCA 主方向 → 旋转矩阵
        centered = pts - center
        cov = centered.T @ centered / (len(pts) - 1)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)

        # 按特征值升序排列，取最大三个
        idx = np.argsort(eigenvalues)[::-1]
        R_cv = eigenvectors[:, idx]

        # 保证右手坐标系 (det = +1)
        if np.linalg.det(R_cv) < 0:
            R_cv[:, 2] *= -1

        # ICP 精化（可选）
        icp_enabled = self.get_parameter('icp_enabled').value
        if icp_enabled:
            R_cv, center = self.refine_icp(cluster, R_cv, center)

        # 转换为 ROS 姿态（四元数）
        pose = Pose()
        pose.position.x = float(center[0])
        pose.position.y = float(center[1])
        pose.position.z = float(center[2])

        # 旋转矩阵 → 四元数
        q = self.rotmat_to_quat(R_cv)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose

    def refine_icp(self, cluster, R_init, t_init):
        """ICP 精化：用聚类自身做 target（学习演示用）"""
        target = cluster.voxel_down_sample(self.get_parameter('voxel_size').value)
        target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.005, max_nn=30))

        # 源 = target 经初始位姿扰动
        T_init = np.eye(4)
        T_init[:3, :3] = R_init
        T_init[:3, 3] = t_init
        source = cluster.voxel_down_sample(self.get_parameter('voxel_size').value)
        source.transform(T_init)

        max_iter = self.get_parameter('icp_max_iterations').value
        reg = o3d.pipelines.registration.registration_icp(
            source, target, 0.005, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=max_iter))

        if reg.fitness < 0.3:
            self.get_logger().debug('ICP fitness < 0.3，保留几何初值')
            return R_init, t_init

        T_refined = reg.transformation
        return T_refined[:3, :3], T_refined[:3, 3]

    def publish_cloud(self, pcd, publisher, frame_id):
        """Open3D PointCloud → ROS2 PointCloud2"""
        pts = np.asarray(pcd.points, dtype=np.float32)
        msg = PointCloud2()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=frame_id)
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * len(pts)
        msg.is_bigendian = False
        msg.is_dense = True
        msg.data = pts.tobytes()
        publisher.publish(msg)

    @staticmethod
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


def main():
    rclpy.init()
    rclpy.spin(PoseEstimator())


if __name__ == '__main__':
    main()
