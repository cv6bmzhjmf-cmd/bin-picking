#!/usr/bin/env python3
"""立体匹配最终版：SGBM + WLS + 底板深度验证"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.ndimage import median_filter

class StereoMatcher(Node):
    def __init__(self):
        super().__init__('stereo_matcher')
        self.bridge = CvBridge()
        self.left_img = None
        self.right_img = None
        self.K = None

        self.declare_parameter('baseline', 0.06)
        self.baseline = self.get_parameter('baseline').value
        self._debug_saved = False

        # SGBM parameters
        self.declare_parameter('sgbm_min_disparity', 80)
        self.declare_parameter('sgbm_num_disparities', 144)
        self.declare_parameter('sgbm_block_size', 15)
        self.declare_parameter('sgbm_uniqueness_ratio', 5)
        self.declare_parameter('sgbm_speckle_window_size', 50)
        self.declare_parameter('sgbm_speckle_range', 16)
        self.declare_parameter('sgbm_pre_filter_cap', 31)
        # WLS parameters
        self.declare_parameter('wls_lambda', 2000)
        self.declare_parameter('wls_sigma_color', 2.5)

        self.left_sub = self.create_subscription(
            Image, '/stereo_camera/left/image_raw', self.left_cb, 10)
        self.right_sub = self.create_subscription(
            Image, '/stereo_camera/right/image_raw', self.right_cb, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/stereo_camera/left/camera_info', self.info_cb, 10)

        self.disp_pub = self.create_publisher(Image, '/stereo/disparity', 10)
        self.disp_raw_pub = self.create_publisher(Image, '/stereo/disparity_raw', 10)
        self.sbs_pub = self.create_publisher(Image, '/stereo/side_by_side', 10)

        self._init_matchers()

        self.timer = self.create_timer(1.0, self.process)

    def _init_matchers(self):
        bs = self.get_parameter('sgbm_block_size').value
        self.sgbm_l = cv2.StereoSGBM_create(
            minDisparity=self.get_parameter('sgbm_min_disparity').value,
            numDisparities=self.get_parameter('sgbm_num_disparities').value,
            blockSize=bs,
            P1=8 * 3 * bs**2,
            P2=32 * 3 * bs**2,
            disp12MaxDiff=-1,
            uniquenessRatio=self.get_parameter('sgbm_uniqueness_ratio').value,
            speckleWindowSize=self.get_parameter('sgbm_speckle_window_size').value,
            speckleRange=self.get_parameter('sgbm_speckle_range').value,
            preFilterCap=self.get_parameter('sgbm_pre_filter_cap').value,
        )
        self.sgbm_r = cv2.ximgproc.createRightMatcher(self.sgbm_l)
        self.wls = cv2.ximgproc.createDisparityWLSFilter(self.sgbm_l)
        self.wls.setLambda(self.get_parameter('wls_lambda').value)
        self.wls.setSigmaColor(self.get_parameter('wls_sigma_color').value)

    def info_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)

    def left_cb(self, msg):
        self.left_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def right_cb(self, msg):
        self.right_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def process(self):
        if self.left_img is None or self.right_img is None or self.K is None:
            return

        gl = cv2.cvtColor(self.left_img, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(self.right_img, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gl = clahe.apply(gl)
        gr = clahe.apply(gr)

        disp_l = self.sgbm_l.compute(gl, gr).astype(np.float32) / 16.0
        disp_r = self.sgbm_r.compute(gr, gl).astype(np.float32) / 16.0
        disp = self.wls.filter(disp_l, gl, None, disp_r)
        disp = np.clip(disp, 80, 224)
        disp = median_filter(disp.astype(np.float32), size=7)

        # 统计
        valid = disp > 80
        fx = self.K[0, 0]
        if valid.any():
            d_valid = disp[valid]
            z_valid = fx * self.baseline / d_valid
            # 取底板中心区域
            h, w = disp.shape
            patch = disp[h//3:2*h//3, w//3:2*w//3]
            pv = patch > 80
            z_patch = fx * self.baseline / patch[pv] if pv.any() else np.array([0])
            self.get_logger().info(
                f'中心区: d={patch[pv].mean():.1f}±{patch[pv].std():.1f}px '
                f'Z={z_patch.mean():.3f}±{z_patch.std():.3f}m '
                f'({pv.sum()}/{patch.size}) | '
                f'期望Z≈0.445m')

        # 深度图可视化
        depth = np.zeros_like(disp)
        depth[valid] = fx * self.baseline / disp[valid]
        depth_c = np.clip(depth, 0.30, 0.60)
        depth_n = ((depth_c - 0.30) / 0.30 * 255).astype(np.uint8)
        depth_v = cv2.applyColorMap(255 - depth_n, cv2.COLORMAP_JET)
        depth_v[depth <= 0] = 0
        cv2.putText(depth_v, 'NEAR(0.30m)red -- blue(0.60m)FAR', (10, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        self.disp_pub.publish(self.bridge.cv2_to_imgmsg(depth_v, 'bgr8'))
        self.disp_raw_pub.publish(self.bridge.cv2_to_imgmsg(disp.astype(np.float32), '32FC1'))

        # 并排
        h, w = gl.shape
        side = np.zeros((h, w * 2), dtype=np.uint8)
        side[:, :w] = gl; side[:, w:] = gr
        s = cv2.cvtColor(side, cv2.COLOR_GRAY2BGR)
        for y in range(0, h, 60):
            cv2.line(s, (0, y), (w * 2, y), (0, 255, 0), 1)
        self.sbs_pub.publish(self.bridge.cv2_to_imgmsg(s, 'bgr8'))

        if not self._debug_saved:
            cv2.imwrite('/tmp/left_gray.png', gl)
            cv2.imwrite('/tmp/right_gray.png', gr)
            cv2.imwrite('/tmp/depth.png', depth_v)
            self.get_logger().info('已保存')
            self._debug_saved = True


def main():
    rclpy.init()
    rclpy.spin(StereoMatcher())


if __name__ == '__main__':
    main()
