#!/usr/bin/python3
"""
KFS 检测节点
使用 D435i 深度+彩色相机检测 350mm 武术秘籍立方体
输出: 每个 KFS 的 (x, y, z) 相机坐标
"""
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import json


class KFSDetector(Node):
    def __init__(self):
        super().__init__('kfs_detector')
        self.declare_parameter('depth_topic', '/d435i/d435i/aligned_depth_to_color/image_raw')
        self.declare_parameter('color_topic', '/d435i/d435i/color/image_raw')
        self.declare_parameter('camera_info', '/d435i/d435i/color/camera_info')
        self.declare_parameter('cube_size_mm', 350)
        self.declare_parameter('min_distance_mm', 200)
        self.declare_parameter('max_distance_mm', 3000)
        self.declare_parameter('depth_scale', 0.001)

        self.bridge = CvBridge()
        self.depth_image = None
        self.fx = 636.0  # 默认
        self.fy = 636.0
        self.cx = 640.0
        self.cy = 360.0
        self.cube_size = self.get_parameter('cube_size_mm').value / 1000.0
        self.min_dist = self.get_parameter('min_distance_mm').value / 1000.0
        self.max_dist = self.get_parameter('max_distance_mm').value / 1000.0
        self.depth_scale = self.get_parameter('depth_scale').value

        self.sub_color = self.create_subscription(
            Image, self.get_parameter('color_topic').value, self.color_cb, 10)
        self.sub_depth = self.create_subscription(
            Image, self.get_parameter('depth_topic').value, self.depth_cb, 10)
        self.sub_info = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info').value, self.info_cb, 10)

        self.pub_marker = self.create_publisher(MarkerArray, '/r2/kfs_markers', 10)
        self.pub_result = self.create_publisher(Float32MultiArray, '/r2/kfs_result', 10)
        self.pub_info = self.create_publisher(String, '/r2/kfs_info', 10)

        self.timer = self.create_timer(0.1, self.detect_loop)
        self.get_logger().info('KFS detector started')

    def info_cb(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def color_cb(self, msg: Image):
        pass  # 保留用于彩色辅助检测

    def depth_cb(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    def detect_loop(self):
        if self.depth_image is None:
            return
        depth = self.depth_image.copy()
        kfs_list = self._detect_cubes(depth)
        self._publish_results(kfs_list)

    def _detect_cubes(self, depth: np.ndarray) -> list:
        """深度图检测立方体"""
        h, w = depth.shape
        mask = (depth > self.min_dist / self.depth_scale) & \
               (depth < self.max_dist / self.depth_scale)
        depth_valid = depth.copy()
        depth_valid[~mask] = 0

        depth_vis = cv2.normalize(depth_valid, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        grad_x = cv2.Sobel(depth_vis, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(depth_vis, cv2.CV_32F, 0, 1, ksize=3)
        edges = np.sqrt(grad_x ** 2 + grad_y ** 2)
        edges = (edges > 15).astype(np.uint8) * 255

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        results = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 200:
                continue
            x, y, cw, ch = cv2.boundingRect(cnt)
            aspect = float(cw) / max(ch, 1)
            if aspect < 0.3 or aspect > 3.0:
                continue

            mask_roi = np.zeros((h, w), dtype=np.uint8)
            cv2.drawContours(mask_roi, [cnt], -1, 255, -1)

            cx_px = int(x + cw / 2)
            cy_px = int(y + ch / 2)
            if 0 <= cx_px < w and 0 <= cy_px < h:
                roi_depth = depth_valid[cy_px - 3:cy_px + 3, cx_px - 3:cx_px + 3]
                roi_depth = roi_depth[roi_depth > 0]
                if len(roi_depth) < 5:
                    continue
                med_depth = np.median(roi_depth) * self.depth_scale

                # 单发深度点来计算 3D
                Zc = med_depth
                Xc = (cx_px - self.cx) / self.fx * Zc
                Yc = (cy_px - self.cy) / self.fy * Zc

                results.append({
                    'x': Xc, 'y': Yc, 'z': Zc,
                    'cx': cx_px, 'cy': cy_px,
                    'width': cw, 'height': ch,
                })
        return results

    def _publish_results(self, kfs_list: list):
        if not kfs_list:
            return
        markers = MarkerArray()
        arr = Float32MultiArray()
        infos = []
        for i, kfs in enumerate(kfs_list):
            marker = Marker()
            marker.header.frame_id = 'camera_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'kfs'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = kfs['x']
            marker.pose.position.y = kfs['y']
            marker.pose.position.z = kfs['z']
            s = self.cube_size
            marker.scale.x = s
            marker.scale.y = s
            marker.scale.z = s
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 0.7
            markers.markers.append(marker)
            arr.data.extend([kfs['x'], kfs['y'], kfs['z']])
            infos.append(f'{kfs["x"]:.3f},{kfs["y"]:.3f},{kfs["z"]:.3f}')
        self.pub_marker.publish(markers)
        self.pub_result.publish(arr)
        self.pub_info.publish(String(data=';'.join(infos)))


def main():
    rclpy.init()
    node = KFSDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
