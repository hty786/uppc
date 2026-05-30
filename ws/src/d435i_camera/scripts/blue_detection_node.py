#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np


class BlueDetectionNode(Node):
    def __init__(self):
        super().__init__('blue_detection_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_topic', '/d435i/d435i/color/image_raw'),
                ('hue_low', 100),
                ('hue_high', 130),
                ('sat_low', 80),
                ('sat_high', 255),
                ('val_low', 80),
                ('val_high', 255),
                ('min_area', 500),
                ('min_aspect_ratio', 2.0),
                ('publish_annotated', True),
            ]
        )

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_callback,
            1
        )
        self.marker_pub = self.create_publisher(
            Marker, '/d435i/d435i/blue_detection/marker', 1
        )
        self.image_pub = self.create_publisher(
            Image, '/d435i/d435i/blue_detection/image', 1
        )

        self.get_logger().info('Blue detection node started')

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        h_low = self.get_parameter('hue_low').value
        h_high = self.get_parameter('hue_high').value
        s_low = self.get_parameter('sat_low').value
        s_high = self.get_parameter('sat_high').value
        v_low = self.get_parameter('val_low').value
        v_high = self.get_parameter('val_high').value
        min_area = self.get_parameter('min_area').value
        min_aspect = self.get_parameter('min_aspect_ratio').value

        lower = np.array([h_low, s_low, v_low])
        upper = np.array([h_high, s_high, v_high])
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = max(w, h) / max(float(min(w, h)), 1)
            if aspect < min_aspect:
                continue
            if area > best_area:
                best_area = area
                best = (x, y, w, h)

        if best is not None:
            x, y, w, h = best
            cx, cy = x + w // 2, y + h // 2
            self.get_logger().info(
                f'Blue strip detected | center: ({cx}, {cy}) | size: {w}x{h} | area: {best_area}'
            )
            self.publish_marker(cx, cy, w, h, msg.header)
            if self.get_parameter('publish_annotated').value:
                self.publish_annotated(cv_img, x, y, w, h, cx, cy, msg.header)
        else:
            self.get_logger().debug('No blue strip detected')

    def publish_marker(self, cx, cy, w, h, header):
        marker = Marker()
        marker.header = header
        marker.ns = 'blue_strip'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = w * 0.001
        marker.scale.y = h * 0.001
        marker.scale.z = 0.01
        marker.color.a = 0.6
        marker.color.b = 1.0
        marker.color.g = 0.0
        marker.color.r = 0.0
        marker.lifetime.nanosec = int(1e8)
        self.marker_pub.publish(marker)

    def publish_annotated(self, cv_img, x, y, w, h, cx, cy, header):
        annotated = cv_img.copy()
        cv2.rectangle(annotated, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.circle(annotated, (cx, cy), 4, (0, 255, 0), -1)
        cv2.putText(
            annotated,
            f'({cx}, {cy}) {w}x{h}',
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            2
        )
        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            out_msg.header = header
            self.image_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f'Publish annotated image error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BlueDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
