#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math


def quat_to_euler(x, y, z, w):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class OdometryPrinter(Node):
    def __init__(self):
        super().__init__('odometry_printer')
        self.sub_odom = self.create_subscription(
            Odometry, '/aft_mapped_to_init', self.odom_callback, 10)
        self.sub_imu = self.create_subscription(
            Imu, '/livox/imu', self.imu_callback, 10)
        self.get_logger().info('Odometry printer started')

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        print(f'\033[36m[里程计] x={p.x:.3f}  y={p.y:.3f}  z={p.z:.3f}  '
              f'roll={roll:.1f}°  pitch={pitch:.1f}°  yaw={yaw:.1f}°\033[0m')

    def imu_callback(self, msg):
        g = msg.angular_velocity
        print(f'\033[33m[陀螺仪] gx={g.x:.2f}  gy={g.y:.2f}  gz={g.z:.2f} rad/s\033[0m')


def main():
    rclpy.init()
    node = OdometryPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
