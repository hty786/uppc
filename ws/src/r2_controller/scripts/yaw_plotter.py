#!/usr/bin/python3
"""
雷达 yaw 实时曲线绘图工具
订阅 /aft_mapped_to_init 并弹出 matplotlib 窗口
"""
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import sys
import os

if 'DISPLAY' not in os.environ and 'WAYLAND_DISPLAY' not in os.environ:
    sys.exit(0)

WINDOW_SEC = 20
MAX_POINTS = 2000


class YawPlotter(Node):
    def __init__(self):
        super().__init__('yaw_plotter')
        self.t_buffer = deque(maxlen=MAX_POINTS)
        self.yaw_buffer = deque(maxlen=MAX_POINTS)
        self.t0 = None
        self.sub = self.create_subscription(
            Odometry, '/aft_mapped_to_init', self.cb, 10)

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        q = msg.pose.pose.orientation
        yaw = math.degrees(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        self.t_buffer.append(t)
        self.yaw_buffer.append(yaw)


def main():
    rclpy.init()
    plotter = YawPlotter()

    fig, ax = plt.subplots(1, 1, figsize=(12, 5))
    fig.canvas.manager.set_window_title('雷达 yaw 实时曲线')
    line, = ax.plot([], [], color='#2196F3', label='yaw (deg)', lw=1.5)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('yaw (deg)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_title('Radar Yaw /aft_mapped_to_init')
    ax.set_ylim(-180, 180)

    def update(frame):
        if len(plotter.t_buffer) < 2:
            return [line]
        t_arr = list(plotter.t_buffer)
        t_min = max(0, t_arr[-1] - WINDOW_SEC)
        start = next((i for i, t in enumerate(t_arr) if t >= t_min), 0)
        tx = t_arr[start:]
        yaw = list(plotter.yaw_buffer)[start:]
        line.set_data(tx, yaw)
        ax.set_xlim(t_min, t_arr[-1] + 0.5)
        ax.relim()
        ax.autoscale_view(scalex=False)
        fig.tight_layout()
        return [line]

    ani = FuncAnimation(fig, update, interval=50, blit=True)

    import threading
    def spin():
        while rclpy.ok():
            rclpy.spin_once(plotter, timeout_sec=0.01)
    t = threading.Thread(target=spin, daemon=True)
    t.start()

    plt.show()
    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
