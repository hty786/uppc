#!/usr/bin/python3
"""
导航到点 PI(D) 实时曲线调试工具
订阅 /r2/debug_nav_goto (Float32MultiArray) 并弹出 matplotlib 窗口
数据: [ex, ey, dist, zone, vy_fwd, vw_str]
"""
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys

# 无显示环境时跳过
import os
if 'DISPLAY' not in os.environ and 'WAYLAND_DISPLAY' not in os.environ:
    sys.exit(0)

WINDOW_SEC = 15
MAX_POINTS = 1500


class NavPlotter(Node):
    def __init__(self):
        super().__init__('nav_plotter')
        self.buffers = [deque(maxlen=MAX_POINTS) for _ in range(6)]
        self.t_buffer = deque(maxlen=MAX_POINTS)
        self.t0 = None
        self.sub = self.create_subscription(
            Float32MultiArray, '/r2/debug_nav_goto', self.cb, 10)

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        self.t_buffer.append(t)
        for i in range(min(6, len(msg.data))):
            self.buffers[i].append(msg.data[i])
        for i in range(len(msg.data), 6):
            self.buffers[i].append(0.0)


def main():
    rclpy.init()
    plotter = NavPlotter()

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.canvas.manager.set_window_title('导航到点调试曲线')

    # 上图：位置误差与距离
    l1, = ax1.plot([], [], color='#F44336', label='ex (X误差 m)', lw=1.5)
    l2, = ax1.plot([], [], color='#2196F3', label='ey (Y误差 m)', lw=1.5)
    l3, = ax1.plot([], [], color='#4CAF50', label='dist (距离 m)', lw=1.5, linestyle='--')
    ax1.set_ylabel('距离 (m)')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('位置误差 & 距离 — 到达绿线以下即到位')

    # 判断到位线
    ax1.axhline(y=0.02, color='gray', linestyle=':', alpha=0.5, label='到位阈值(0.02m)')

    # 下图：速度输出 + 远近场
    l4, = ax2.plot([], [], color='#FF9800', label='vy_fwd (前后)', lw=1.5)
    l5, = ax2.plot([], [], color='#9C27B0', label='vw_str (左右)', lw=1.5)
    l6, = ax2.plot([], [], color='#00BCD4', label='zone (0=远 1=近)', lw=1.0, linestyle=':', alpha=0.7)
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('输出')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('底盘速度输出 & 远/近场区')

    lines = [l1, l2, l3, l4, l5, l6]

    def update(frame):
        if len(plotter.t_buffer) < 2:
            return lines
        t_arr = list(plotter.t_buffer)
        t_min = max(0, t_arr[-1] - WINDOW_SEC)
        start = next((i for i, t in enumerate(t_arr) if t >= t_min), 0)
        tx = t_arr[start:]
        for i, line in enumerate(lines):
            data = list(plotter.buffers[i])[start:]
            line.set_data(tx, data)
        ax1.set_xlim(t_min, t_arr[-1] + 0.5)
        ax2.set_xlim(t_min, t_arr[-1] + 0.5)
        for ax in (ax1, ax2):
            ax.relim()
            ax.autoscale_view(scalex=False)
        fig.tight_layout()
        return lines

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
