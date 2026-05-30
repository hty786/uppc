#!/usr/bin/python3
"""
航向保持 PID 实时曲线绘图工具
订阅 /r2/debug_heading_hold (Float32MultiArray) 并弹出 matplotlib 窗口
数据: [yaw_ref, yaw, err, i_term, output, yaw_rate]
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

WINDOW_SEC = 10      # 显示最近 10 秒
MAX_POINTS = 1000     # 最多保留 1000 个点
FIELD_NAMES = ['yaw_ref_deg', 'yaw_deg', 'err_deg',
               'i_term', 'output', 'yaw_rate_dps']
FIELD_COLORS = ['#2196F3', '#F44336', '#4CAF50',
                '#FF9800', '#9C27B0', '#00BCD4']


class PidPlotter(Node):
    def __init__(self):
        super().__init__('pid_plotter')
        # 每条曲线一个 deque
        self.buffers = [deque(maxlen=MAX_POINTS) for _ in range(6)]
        self.t_buffer = deque(maxlen=MAX_POINTS)
        self.t0 = None
        self.sub = self.create_subscription(
            Float32MultiArray, '/r2/debug_heading_hold', self.cb, 10)

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        self.t_buffer.append(t)
        for i in range(min(6, len(msg.data))):
            self.buffers[i].append(msg.data[i])
        # 补齐不足 6 个通道的情况
        for i in range(len(msg.data), 6):
            self.buffers[i].append(0.0)


def main():
    rclpy.init()
    plotter = PidPlotter()

    # matplotlib 初始化
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.canvas.manager.set_window_title('航向保持 PID 调试曲线')

    # 上方：角度跟踪
    lines1 = []
    for i, name in enumerate(['yaw_ref_deg', 'yaw_deg', 'err_deg']):
        line, = ax1.plot([], [], color=FIELD_COLORS[i],
                         label=name, lw=1.5 if i < 2 else 1.0,
                         linestyle='-' if i < 2 else '--')
        lines1.append(line)
    ax1.set_ylabel('角度 (deg)')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('航向跟踪')

    # 下方：PID 分量
    lines2 = []
    for i, name in enumerate(['i_term', 'output', 'yaw_rate_dps'], start=3):
        line, = ax2.plot([], [], color=FIELD_COLORS[i],
                         label=name, lw=1.5 if i == 4 else 1.0,
                         linestyle='-')
        lines2.append(line)
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('输出')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('PID 分量 / 角速度')

    def update(frame):
        if len(plotter.t_buffer) < 2:
            return lines1 + lines2

        t_arr = list(plotter.t_buffer)
        t_min = max(0, t_arr[-1] - WINDOW_SEC)
        # 截取窗口内的数据
        start = 0
        for i, t in enumerate(t_arr):
            if t >= t_min:
                start = i
                break

        tx = t_arr[start:]
        for i, line in enumerate(lines1 + lines2):
            data = list(plotter.buffers[i])[start:]
            line.set_data(tx, data)

        ax1.set_xlim(t_min, t_arr[-1] + 0.5)
        ax2.set_xlim(t_min, t_arr[-1] + 0.5)
        # 自动 Y 轴
        for ax in (ax1, ax2):
            ax.relim()
            ax.autoscale_view(scalex=False)

        fig.tight_layout()
        return lines1 + lines2

    ani = FuncAnimation(fig, update, interval=50, blit=True)

    # 在 ROS2 spin 线程中运行 GUI
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
