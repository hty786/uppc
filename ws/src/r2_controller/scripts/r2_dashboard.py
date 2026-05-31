#!/usr/bin/python3
"""
R2 实时调试仪表盘
单窗口四面板，pyqtgraph + PyQt5，暗色主题
订阅:
  /r2/debug_nav_goto     (Float32MultiArray) [ex, ey, dist, zone, vy_fwd, vw_str] 50Hz
  /r2/debug_heading_hold (Float32MultiArray) [yaw_ref_deg, yaw_deg, err_deg, i_term, output, yaw_rate_dps] 50Hz
"""

import sys
import os

# ── 无显示环境时优雅退出 ──────────────────────────────────────────────
if 'DISPLAY' not in os.environ and 'WAYLAND_DISPLAY' not in os.environ:
    print("[r2_dashboard] No display available, exiting gracefully.")
    sys.exit(0)

import threading
from collections import deque

import numpy as np

# ── Qt ─────────────────────────────────────────────────────────────────
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QLabel, QHBoxLayout, QFrame)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QColor

import pyqtgraph as pg

# ── ROS2 ────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# ═══════════════════════════════════════════════════════════════════════════
# 常量
# ═══════════════════════════════════════════════════════════════════════════
WINDOW_SEC = 15.0
RATE_HZ = 50
MAX_POINTS = int(WINDOW_SEC * RATE_HZ * 2)  # 1500
TIMER_MS = 50

BG_COLOR = '#1a1a1a'
GRID_COLOR = '#333333'
TEXT_COLOR = '#cccccc'
TITLE_COLOR = '#ffffff'
PANEL_BORDER = '#444444'

# ═══════════════════════════════════════════════════════════════════════════
# ROS2 节点
# ═══════════════════════════════════════════════════════════════════════════


class DashboardNode(Node):
    """订阅两个 debug topic，数据存入 deque"""

    def __init__(self):
        super().__init__('r2_dashboard')
        self.lock = threading.Lock()
        self.t0 = None

        # 导航 debug
        self.nav_t = deque(maxlen=MAX_POINTS)
        self.nav = [deque(maxlen=MAX_POINTS) for _ in range(6)]

        # 航向 debug
        self.hdg_t = deque(maxlen=MAX_POINTS)
        self.hdg = [deque(maxlen=MAX_POINTS) for _ in range(6)]

        self.sub_nav = self.create_subscription(
            Float32MultiArray, '/r2/debug_nav_goto', self.nav_cb, 10)
        self.sub_hdg = self.create_subscription(
            Float32MultiArray, '/r2/debug_heading_hold', self.heading_cb, 10)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def nav_cb(self, msg: Float32MultiArray):
        t = self._now()
        with self.lock:
            if self.t0 is None:
                self.t0 = t
            self.nav_t.append(t - self.t0)
            data = msg.data
            for i in range(6):
                self.nav[i].append(data[i] if i < len(data) else 0.0)

    def heading_cb(self, msg: Float32MultiArray):
        t = self._now()
        with self.lock:
            if self.t0 is None:
                self.t0 = t
            self.hdg_t.append(t - self.t0)
            data = msg.data
            for i in range(6):
                self.hdg[i].append(data[i] if i < len(data) else 0.0)


# ═══════════════════════════════════════════════════════════════════════════
# 单个面板 — pyqtgraph PlotItem 封装
# ═══════════════════════════════════════════════════════════════════════════


class DashboardPanel:
    """一个带标题的画板，管理若干曲线"""

    def __init__(self, plot_item: pg.PlotItem, title: str,
                 curve_defs: list[tuple[str, str, str]]):
        """
        curve_defs: [(label, color, linestyle), ...]
           linestyle: '-' 实线, '--' 虚线, ':' 点线
        """
        self.plot = plot_item
        self.curves = []
        self.hlines = []  # 水平参考线

        # 标题
        self.plot.setTitle(title, color=TITLE_COLOR, size='11pt')
        self.plot.showGrid(x=True, y=True, alpha=0.25)

        # 背景
        self.plot.setLabel('left', '')
        self.plot.getAxis('left').setPen(pg.mkPen(color=TEXT_COLOR, width=1))
        self.plot.getAxis('left').setTextPen(pg.mkPen(color=TEXT_COLOR))
        self.plot.getAxis('bottom').setPen(pg.mkPen(color=TEXT_COLOR, width=1))
        self.plot.getAxis('bottom').setTextPen(pg.mkPen(color=TEXT_COLOR))

        # 图例
        self.plot.addLegend(offset=(-5, 5),
                            labelTextColor=TEXT_COLOR,
                            brush=pg.mkBrush(BG_COLOR + 'cc'),
                            pen=pg.mkPen(color=PANEL_BORDER, width=1))

        # 创建曲线
        for label, color, ls in curve_defs:
            pen = pg.mkPen(color=color, width=1.5,
                           style=self._linestyle(ls))
            curve = self.plot.plot([], [], pen=pen, name=label)
            self.curves.append(curve)

    @staticmethod
    def _linestyle(ls: str):
        if ls == '--':
            return Qt.DashLine
        elif ls == ':':
            return Qt.DotLine
        return Qt.SolidLine

    def add_hline(self, y: float, color: str, label: str = None):
        """添加水平参考线"""
        pen = pg.mkPen(color=color, width=1, style=Qt.DashLine)
        line = pg.InfiniteLine(pos=y, angle=0, pen=pen, label=label,
                               labelOpts={'color': color, 'position': 0.02})
        self.plot.addItem(line)
        self.hlines.append(line)
        return line

    def update(self, t_arr, data_buffers, t_min: float, t_max: float):
        """用新数据刷新所有曲线"""
        if len(t_arr) < 2:
            return

        # 裁剪到窗口内
        start = 0
        for i, tv in enumerate(t_arr):
            if tv >= t_min:
                start = i
                break
        tx = t_arr[start:]

        for i, curve in enumerate(self.curves):
            y_arr = list(data_buffers[i])[start:]
            curve.setData(tx, y_arr)

        self.plot.setXRange(t_min, t_max, padding=0.0)
        self.plot.enableAutoRange(axis='y')


# ═══════════════════════════════════════════════════════════════════════════
# 主窗口
# ═══════════════════════════════════════════════════════════════════════════


class DashboardWindow(QMainWindow):
    def __init__(self, node: DashboardNode):
        super().__init__()
        self.node = node
        self.setWindowTitle('R2 实时调试面板')
        self.resize(1400, 950)
        self._setup_ui()
        self._setup_timer()

    def _setup_ui(self):
        # 全局样式
        self.setStyleSheet(f"""
            QMainWindow {{
                background-color: {BG_COLOR};
            }}
            QLabel#header {{
                color: {TITLE_COLOR};
                font-size: 16px;
                font-weight: bold;
                padding: 6px 12px;
            }}
            QLabel#status {{
                color: #888888;
                font-size: 11px;
                padding: 2px 12px;
            }}
            QFrame#separator {{
                background-color: {PANEL_BORDER};
                max-height: 1px;
            }}
        """)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(2)

        # 标题行
        hdr = QHBoxLayout()
        title_lbl = QLabel('R2 实时调试面板')
        title_lbl.setObjectName('header')
        hdr.addWidget(title_lbl)
        hdr.addStretch()
        self.status_lbl = QLabel('等待数据...')
        self.status_lbl.setObjectName('status')
        hdr.addWidget(self.status_lbl)
        layout.addLayout(hdr)

        # 分隔线
        sep = QFrame()
        sep.setObjectName('separator')
        layout.addWidget(sep)

        # ── pyqtgraph 画布 ──
        pw = pg.GraphicsLayoutWidget()
        pw.setBackground(BG_COLOR)

        # 共享 X 轴：使用同一个 ViewBox 的 X 链接
        # 创建四个 PlotItem，链接 X 轴
        self.p1_plot = pw.addPlot(row=0, col=0)
        self.p2_plot = pw.addPlot(row=1, col=0)
        self.p3_plot = pw.addPlot(row=2, col=0)
        self.p4_plot = pw.addPlot(row=3, col=0)

        # 隐藏前三张图的 X 轴标签
        for p in [self.p1_plot, self.p2_plot, self.p3_plot]:
            p.getAxis('bottom').setStyle(showValues=False)
        self.p4_plot.setLabel('bottom', '时间 (s)', color=TEXT_COLOR)

        # 链接 X 轴
        self.p2_plot.setXLink(self.p1_plot)
        self.p3_plot.setXLink(self.p1_plot)
        self.p4_plot.setXLink(self.p1_plot)

        # ── 面板 1: 导航位置误差 ──
        self.panel1 = DashboardPanel(self.p1_plot, '面板1: 导航位置误差', [
            ('ex (X误差)', '#F44336', '-'),
            ('ey (Y误差)', '#2196F3', '-'),
            ('dist (距离)', '#4CAF50', '--'),
        ])
        self.p1_plot.setLabel('left', '距离 (m)', color=TEXT_COLOR)
        self.panel1.add_hline(0.02, '#888888', '到位阈值 0.02m')

        # ── 面板 2: 底盘速度指令 + 远近场 ──
        self.panel2 = DashboardPanel(self.p2_plot, '面板2: 底盘速度指令 + 远近场', [
            ('vy_fwd (前后)', '#FF9800', '-'),
            ('vw_str (左右)', '#9C27B0', '-'),
            ('zone (远近场)', '#00BCD4', ':'),
        ])
        self.p2_plot.setLabel('left', '速度 / 标志', color=TEXT_COLOR)

        # ── 面板 3: 航向跟踪 ──
        self.panel3 = DashboardPanel(self.p3_plot, '面板3: 航向跟踪', [
            ('yaw_ref (参考)', '#2196F3', '-'),
            ('yaw (当前)', '#F44336', '-'),
            ('err (误差)', '#4CAF50', '--'),
        ])
        self.p3_plot.setLabel('left', '角度 (deg)', color=TEXT_COLOR)

        # ── 面板 4: PID 分量 ──
        self.panel4 = DashboardPanel(self.p4_plot, '面板4: PID 分量', [
            ('i_term (积分)', '#FF9800', '-'),
            ('output (输出)', '#9C27B0', '-'),
            ('yaw_rate (角速度)', '#00BCD4', '-'),
        ])
        self.p4_plot.setLabel('left', '输出 / 角速度', color=TEXT_COLOR)

        layout.addWidget(pw, stretch=1)

    def _setup_timer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self._refresh)
        self.timer.start(TIMER_MS)

    def _get_arrays(self, t_deque, buffers):
        """线程安全地拷贝数据"""
        with self.node.lock:
            t_arr = np.array(t_deque, dtype=float)
            data = [np.array(b, dtype=float) for b in buffers]
        return t_arr, data

    def _refresh(self):
        node = self.node
        with node.lock:
            if node.t0 is None:
                return
            nav_t_ok = len(node.nav_t) >= 2
            hdg_t_ok = len(node.hdg_t) >= 2
            nav_t_arr = np.array(node.nav_t, dtype=float)
            hdg_t_arr = np.array(node.hdg_t, dtype=float)
            nav_data = [np.array(b, dtype=float) for b in node.nav]
            hdg_data = [np.array(b, dtype=float) for b in node.hdg]
            nav_t_max = float(nav_t_arr[-1]) if nav_t_ok else 0.0
            hdg_t_max = float(hdg_t_arr[-1]) if hdg_t_ok else 0.0

        t_max = max(nav_t_max, hdg_t_max)
        t_min = max(0.0, t_max - WINDOW_SEC)

        # 更新导航面板 (面板1、2)
        if nav_t_ok:
            self.panel1.update(nav_t_arr, [nav_data[0], nav_data[1], nav_data[2]],
                               t_min, t_max)
            self.panel2.update(nav_t_arr, [nav_data[4], nav_data[5], nav_data[3]],
                               t_min, t_max)

        # 更新航向面板 (面板3、4)
        if hdg_t_ok:
            self.panel3.update(hdg_t_arr, [hdg_data[0], hdg_data[1], hdg_data[2]],
                               t_min, t_max)
            self.panel4.update(hdg_t_arr, [hdg_data[3], hdg_data[4], hdg_data[5]],
                               t_min, t_max)

        # 状态栏
        ns = len(nav_t_arr) if nav_t_ok else 0
        hs = len(hdg_t_arr) if hdg_t_ok else 0
        self.status_lbl.setText(
            f'数据点: nav={ns}  hdg={hs}  |  窗口: [{t_min:.1f}s, {t_max:.1f}s]  |  '
            f'刷新: {TIMER_MS}ms')


# ═══════════════════════════════════════════════════════════════════════════
# 入口
# ═══════════════════════════════════════════════════════════════════════════


def main():
    # 初始化 ROS2
    rclpy.init(args=[sys.argv[0]])
    node = DashboardNode()

    # 启动 ROS spin 线程
    spin_thread = threading.Thread(target=_spin, args=(node,), daemon=True)
    spin_thread.start()

    # 启动 Qt 应用
    pg.setConfigOptions(antialias=True, background=BG_COLOR,
                        foreground=TEXT_COLOR)
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    # 全局暗色调色板
    dark_palette = app.palette()
    dark_palette.setColor(dark_palette.Window, QColor(BG_COLOR))
    dark_palette.setColor(dark_palette.WindowText, QColor(TEXT_COLOR))
    dark_palette.setColor(dark_palette.Base, QColor('#2a2a2a'))
    dark_palette.setColor(dark_palette.AlternateBase, QColor('#333333'))
    dark_palette.setColor(dark_palette.ToolTipBase, QColor('#2a2a2a'))
    dark_palette.setColor(dark_palette.ToolTipText, QColor(TEXT_COLOR))
    dark_palette.setColor(dark_palette.Text, QColor(TEXT_COLOR))
    dark_palette.setColor(dark_palette.Button, QColor('#333333'))
    dark_palette.setColor(dark_palette.ButtonText, QColor(TEXT_COLOR))
    dark_palette.setColor(dark_palette.BrightText, QColor('#ff4444'))
    dark_palette.setColor(dark_palette.Highlight, QColor('#2196F3'))
    dark_palette.setColor(dark_palette.HighlightedText, QColor('#ffffff'))
    app.setPalette(dark_palette)

    window = DashboardWindow(node)
    window.show()

    try:
        app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _spin(node: DashboardNode):
    """ROS2 spin 在独立线程运行"""
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
    except (KeyboardInterrupt, SystemExit):
        pass


if __name__ == '__main__':
    main()
