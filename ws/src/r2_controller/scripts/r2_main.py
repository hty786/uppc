#!/usr/bin/python3
"""
R2 主控制节点 — 比赛流程状态机
协调串口通信、KFS 检测、路径规划，驱动 R2 完成比赛任务
"""
import time
from enum import IntEnum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

from r2_protocol import KfsDetect, KfsInfo, pack_odom, pack_path, pack_kfs, pack_zone_i_path, pack_cmd_rsp, UplinkCMD
from forest_planner import ForestPlanner


class Phase(IntEnum):
    INIT = 0
    MOVE_TO_ENDTIPS = 1
    GRAB_ENDTIPS = 2
    WAIT_DOCK = 3
    RECV_ZONE_I_INFO = 4
    PLAN_PATH = 5
    SEND_PATH = 6
    WAIT_DOCK_OK = 7
    ENTER_ZONE_I = 8
    NAVIGATE_FOREST = 9
    RETURN = 10
    IDLE = 11


class R2Main(Node):
    def __init__(self):
        super().__init__('r2_main')
        self.phase = Phase.INIT
        self.planner = ForestPlanner()
        self.zone_i_kfs: list[KfsInfo] = []
        self.forest_path: list[int] = []
        self.start_time = None
        self.dock_ok_received = False
        self.go_zone_i_requested = False

        # KFS 检测结果
        self.latest_kfs_detect: list = []

        # 下位机状态
        self.lower_state = 0

        self.sub_kfs_detect = self.create_subscription(
            Float32MultiArray, '/r2/kfs_result', self.kfs_detect_cb, 10)
        self.sub_lower_state = self.create_subscription(
            String, '/r2/lower_state', self.lower_state_cb, 10)
        self.sub_lower_ack = self.create_subscription(
            String, '/r2/lower_ack', self.lack_ack_cb, 10)
        self.sub_zone_i_info = self.create_subscription(
            String, '/r2/zone_i_info', self.zone_i_info_cb, 10)

        self.timer = self.create_timer(0.05, self.state_machine_loop)
        self.get_logger().info('R2 main controller started')

    def kfs_detect_cb(self, msg: Float32MultiArray):
        self.latest_kfs_detect = msg.data

    def lower_state_cb(self, msg: String):
        try:
            self.lower_state = int(msg.data)
        except ValueError:
            pass

    def lack_ack_cb(self, msg: String):
        pass

    def zone_i_info_cb(self, msg: String):
        self.get_logger().info(f'收到 I 区信息: {msg.data}')

    def state_machine_loop(self):
        if self.phase == Phase.INIT:
            self.get_logger().info('[STATE] INIT → 等待下位机就绪')
            self.phase = Phase.MOVE_TO_ENDTIPS

        elif self.phase == Phase.MOVE_TO_ENDTIPS:
            """比赛开始，下位机根据上位机里程计运动到端头位置"""
            if self.lower_state == 2:
                self.get_logger().info('[STATE] 已到达端头区')
                self.phase = Phase.GRAB_ENDTIPS

        elif self.phase == Phase.GRAB_ENDTIPS:
            """下位机抓取端头后原地等待 R1 对接"""
            if self.lower_state == 4:
                self.get_logger().info('[STATE] 端头抓取完成，等待 R1 对接')
                self.phase = Phase.WAIT_DOCK

        elif self.phase == Phase.WAIT_DOCK:
            """等待 R1 红外通信传来 II 区 KFS 布局"""
            # 下位机会通过 ZONE_I_INFO (CMD 0x13) 上传 II 区信息
            pass

        elif self.phase == Phase.RECV_ZONE_I_INFO:
            """接收到 II 区 KFS 布局，进行路径规划"""
            if self.zone_i_kfs:
                self.phase = Phase.PLAN_PATH

        elif self.phase == Phase.PLAN_PATH:
            """根据 II 区 KFS 布局规划树林路径"""
            target_blocks = [info.block_id for info in self.zone_i_kfs
                             if info.kfs_type == 2]
            if target_blocks:
                self.forest_path = self.planner.plan_route(
                    self.planner.ENTRANCE_BLOCKS[0], target_blocks)
                self.get_logger().info(f'路径规划完成: {self.forest_path}')
            self.phase = Phase.SEND_PATH

        elif self.phase == Phase.SEND_PATH:
            """发送路径到下位机"""
            self.get_logger().info(f'下发路径: {self.forest_path}')
            # 路径通过 serial_bridge 的 pub 发布，或直接通过 service 调用
            self.phase = Phase.WAIT_DOCK_OK

        elif self.phase == Phase.WAIT_DOCK_OK:
            """等待 R1 红外通知武器对接成功"""
            if self.dock_ok_received:
                self.get_logger().info('[STATE] 对接成功，准备进入 II 区')
                self.phase = Phase.ENTER_ZONE_I

        elif self.phase == Phase.ENTER_ZONE_I:
            """跟随 R1 进入 II 区，等待下位机发出导航请求"""
            if self.go_zone_i_requested:
                self.phase = Phase.NAVIGATE_FOREST

        elif self.phase == Phase.NAVIGATE_FOREST:
            """在 II 区树林中自主导航收集 KFS"""
            # 上位机实时发 KFS 深度和位置信息给下位机
            if self.lower_state == 4:
                self.get_logger().info('[STATE] 树林导航完成')
                self.phase = Phase.IDLE

        elif self.phase == Phase.IDLE:
            pass


def main():
    rclpy.init()
    node = R2Main()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
