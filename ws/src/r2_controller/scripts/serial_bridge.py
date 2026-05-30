#!/usr/bin/python3
"""
R2 串口桥接节点
管理上位机与 STM32 下位机的双向串口通信
"""
import struct
import threading
import time
from collections import deque
from typing import List

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Float32MultiArray
from std_msgs.msg import MultiArrayDimension, MultiArrayLayout

import serial

from pathlib import Path

from r2_protocol import (
    SYNC1, SYNC2,
    UplinkCMD, DownlinkCMD, AckCode, ReqType, LowerState,
    OdomData, KfsDetect, KfsInfo, HeadingHoldDebug, NavGotoDebug,
    pack_odom, pack_path, pack_kfs, pack_zone_i_path, pack_cmd_rsp,
    pack_frame, calc_checksum, unpack_heading_hold_debug, unpack_nav_goto_debug,
)


class SerialBridge(Node):
    def __init__(self, external_ser=None):
        super().__init__('serial_bridge')
        self.declare_parameter('port', 'auto')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('odom_rate', 50.0)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.odom_period = 1.0 / self.get_parameter('odom_rate').value

        self.latest_odom = OdomData()
        self.latest_kfs: list[KfsDetect] = []
        self.downlink_queue = deque()
        self.rx_buffer = bytearray()
        self.lock = threading.Lock()
        self.running = True
        self._reset_detected = False
        self._reset_reason = ''
        self._odom_offset = None  # 首次收到的里程计作为偏移量减掉，实现归零
        self._serial_was_lost = False  # 串口断开过（用于判断 MCU 复位）

        # ROS 订阅
        self.sub_odom = self.create_subscription(
            Odometry, '/aft_mapped_to_init', self.odom_cb, 10)
        self.sub_kfs = self.create_subscription(
            String, '/r2/kfs_detect', self.kfs_cb, 10)

        # 下行发布 (下位机 → ROS)
        self.pub_lower_state = self.create_publisher(
            String, '/r2/lower_state', 10)
        self.pub_zone_i_info = self.create_publisher(
            String, '/r2/zone_i_info', 10)
        self.pub_lower_ack = self.create_publisher(
            String, '/r2/lower_ack', 10)
        self.pub_debug_pid = self.create_publisher(
            Float32MultiArray, '/r2/debug_heading_hold', 10)
        self.pub_debug_nav = self.create_publisher(
            Float32MultiArray, '/r2/debug_nav_goto', 10)

        # 串口
        self.ser = external_ser
        self.serial_ok = self.ser is not None and self.ser.is_open
        if self.serial_ok:
            self.get_logger().info(f'using serial port: {self.ser.port}')
        else:
            self.get_logger().warn('serial port not available')

        self._last_print = 0.0

        # 定时发 ODOM
        self.odom_timer = self.create_timer(self.odom_period, self.send_odom)
        # MCU 复位检测定时器
        self._reset_check_timer = self.create_timer(0.5, self._check_reset)
        # 串口监控（设备重连/切换时自动恢复）
        self._retry_timer = self.create_timer(2.0, self._retry_serial)
        # 收数据线程
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def _retry_serial(self):
        import serial as _s, os, glob
        # 检查当前串口设备文件是否还存在
        if self.ser is not None and self.serial_ok:
            if os.path.exists(self.ser.port):
                return  # 设备文件还在，正常
            # 设备文件消失 (USB 断开) — 说明 MCU 复位了
            self.get_logger().warn(f'serial device {self.ser.port} lost, reconnecting...')
            self._serial_was_lost = True  # 标记串口断开过
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.serial_ok = False
        # 扫描新设备
        ports = sorted(glob.glob('/dev/ttyACM*')) + sorted(glob.glob('/dev/ttyUSB*'))
        for p in ports:
            if not os.path.exists(p):
                continue
            try:
                self.ser = _s.Serial(p, self.baudrate, timeout=0.01)
                self.serial_ok = True
                self.get_logger().info(f'serial connected: {p}')
                # 串口断连后重连 = MCU 复位，触发系统重启
                if self._serial_was_lost and not self._reset_detected:
                    self.get_logger().info('串口重连，判定 MCU 复位，触发系统重启')
                    self._mark_reset_detected('MCU reset detected via serial reconnect')
                return
            except Exception:
                continue

    def odom_cb(self, msg: Odometry):
        with self.lock:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            q = msg.pose.pose.orientation
            roll, pitch, yaw = self._quat_to_euler(q.x, q.y, q.z, q.w)

            # 首次收到里程计：记录偏移量，实现位置归零
            if self._odom_offset is None:
                self._odom_offset = (x, y, z, yaw)
                self.get_logger().info(
                    f'里程计归零点: x={x:.3f} y={y:.3f} z={z:.3f} yaw={yaw:.1f}°')

            ox, oy, oz, oyaw = self._odom_offset
            self.latest_odom.x = x - ox
            self.latest_odom.y = y - oy
            self.latest_odom.z = z - oz
            # yaw 做角度差值并归一化
            dyaw = yaw - oyaw
            while dyaw > 180.0:
                dyaw -= 360.0
            while dyaw < -180.0:
                dyaw += 360.0
            self.latest_odom.roll = roll
            self.latest_odom.pitch = pitch
            self.latest_odom.yaw = dyaw
        now = self.get_clock().now().nanoseconds / 1e9
        if now - getattr(self, '_last_odom_print', 0) > 1.0:
            self._last_odom_print = now
            self.get_logger().info(f'[DBG] odom_cb received: x={msg.pose.pose.position.x:.3f} y={msg.pose.pose.position.y:.3f} z={msg.pose.pose.position.z:.3f}')

    def kfs_cb(self, msg: String):
        pass

    def _check_reset(self):
        """主线程回调：检测到 MCU 复位信号后触发系统重启"""
        if self._reset_detected:
            if self._reset_reason:
                self.get_logger().info(self._reset_reason)
            self.get_logger().info('MCU 复位，正在重启系统...')
            rclpy.shutdown()

    def _mark_reset_detected(self, reason: str):
        if self._reset_detected:
            return
        self._reset_reason = reason
        try:
            Path('/tmp/r2_reset').touch()
        except OSError as exc:
            self.get_logger().warn(f'Failed to write reset marker: {exc}')
        self._reset_detected = True
        self.running = False

    def send_odom(self):
        with self.lock:
            frame = pack_odom(self.latest_odom)
            odom = self.latest_odom
        if self.serial_ok:
            self._write_serial(frame)
        else:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_print > 0.5:  # 2Hz 打印
                self._last_print = now
                print(f'\033[32m[串口→STM32] x={odom.x:.3f} y={odom.y:.3f} z={odom.z:.3f} '
                      f'roll={odom.roll:.1f}° pitch={odom.pitch:.1f}° yaw={odom.yaw:.1f}°\033[0m')

    def send_path(self, waypoints: list[tuple[float, float]]):
        frame = pack_path(waypoints)
        self._write_serial(frame)
        self.get_logger().info(f'发送路径: {len(waypoints)} 个点')

    def send_kfs(self, detections: list[KfsDetect]):
        frame = pack_kfs(detections)
        self._write_serial(frame)
        self.get_logger().info(f'发送 KFS 检测: {len(detections)} 个')

    def send_zone_i_path(self, start_block: int, end_block: int, path: list[int]):
        frame = pack_zone_i_path(start_block, end_block, path)
        self._write_serial(frame)
        self.get_logger().info(f'发送 I 区路径: {start_block}→{end_block}, via {path}')

    def send_ack(self, cmd: int, code: int = AckCode.OK):
        payload = struct.pack('<BB', cmd, code)
        frame = pack_frame(UplinkCMD.CMD_RSP, payload)
        self._write_serial(frame)

    def _write_serial(self, data: bytes):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data)
            except (OSError, serial.SerialException) as exc:
                self.get_logger().warn(f'serial write error: {exc}')
                self.serial_ok = False

    def _rx_loop(self):
        while self.running and rclpy.ok():
            if self.ser and self.ser.is_open:
                try:
                    if self.ser.in_waiting:
                        data = self.ser.read(self.ser.in_waiting)
                        if len(self.rx_buffer) == 0 and data and data[0] == 0xAB:
                            self.get_logger().info('收到 MCU 复位信号 0xAB，系统将重启')
                            self._mark_reset_detected('Received MCU reset signal 0xAB; restarting system')
                            return
                        self.rx_buffer.extend(data)
                        self._parse_rx()
                except (OSError, serial.SerialException) as exc:
                    self.get_logger().warn(f'serial read error: {exc}')
                    self.serial_ok = False
                    time.sleep(1.0)
            else:
                time.sleep(0.1)

    def _parse_rx(self):
        while len(self.rx_buffer) >= 5:
            if self.rx_buffer[0] != SYNC1 or self.rx_buffer[1] != SYNC2:
                self.rx_buffer.pop(0)
                continue
            cmd = self.rx_buffer[2]
            pay_len = struct.unpack('<H', self.rx_buffer[3:5])[0]
            if len(self.rx_buffer) < 5 + pay_len + 1:
                break
            payload = bytes(self.rx_buffer[5:5 + pay_len])
            chk = self.rx_buffer[5 + pay_len]
            if chk != calc_checksum(cmd, payload):
                self.rx_buffer = self.rx_buffer[5 + pay_len + 1:]
                continue
            self.rx_buffer = self.rx_buffer[5 + pay_len + 1:]
            self._handle_downlink(cmd, payload)

    def _handle_downlink(self, cmd: int, payload: bytes):
        if cmd == DownlinkCMD.ACK:
            ack_cmd = payload[0]
            code = payload[1]
            self.get_logger().info(f'ACK: cmd={hex(ack_cmd)} code={code}')
            self.pub_lower_ack.publish(String(data=f'{ack_cmd},{code}'))
        elif cmd == DownlinkCMD.REQ:
            req = payload[0]
            self.get_logger().info(f'REQ: type={req}')
            # 回复请求由 r2_main 节点处理
            self.downlink_queue.append(('REQ', req))
        elif cmd == DownlinkCMD.STATUS:
            state = payload[0]
            self.get_logger().info(f'STATUS: {LowerState(state).name}')
            self.pub_lower_state.publish(String(data=str(state)))
            self.downlink_queue.append(('STATUS', state))
        elif cmd == DownlinkCMD.ZONE_I_INFO:
            n = payload[0]
            infos = []
            for i in range(n):
                bid = payload[1 + i * 2]
                kt = payload[1 + i * 2 + 1]
                infos.append(KfsInfo(bid, kt))
            self.get_logger().info(f'ZONE_I_INFO: {n} KFS')
            self.pub_zone_i_info.publish(String(data=f'{n}'))
            self.downlink_queue.append(('ZONE_I_INFO', infos))
        elif cmd == DownlinkCMD.DOCK_OK:
            self.get_logger().info('DOCK_OK received')
            self.downlink_queue.append(('DOCK_OK', None))
        elif cmd == DownlinkCMD.GO_ZONE_I:
            self.get_logger().info('GO_ZONE_I received')
            self.downlink_queue.append(('GO_ZONE_I', None))
        elif cmd == DownlinkCMD.DEBUG_HEADING_HOLD:
            dbg = unpack_heading_hold_debug(payload)
            msg = Float32MultiArray()
            msg.layout.dim = [MultiArrayDimension(label='pid_debug', size=6, stride=6)]
            msg.data = [dbg.yaw_ref_deg, dbg.yaw_deg, dbg.err_deg,
                        dbg.i_term, dbg.output, dbg.yaw_rate_dps]
            self.pub_debug_pid.publish(msg)
        elif cmd == DownlinkCMD.DEBUG_NAV_GOTO:
            dbg = unpack_nav_goto_debug(payload)
            msg = Float32MultiArray()
            msg.layout.dim = [MultiArrayDimension(label='nav_debug', size=6, stride=6)]
            msg.data = [dbg.ex, dbg.ey, dbg.dist, dbg.zone, dbg.vy_fwd, dbg.vw_str]
            self.pub_debug_nav.publish(msg)

    def pop_downlink(self):
        if self.downlink_queue:
            return self.downlink_queue.popleft()
        return None

    @staticmethod
    def _quat_to_euler(x, y, z, w):
        import math
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        t2 = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(t2)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def destroy(self):
        self.running = False
        if self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


class SerialBridgeStub:
    """当串口不可用时的模拟桩"""

    def __init__(self, bridge: SerialBridge):
        self.bridge = bridge

    def send_ack(self, cmd: int, code: int = AckCode.OK):
        payload = struct.pack('<BB', cmd, code)
        self.bridge._handle_downlink(DownlinkCMD.ACK, payload)

    def send_status(self, state: int):
        payload = struct.pack('<B', state)
        self.bridge._handle_downlink(DownlinkCMD.STATUS, payload)

    def send_zone_i_info(self, infos: List[KfsInfo]):
        payload = struct.pack('<B', len(infos))
        for i in infos:
            payload += struct.pack('<BB', i.block_id, i.kfs_type)
        self.bridge._handle_downlink(DownlinkCMD.ZONE_I_INFO, payload)

    def send_dock_ok(self):
        self.bridge._handle_downlink(DownlinkCMD.DOCK_OK, b'')

    def send_go_zone_i(self):
        self.bridge._handle_downlink(DownlinkCMD.GO_ZONE_I, b'')


def _init_serial():
    import serial, os, glob
    ports = ([_port] if _port != 'auto' else []) + \
        sorted(glob.glob('/dev/ttyACM*')) + sorted(glob.glob('/dev/ttyUSB*'))
    ser = None
    for p in ports:
        if not os.path.exists(p):
            continue
        try:
            ser = serial.Serial(p, _baudrate, timeout=0.01)
            try:
                ser.dtr = True
                ser.rts = True
            except Exception:
                pass
            break
        except Exception:
            continue
    return ser


def main():
    global _port, _baudrate
    import sys

    # 解析参数
    _port = 'auto'
    _baudrate = 115200
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == '-p' and i + 1 < len(args):
            _port = args[i + 1]; i += 2
        elif args[i].startswith('port:='):
            _port = args[i].split(':=', 1)[1]; i += 1
        elif args[i] == '-p' or args[i] == '--port':
            if i + 1 >= len(args): i += 1; continue
            _port = args[i + 1]; i += 2
        else:
            i += 1

    # 在 rclpy.init 之前开串口
    print(f'[serial_bridge] opening serial port (port={_port}, baud={_baudrate})...', flush=True)
    _ser = _init_serial()
    if _ser is not None:
        print(f'[serial_bridge] serial port opened: {_ser.port}', flush=True)

    rclpy.init()
    node = SerialBridge(_ser)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
