#!/usr/bin/python3
"""
R2 上位机 ↔ 下位机 STM32 串口通信协议

帧格式 (定长二进制包):
  [SYNC1 SYNC2 CMD LEN PAYLOAD... CHKSUM]
  - SYNC1: 0xA5 (帧头1)
  - SYNC2: 0x5A (帧头2)
  - CMD:   1 字节命令码
  - LEN:   2 字节负载长度 (小端)
  - PAYLOAD: LEN 字节数据
  - CHKSUM: 1 字节异或校验 (CMD^LEN_LO^LEN_HI^PAYLOAD 逐字节)

上位机 → 下位机命令:
  CMD 0x01  ODOM    里程计定位数据
                     Payload: float x(4) y(4) z(4) roll(4) pitch(4) yaw(4) = 24B
  CMD 0x02  PATH    路径规划点序列
                     Payload: uint8 n, [float x y]*n
  CMD 0x03  KFS     KFS 检测结果
                     Payload: uint8 n, [uint8 id, float x y z]*n
  CMD 0x04  CMD_RSP 上位机指令响应 (对 REQ 的回复)
                     Payload: uint8 req_id, uint8 result, 可变数据
  CMD 0x05  ZONE_I_PATH  I 区目标路径 (树林导航)
                     Payload: uint8 block_start, uint8 block_end, uint8 n_waypoints, [uint8 block_id]*n

下位机 → 上位机命令:
  CMD 0x10  ACK     通用确认
                     Payload: uint8 cmd, uint8 code (0=OK, 1=ERROR)
  CMD 0x11  REQ     请求上位机数据
                     Payload: uint8 req_type (1=请求 ODOM, 2=请求 KFS, 3=请求 PATH)
  CMD 0x12  STATUS  下位机状态上报
                     Payload: uint8 state (0=IDLE, 1=MOVING, 2=AT_TARGET, 3=GRABBING, 4=DONE, 5=ERROR)
  CMD 0x13  ZONE_I_INFO  R1 转发的 I 区 KFS 布局 (红外来)
                     Payload: uint8 n_kfs, [uint8 block_id, uint8 kfs_type]*n
  CMD 0x14  DOCK_OK  R1 对接武器成功
                     Payload: 空
  CMD 0x15  GO_ZONE_I  下位机准备好进入 I 区，请求开始导航
                     Payload: 空

比赛状态机:
  IDLE → START → MOVE_TO_ENDTIPS → GRAB_ENDTIPS → WAIT_DOCK → 
  RECV_ZONE_I_INFO → PLAN_PATH → SEND_PATH → WAIT_DOCK_OK → 
  ENTER_ZONE_I → NAVIGATE_FOREST → DONE
"""

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import List, Optional, Tuple


SYNC1 = 0xA5
SYNC2 = 0x5A


class UplinkCMD(IntEnum):
    """上位机 → 下位机"""
    ODOM = 0x01
    PATH = 0x02
    KFS = 0x03
    CMD_RSP = 0x04
    ZONE_I_PATH = 0x05


class DownlinkCMD(IntEnum):
    """下位机 → 上位机"""
    ACK = 0x10
    REQ = 0x11
    STATUS = 0x12
    ZONE_I_INFO = 0x13
    DOCK_OK = 0x14
    GO_ZONE_I = 0x15
    DEBUG_HEADING_HOLD = 0x20  # 航向保持 PID 调试数据
    DEBUG_NAV_GOTO = 0x21      # 导航到点调试数据


class AckCode(IntEnum):
    OK = 0x00
    ERROR = 0x01


class ReqType(IntEnum):
    REQ_ODOM = 1
    REQ_KFS = 2
    REQ_PATH = 3


class LowerState(IntEnum):
    IDLE = 0
    MOVING = 1
    AT_TARGET = 2
    GRABBING = 3
    DONE = 4
    ERROR = 5
    WAIT_CMD = 6
    ESTOP_CH6_MAX = 0x08  # 急停 + CH6最大 → 上位机关机


@dataclass
class OdomData:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class HeadingHoldDebug:
    """航向保持 PID 调试数据"""
    yaw_ref_deg: float = 0.0
    yaw_deg: float = 0.0
    err_deg: float = 0.0
    i_term: float = 0.0
    output: float = 0.0
    yaw_rate_dps: float = 0.0


@dataclass
class NavGotoDebug:
    """导航到点调试数据"""
    ex: float = 0.0       # X方向位置误差 (m)
    ey: float = 0.0       # Y方向位置误差 (m)
    dist: float = 0.0     # 到目标距离 (m)
    zone: float = 0.0     # 0=远场, 1=近场
    vy_fwd: float = 0.0   # 前后速度输出
    vw_str: float = 0.0   # 左右速度输出


@dataclass
class KfsInfo:
    block_id: int = 0
    kfs_type: int = 0


@dataclass
class KfsDetect:
    kfs_id: int = 0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


def calc_checksum(cmd: int, payload: bytes) -> int:
    chk = cmd
    chk ^= (len(payload) & 0xFF)
    chk ^= ((len(payload) >> 8) & 0xFF)
    for b in payload:
        chk ^= b
    return chk & 0xFF


def pack_frame(cmd: int, payload: bytes) -> bytes:
    chk = calc_checksum(cmd, payload)
    frame = struct.pack('<BBBH', SYNC1, SYNC2, cmd, len(payload))
    frame += payload
    frame += struct.pack('B', chk)
    return frame


def pack_odom(data: OdomData) -> bytes:
    payload = struct.pack('<6f', data.x, data.y, data.z, data.roll, data.pitch, data.yaw)
    return pack_frame(UplinkCMD.ODOM, payload)


def pack_path(waypoints: List[Tuple[float, float]]) -> bytes:
    payload = struct.pack('<B', len(waypoints))
    for x, y in waypoints:
        payload += struct.pack('<2f', x, y)
    return pack_frame(UplinkCMD.PATH, payload)


def pack_kfs(detections: List[KfsDetect]) -> bytes:
    payload = struct.pack('<B', len(detections))
    for d in detections:
        payload += struct.pack('<B3f', d.kfs_id, d.x, d.y, d.z)
    return pack_frame(UplinkCMD.KFS, payload)


def pack_zone_i_path(start_block: int, end_block: int, path_blocks: List[int]) -> bytes:
    payload = struct.pack('<BBB', start_block, end_block, len(path_blocks))
    for bid in path_blocks:
        payload += struct.pack('<B', bid)
    return pack_frame(UplinkCMD.ZONE_I_PATH, payload)


def pack_cmd_rsp(req_id: int, result: int) -> bytes:
    payload = struct.pack('<BB', req_id, result)
    return pack_frame(UplinkCMD.CMD_RSP, payload)


def unpack_heading_hold_debug(payload: bytes) -> HeadingHoldDebug:
    """解包航向保持 PID 调试数据 (6 float = 24B)"""
    vals = struct.unpack('<6f', payload[:24])
    return HeadingHoldDebug(*vals)


def unpack_nav_goto_debug(payload: bytes) -> NavGotoDebug:
    """解包导航到点调试数据 (6 float = 24B)"""
    vals = struct.unpack('<6f', payload[:24])
    return NavGotoDebug(*vals)
