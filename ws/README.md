# DIP_Control — R2 上位机控制系统

ROS 2 Humble 工作区，用于 2026 ROBOCON "武林探秘" 比赛 R2 自动机器人控制。

## 硬件

- Intel RealSense D435i 深度相机
- Livox Mid-360 激光雷达 (LiDAR)
- STM32 下位机 (虚拟串口 `/dev/ttyACM0`, 115200)

## 快速开始

### 1. 安装依赖

```bash
# pyserial (串口通信)
sudo apt install python3-pip
pip3 install pyserial
```

确保已安装 ROS 2 Humble 和 RealSense SDK。Livox SDK2 需安装到 `/usr/local`。

### 2. 编译

```bash
cd /home/ek/ws
colcon build --symlink-install
```

### 3. 启动

```bash
# 一键初始化环境
source scripts/source_all.sh

# 仅激光雷达
ros2 launch d435i_camera mid360.launch.py

# 仅相机
ros2 launch d435i_camera d435i.launch.py

# 全系统启动 (相机 + 雷达 + 里程计 + 串口 + KFS检测 + 主控)
ros2 launch d435i_camera r2_full.launch.py
```

### 4. 开发调试

```bash
# 里程计打印 (终端实时显示 x,y,z 和欧拉角)
ros2 run d435i_camera odometry_printer.py

# 单独跑 FAST-LIVO2 里程计
ros2 launch fast_livo mapping_mid360.launch.py

# 相机 + 蓝检测 + 雷达 + RViz
ros2 launch d435i_camera d435i_full_with_lidar.launch.py
```

## 架构

```
src/
├── d435i_camera/          # 主控包 (生产代码)
│   ├── scripts/
│   │   ├── r2_main.py           # 比赛状态机
│   │   ├── r2_protocol.py       # 串口协议定义
│   │   ├── serial_bridge.py     # 上/下位机串口桥接
│   │   ├── kfs_detector.py      # D435i KFS 视觉检测
│   │   ├── forest_planner.py    # 树林路径规划
│   │   ├── blue_detection_node.py  # HSV 蓝检测
│   │   └── odometry_printer.py  # 里程计终端打印
│   ├── launch/                  # 启动文件
│   └── config/                  # 相机 + LiDAR 配置
├── livox_ros_driver2/     # Mid-360 驱动
├── fast_livo/             # LiDAR-inertial 里程计
├── rpg_vikit/             # FAST-LIVO2 依赖
└── ws_test_pkg/           # 空占位
```

## 比赛流程

```
INIT → 端头抓取 → 等待R1对接 → 接收II区KFS布局(红外→下位机)
→ 路径规划 → 下发路径 → 等待对接确认 → 进入II区 → 树林导航收集KFS
```

## 上下位机通信协议

帧格式: `[0xA5 0x5A CMD LEN PAYLOAD CHKSUM]`

| 方向 | CMD | 内容 | 说明 |
|------|-----|------|------|
| 上→下 | 0x01 | ODOM | 里程计 (x,y,z,roll,pitch,yaw) |
| 上→下 | 0x02 | PATH | 路径点序列 |
| 上→下 | 0x03 | KFS | KFS 检测结果 (id,x,y,z) |
| 上→下 | 0x05 | ZONE_I_PATH | I区方块导航路径 |
| 下→上 | 0x10 | ACK | 确认 |
| 下→上 | 0x12 | STATUS | 下位机状态 |
| 下→上 | 0x13 | ZONE_I_INFO | II区 KFS 布局 |
| 下→上 | 0x14 | DOCK_OK | R1对接成功 |
| 下→上 | 0x15 | GO_ZONE_I | 请求进入I区 |

协议细节见 `src/d435i_camera/scripts/r2_protocol.py`。

## LiDAR 网络配置

编辑 `src/d435i_camera/config/MID360_config.json`：

```json
{
  "host_net_info": {
    "cmd_data_ip": "192.168.1.50",
    ...
  },
  "lidar_configs": [{
    "ip": "192.168.1.121"
  }]
}
```

默认: LiDAR `192.168.1.121` → 主机 `192.168.1.50`

## FAST-LIVO2 调参

编辑 `src/fast_livo/config/mid360.yaml`，主要参数：

| 参数 | 说明 |
|------|------|
| `acc_cov` | 加速度计协方差，越小越信任IMU |
| `gyr_cov` | 陀螺仪协方差 |
| `voxel_size` | 点云降采样体素大小 |
| `point_filter_num` | 点云滤波间隔 |
