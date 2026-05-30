# AGENTS.md — ROS 2 Workspace (ROBOCON R2)

## Build & env

```bash
colcon build --symlink-install
colcon build --packages-select d435i_camera --symlink-install
colcon build --packages-select r2_controller --symlink-install
source /opt/ros/humble/setup.bash && source install/setup.bash   # or: source scripts/source_all.sh
```

- `Python3_EXECUTABLE` pinned to `/usr/bin/python3` in `.colcon/defaults.yaml` (avoids Conda Python)
- External deps: `realsense2_camera` at `/home/ek/realsense-ros/install`, OpenCV (`cv_bridge`), pyserial
- No tests, no CI, no pre-commit, no lint/typecheck commands

## Package layout

| Package | Role |
|---------|------|
| `d435i_camera` | Camera perception: `blue_detection_node.py`, `kfs_detector.py`. Holds all launch files. |
| `r2_controller` | **R2 main control**: `r2_main.py` (state machine), `serial_bridge.py`, `r2_protocol.py`, `forest_planner.py`, `odometry_printer.py` |
| `fast_livo` | LiDAR-inertial odometry (FAST-LIVO2) |
| `livox_ros_driver2` | Mid-360 LiDAR driver |
| `rpg_vikit` + `ws_test_pkg` | FAST-LIVO2 dep / empty placeholder |

Real entrypoints in `r2_controller/scripts/` (all Python, installed as executables via CMakeLists.txt) and `d435i_camera/scripts/`. Launch files are in `d435i_camera/launch/`.

## Launch

```bash
ros2 launch d435i_camera r2_full.launch.py          # camera + LiDAR + odometry + serial + KFS + main state machine
ros2 launch d435i_camera d435i_full_with_lidar.launch.py  # camera + blue detection + LiDAR + odometry + RViz
ros2 launch d435i_camera d435i_full.launch.py        # camera + blue detection + RViz
ros2 launch d435i_camera d435i.launch.py             # camera only
ros2 launch d435i_camera mid360.launch.py            # LiDAR only
```

## Serial protocol (STM32)

- Port: `/dev/ttyUSB0` (r2_full.launch.py), 115200 baud
- Frame: `[0xA5 0x5A CMD LEN(2B LE) PAYLOAD... CHKSUM(1B XOR)]`
- Defined in `r2_controller/scripts/r2_protocol.py`
- Upstream CMDs: 0x01 ODOM, 0x02 PATH, 0x03 KFS, 0x05 ZONE_I_PATH
- Downstream CMDs: 0x10 ACK, 0x12 STATUS, 0x13 ZONE_I_INFO, 0x14 DOCK_OK, 0x15 GO_ZONE_I

## Key topics

| Topic | Direction | Type |
|-------|-----------|------|
| `/Odometry` | Published by fast_livo | `nav_msgs/Odometry` |
| `/r2/kfs_result` | Sub by r2_main | `Float32MultiArray` |
| `/r2/lower_state` | Sub by r2_main | `String` |
| `/r2/lower_ack` | Sub by r2_main | `String` |
| `/r2/zone_i_info` | Sub by r2_main | `String` |

## Mid-360 LiDAR

- Host `192.168.1.50` → LiDAR `192.168.1.121`, config in `src/d435i_camera/config/MID360_config.json`
- FAST-LIVO2 tuning: `src/fast_livo/config/mid360.yaml` (`acc_cov`, `gyr_cov`, `voxel_size`, `point_filter_num`)

## Odometry debug

```bash
# Terminal 1: LiDAR
ros2 launch d435i_camera mid360.launch.py
# Terminal 2: odometry + print
ros2 launch fast_livo mapping_mid360.launch.py
ros2 run r2_controller odometry_printer.py
```

## Competition flow (r2_main.py)

`INIT → MOVE_TO_ENDTIPS → GRAB_ENDTIPS → WAIT_DOCK → RECV_ZONE_I_INFO → PLAN_PATH → SEND_PATH → WAIT_DOCK_OK → ENTER_ZONE_I → NAVIGATE_FOREST → RETURN`
