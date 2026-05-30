#!/bin/bash
#
# R2 上位机自动重启脚本
# 当 MCU 复位（发送 0xAB）时，serial_bridge 会退出，
# 触发整个 launch 关闭，然后此脚本自动重新启动所有节点。
#
# 用法: 手动运行或由 systemd 服务拉起
#

set -e
set -o pipefail

ROS2_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="/home/shijue2/main_R2/uppc/ws/install/setup.bash"

export PATH="/usr/bin:/bin:/usr/local/bin:/opt/ros/humble/bin:$PATH"
export LD_LIBRARY_PATH=""

if [ ! -f "$ROS2_SETUP" ]; then
    echo "ERROR: ROS2 not found at $ROS2_SETUP"
    exit 1
fi
if [ ! -f "$WORKSPACE_SETUP" ]; then
    echo "ERROR: workspace not built, run: colcon build --symlink-install"
    exit 1
fi

source "$ROS2_SETUP"
source "$WORKSPACE_SETUP"

echo "ENV: AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH" >&2
echo "ENV: PATH=$PATH" >&2

LAUNCH_FILE="d435i_camera r2_serial_only.launch.py"
MIN_UPTIME_SEC=5
RESTART_DELAY_SEC=2
LOG_DIR="$HOME/r2_logs"

mkdir -p "$LOG_DIR"

restart_count=0
stop_requested=0

trap 'stop_requested=1' INT TERM

while true; do
    if [ $stop_requested -eq 1 ]; then
        echo "收到退出信号，停止重启循环"
        break
    fi

    restart_count=$((restart_count + 1))
    start_time=$(date +%s)

    echo "============================================"
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] R2 启动 #${restart_count}"
    echo "============================================"

    log_file="$LOG_DIR/r2_$(date '+%Y%m%d_%H%M%S').log"

    ros2 launch $LAUNCH_FILE 2>&1 | tee "$log_file"
    exit_code=$?

    elapsed=$(($(date +%s) - start_time))

    echo "[$(date '+%Y-%m-%d %H:%M:%S')] 系统退出 (exit=$exit_code, 运行了 ${elapsed}s)"

    if [ $elapsed -lt $MIN_UPTIME_SEC ]; then
        wait_time=5
        echo "运行不到 ${MIN_UPTIME_SEC}s 就退出，等待 ${wait_time}s 后重试..."
    else
        wait_time=$RESTART_DELAY_SEC
        echo "等待 ${RESTART_DELAY_SEC}s 后重启..."
    fi

    while [ $wait_time -gt 0 ] && [ $stop_requested -eq 0 ]; do
        sleep 1
        wait_time=$((wait_time - 1))
    done

    rm -f /tmp/r2_reset
done
