#!/bin/bash
# 改进的仿真启动脚本 - 确保所有必要组件都启动

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR=$(dirname $(dirname $(dirname "$SCRIPT_DIR")))

echo "=========================================="
echo "启动 ROS 仿真环境"
echo "=========================================="
echo ""

# 清理之前的进程
echo "【步骤1】清理之前的 ROS 进程..."
killall -9 rosmaster roslaunch gzserver gzclient 2>/dev/null || true
killall -9 roscore 2>/dev/null || true
sleep 2

# Source 环境
echo "【步骤2】加载 ROS 环境..."
cd "$WS_DIR"
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

# 启动 roscore
echo "【步骤3】启动 ROS Core..."
roscore &
ROSCORE_PID=$!
sleep 3

# 检查 roscore 是否成功启动
if ! kill -0 $ROSCORE_PID 2>/dev/null; then
    echo "✗ ROS Core 启动失用失败！"
    exit 1
fi
echo "✓ ROS Core 已启动 (PID: $ROSCORE_PID)"

# 启动 Gazebo 仿真
echo ""
echo "【步骤4】启动 Gazebo 仿真..."
echo "运行: roslaunch tri_steer_gazebo sim.launch"
echo ""

# 等待 ROS 完全初始化
sleep 2

# 检查 /clock 话题
echo "【步骤5】验证仿真是否正确启动..."
timeout 30 bash -c 'while ! rostopic list | grep -q /clock; do echo "等待 /clock 话题..."; sleep 1; done' && \
    echo "✓ Gazebo 仿真已启动，/clock 话题正常发布" || \
    echo "⚠️ 等待 /clock 话题超时，仿真可能未启动"

echo ""
echo "【步骤6】检查关键话题..."
rostopic list | grep -E "clock|joint_states|odom|cmd_vel" || echo "注意：某些话题可能未发布"

echo ""
echo "=========================================="
echo "✓ 仿真环境已就绪"
echo "=========================================="
echo ""
echo "现在可以在另一个终端中运行："
echo "  roslaunch robot_navigation motionPlan_sim.launch"
echo ""
echo "按 Ctrl+C 停止所有进程"
echo ""

# 保持脚本运行
wait $ROSCORE_PID
