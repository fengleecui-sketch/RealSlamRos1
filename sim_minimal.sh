#!/bin/bash
# 最小化仿真启动 - 用于诊断 tri_steer_odom

echo "=========================================="
echo "最小化仿真启动 - 诊断模式"
echo "=========================================="
echo ""

# 清理
killall -9 roscore rosmaster gzserver gzclient 2>/dev/null || true
sleep 2

# 加载环境
source ~/.bashrc
cd /home/cfl/RealSlamRos1
source devel/setup.bash

echo "【步骤1】启动 roscore..."
gnome-terminal --title="roscore" -- bash -c "roscore" &
ROSCORE_PID=$!
sleep 3

echo "【步骤2】启动 Gazebo 仿真..."
gnome-terminal --title="gazebo_start" -- bash -c "source ~/RealSlamRos1/devel/setup.bash; roslaunch tri_steer_gazebo sim_gazebo_rviz.launch" &
GAZEBO_PID=$!

echo ""
echo "等待 Gazebo 启动..."
for i in {1..30}; do
    if rostopic list 2>/dev/null | grep -q "/gazebo/model_states"; then
        echo "✓ Gazebo 已启动"
        break
    fi
    echo "  等待中... ($i/30)"
    sleep 1
done

sleep 2

echo ""
echo "【步骤3】启动导航节点..."
gnome-terminal --title="motion plan" -- bash -c "source ~/RealSlamRos1/devel/setup.bash; roslaunch robot_navigation motionPlan_sim.launch" &

echo ""
echo "=========================================="
echo "仿真启动完成"
echo "=========================================="
echo ""
echo "现在你可以在新终端中检查话题："
echo "  rostopic list"
echo "  rostopic echo /odom/twist/twist/linear"
echo ""
echo "按 Ctrl+C 停止所有进程"
echo ""

wait $ROSCORE_PID
