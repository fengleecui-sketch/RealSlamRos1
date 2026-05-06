#!/usr/bin/env bash
# 完整的快速测试流程 - 包括启动和监测

set -e

echo "======================================================================"
echo "【完整测试流程】tri_steer_odom 修复验证"
echo "======================================================================"
echo ""

# 确保在正确的目录
cd /home/cfl/RealSlamRos1 || exit 1

# Source setup
source devel/setup.bash

echo "【步骤1】检查仿真是否已启动..."
if ! rostopic list 2>/dev/null | grep -q "/clock"; then
    echo "❌ 仿真未启动！"
    echo "请先运行: cd /home/cfl/RealSlamRos1 && ./sim_minimal.sh"
    exit 1
fi
echo "✅ 仿真已启动"
echo ""

echo "【步骤2】启动 tri_steer_odom 节点（如果未启动）..."
if ! rosnode list 2>/dev/null | grep -q "tri_steer_odom"; then
    echo "启动导航节点..."
    roslaunch robot_navigation motionPlan_sim.launch &
    NAV_PID=$!
    echo "等待节点启动..."
    sleep 3
else
    echo "✅ tri_steer_odom 已在运行"
fi
echo ""

echo "【步骤3】检查 /odom 话题..."
if ! rostopic list 2>/dev/null | grep -q "^/odom$"; then
    echo "❌ /odom 话题不存在！"
    exit 1
fi
echo "✅ /odom 话题存在"
echo ""

echo "【步骤4】采样 /odom 数据（5次）..."
echo "─────────────────────────────────────"
for i in {1..5}; do
    echo "【样本 $i】"
    timeout 1 rostopic echo /odom -n 1 2>/dev/null | grep -E "x:|y:|z:" | head -3 || echo "（无数据）"
    sleep 0.2
done
echo ""

echo "【步骤5】采样 /joint_states 关键数据..."
echo "─────────────────────────────────────"
timeout 1 rostopic echo /joint_states -n 1 2>/dev/null | grep -A 12 "^position:" | head -10
echo ""

echo "======================================================================"
echo "【分析结果】"
echo "======================================================================"
echo ""
echo "✅ 如果上面看到：x: 0.0, y: 0.0, z: 0.0"
echo "   → 修复成功！停车速度正确"
echo ""
echo "❌ 如果上面看到：x: 0.5+, y: 0.5+"
echo "   → 还有问题，需要进一步诊断"
echo ""
echo "❌ 如果上面显示 '（无数据）'"
echo "   → /odom 话题没有数据，tri_steer_odom 可能没有发布"
echo ""

# 清理
if [ ! -z "$NAV_PID" ]; then
    kill $NAV_PID 2>/dev/null || true
fi

echo "测试完成！"
