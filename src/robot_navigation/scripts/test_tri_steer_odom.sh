#!/bin/bash
# 快速测试脚本：验证 tri_steer_odom 修复效果

echo "===================================================="
echo "tri_steer_odom 快速测试"
echo "===================================================="

# 检查是否在ROS环境中
if [ -z "$ROS_MASTER_URI" ]; then
    echo "❌ 未检测到ROS环境，请先source setup.bash"
    exit 1
fi

echo ""
echo "【测试 1】检查 joint_states 数据异常情况"
echo "=================================================="
echo "运行诊断脚本... (按 Ctrl+C 停止)"
echo ""

rosrun robot_navigation diagnose_joint_states.py &
DIAG_PID=$!

sleep 5

echo ""
echo "【测试 2】检查 /odom 速度输出"
echo "=================================================="
echo "【停车状态】应该输出接近 0 的速度值"
echo ""

timeout 10 rostopic echo /odom/twist/twist/linear || true

echo ""
echo "【测试 3】关键文件检查"
echo "=================================================="

# 检查诊断脚本
if [ -f "$(rospack find robot_navigation)/scripts/diagnose_joint_states.py" ]; then
    echo "✅ diagnose_joint_states.py 存在"
else
    echo "❌ diagnose_joint_states.py 不存在"
fi

# 检查修改后的 tri_steer_odom
if grep -q "编码器累积值" "$(rospack find robot_navigation)/scripts/tri_steer_odom.py"; then
    echo "✅ tri_steer_odom.py 已包含新的数据检查"
else
    echo "❌ tri_steer_odom.py 未更新"
fi

# 清理
kill $DIAG_PID 2>/dev/null || true

echo ""
echo "===================================================="
echo "测试完成！"
echo "===================================================="
echo ""
echo "📖 详细文档: README_tri_steer_odom_fix.md"
echo "💡 建议: 检查 URDF 配置和关节限制"
echo ""
