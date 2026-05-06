#!/usr/bin/env bash
# 快速诊断：检查 tri_steer_odom 是否在运行和发布数据

echo "======================================================================"
echo "【快速诊断】tri_steer_odom 状态检查"
echo "======================================================================"
echo ""

# 1. 检查节点是否在运行
echo "【1】检查节点是否在运行..."
if rosnode list | grep -q "tri_steer_odom"; then
    echo "✅ tri_steer_odom 节点正在运行"
else
    echo "❌ tri_steer_odom 节点未运行！需要启动导航节点"
    echo "   运行命令: roslaunch robot_navigation motionPlan_sim.launch"
    exit 1
fi
echo ""

# 2. 检查 /odom 话题是否有发布者
echo "【2】检查 /odom 话题发布者..."
ODOM_PUBLISHERS=$(rostopic info /odom 2>/dev/null | grep "Publishers:" -A 5 | tail -4)
if [ -z "$ODOM_PUBLISHERS" ]; then
    echo "❌ /odom 话题没有发布者"
    exit 1
else
    echo "✅ /odom 话题发布者信息:"
    echo "$ODOM_PUBLISHERS"
fi
echo ""

# 3. 检查 /odom 数据内容
echo "【3】检查 /odom 话题数据（采样3次）..."
echo "─────────────────────────────────────"
for i in {1..3}; do
    echo "【样本 $i】"
    timeout 2 rostopic echo /odom -n 1 2>/dev/null | head -20
    echo ""
done
echo ""

# 4. 检查 /joint_states 数据是否异常
echo "【4】检查 /joint_states 关键关节数据..."
echo "─────────────────────────────────────"
timeout 2 rostopic echo /joint_states -n 1 2>/dev/null | grep -A 15 "^position:" | head -10
echo ""

# 5. 查看节点日志
echo "【5】查看 tri_steer_odom 节点的最近日志..."
echo "─────────────────────────────────────"
rosnode info /tri_steer_odom 2>/dev/null | grep -E "Publications|Subscriptions" -A 10
echo ""

# 6. 检查异常数据是否被拦截
echo "【6】检查是否有异常数据被拦截..."
echo "─────────────────────────────────────"
if dmesg 2>/dev/null | tail -50 | grep -i "encoder\|异常\|abnormal" >/dev/null 2>&1; then
    echo "⚠️  可能有异常数据被拦截"
else
    echo "✅ 未检测到系统级错误"
fi
echo ""

echo "======================================================================"
echo "【诊断总结】"
echo "======================================================================"
echo ""
echo "如果看到上面的 ✅，说明系统正常工作"
echo "如果看到 ❌，请按照提示操作"
echo ""
echo "【常见问题】"
echo "1. 如果 /odom 数据为空 → 数据可能被异常检查拦截了"
echo "   解决: 检查 joint_states 中的 position/velocity 值"
echo ""
echo "2. 如果 twist 数据全是0 → 可能是编码器还在累积数据"
echo "   解决: 等待数据稳定，或检查 URDF 配置"
echo ""
echo "3. 如果节点未运行 → 启动导航节点"
echo "   解决: roslaunch robot_navigation motionPlan_sim.launch"
echo ""
