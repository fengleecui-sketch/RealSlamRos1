#!/usr/bin/env bash
# 【严重问题】/odom 无数据 - 快速排查指南

echo "======================================================================"
echo "【问题诊断】/odom 话题为空 - 快速排查"
echo "======================================================================"
echo ""

cd /home/cfl/RealSlamRos1
source devel/setup.bash

echo "【检查1】tri_steer_odom 节点是否在运行"
echo "─────────────────────────────────────"
if rosnode list 2>/dev/null | grep -q "tri_steer_odom"; then
    echo "✅ 节点在运行"
else
    echo "❌ 节点未运行！"
    echo "   需要启动: roslaunch robot_navigation motionPlan_sim.launch"
    exit 1
fi
echo ""

echo "【检查2】tri_steer_odom 订阅的话题"
echo "─────────────────────────────────────"
rosnode info /tri_steer_odom 2>/dev/null | grep -A 3 "Subscriptions:" | head -5
echo ""

echo "【检查3】/joint_states 数据质量"
echo "─────────────────────────────────────"
echo "正在采样 /joint_states（显示关键字段）..."
timeout 2 rostopic echo /joint_states -n 1 2>/dev/null | grep -E "name:|position:|velocity:" | head -20
echo ""

echo "【检查4】/odom 话题信息"
echo "─────────────────────────────────────"
rostopic info /odom 2>/dev/null | head -20
echo ""

echo "【检查5】尝试手动发布测试数据"
echo "─────────────────────────────────────"
echo "如果前面的检查都通过了，但仍然无数据，可能是："
echo ""
echo "1️⃣ 数据被异常检查拦截了"
echo "   → 运行带调试的版本:"
echo "     roslaunch robot_navigation motionPlan_sim.launch debug:=true"
echo "   → 查看日志是否有'异常'警告"
echo ""
echo "2️⃣ /joint_states 数据本身有问题"
echo "   → 检查上面的 position/velocity 值是否正常"
echo "   → 正常值范围："
echo "     - position: -3.14 ~ 3.14 (弧度)"
echo "     - velocity: -100 ~ 100 (弧度/秒)"
echo ""
echo "3️⃣ 编码器累积值仍然存在"
echo "   → 这说明 URDF 或 Gazebo 配置有问题"
echo "   → 检查关节定义: grep -A 5 'joint12' ~/RealSlamRos1/src/tri_steer_description/urdf/*.urdf"
echo ""

echo "【检查6】实时监测日志"
echo "─────────────────────────────────────"
echo "在新终端运行以查看实时日志："
echo "  roslaunch robot_navigation motionPlan_sim.launch"
echo ""
echo "然后在另一个终端运行以查看是否有异常警告："
echo "  rostopic echo /rosout -n 50 | grep tri_steer_odom"
echo ""
