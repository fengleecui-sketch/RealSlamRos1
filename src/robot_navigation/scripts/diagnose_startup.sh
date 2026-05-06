#!/bin/bash
# 仿真启动诊断脚本

echo "=========================================="
echo "ROS 仿真启动诊断"
echo "=========================================="
echo ""

# 1. 检查 ROS Master
echo "【1】检查 ROS Master 状态..."
if pgrep -x "rosmaster" > /dev/null; then
    echo "✓ ROS Master 正在运行"
else
    echo "✗ ROS Master 未运行 - 正在启动..."
    roscore &
    ROSCORE_PID=$!
    sleep 2
fi

# 2. 检查话题
echo ""
echo "【2】检查发布的话题..."
sleep 1

if rostopic list > /dev/null 2>&1; then
    TOPIC_COUNT=$(rostopic list | wc -l)
    echo "✓ 已发现 $TOPIC_COUNT 个话题"
    
    # 检查关键话题
    echo ""
    echo "关键话题检查："
    
    if rostopic list | grep -q "/clock"; then
        echo "  ✓ /clock 话题存在"
    else
        echo "  ✗ /clock 话题不存在 - Gazebo 仿真可能未启动"
    fi
    
    if rostopic list | grep -q "joint_states"; then
        echo "  ✓ /joint_states 话题存在"
    else
        echo "  ✗ /joint_states 话题不存在"
    fi
    
    if rostopic list | grep -q "odom"; then
        echo "  ✓ /odom 话题存在"
    else
        echo "  ✗ /odom 话题不存在"
    fi
    
    if rostopic list | grep -q "scan\|laser"; then
        echo "  ✓ 激光话题存在"
    else
        echo "  ✗ 激光话题不存在 - 需要启动激光驱动"
    fi
    
else
    echo "✗ 无法连接到 ROS Master"
fi

echo ""
echo "【3】完整的话题列表："
echo "=========================================="
rostopic list

echo ""
echo "【4】建议的排查步骤："
echo "=========================================="
echo ""
echo "如果 /clock 不存在，说明 Gazebo 仿真未运行"
echo "请确保："
echo "  1. 正确执行了 ./sim_lidar_slam.sh"
echo "  2. Gazebo GUI 已启动（检查是否有窗口打开）"
echo "  3. ROS 环境变量正确（roscore 在运行）"
echo ""
echo "如果激光话题缺失，可能需要："
echo "  1. 启动激光驱动节点"
echo "  2. 检查 launch 文件配置"
echo ""
