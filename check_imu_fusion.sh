#!/bin/bash

# IMU+激光融合里程计系统健康检查脚本

echo "=================================="
echo "IMU+激光融合里程计 - 健康检查"
echo "=================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 检查函数
check_topic() {
    local topic=$1
    local timeout=$2
    
    echo -n "检查话题 $topic ... "
    
    timeout $timeout rostopic list 2>/dev/null | grep -q "^${topic}$"
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓${NC}"
        return 0
    else
        echo -e "${RED}✗ 未找到${NC}"
        return 1
    fi
}

check_node() {
    local node=$1
    
    echo -n "检查节点 $node ... "
    
    rosnode list 2>/dev/null | grep -q "$node"
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓${NC}"
        return 0
    else
        echo -e "${RED}✗ 未找到${NC}"
        return 1
    fi
}

check_frequency() {
    local topic=$1
    local expected=$2
    
    echo -n "检查 $topic 频率 (期望: ~${expected}Hz) ... "
    
    freq=$(timeout 3 rostopic hz $topic 2>/dev/null | grep "average rate" | awk '{print $NF}')
    
    if [ -z "$freq" ]; then
        echo -e "${RED}✗ 无数据${NC}"
        return 1
    fi
    
    # 检查频率是否在合理范围内（±20%）
    lower=$(echo "$expected * 0.8" | bc)
    upper=$(echo "$expected * 1.2" | bc)
    
    if (( $(echo "$freq >= $lower && $freq <= $upper" | bc -l) )); then
        echo -e "${GREEN}✓ ($freq Hz)${NC}"
        return 0
    else
        echo -e "${YELLOW}⚠ ($freq Hz)${NC}"
        return 1
    fi
}

check_tf() {
    echo -n "检查 TF 变换 (odom → base_footprint) ... "
    
    rosrun tf tf_echo odom base_footprint 2>/dev/null | grep -q "Translation"
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓${NC}"
        return 0
    else
        echo -e "${RED}✗ 无效的TF${NC}"
        return 1
    fi
}

# ================== 检查项 ==================

echo "📦 系统状态检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 1. 检查ROS运行
echo -n "ROS Master ... "
if rostopic list >/dev/null 2>&1; then
    echo -e "${GREEN}✓ 运行中${NC}"
else
    echo -e "${RED}✗ 未运行${NC}"
    echo "请先启动: roscore"
    exit 1
fi

echo ""
echo "🔌 输入话题"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

check_topic "/scan" 2
check_topic "/joint_states" 2
check_topic "/clock" 2

echo ""
echo "📡 输出话题"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

check_topic "/imu" 2
check_topic "/odom" 2

echo ""
echo "🖥️  节点状态"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

check_node "/virtual_imu_publisher"
check_node "/imu_lidar_fusion_odom"

echo ""
echo "📊 数据质量"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

check_frequency "/scan" 10
check_frequency "/imu" 20
check_frequency "/odom" 50

echo ""
echo "🧭 变换框架"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

check_tf

echo ""
echo "📊 数据样本"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

echo "🎯 最新里程计数据:"
timeout 2 rostopic echo /odom -n 1 2>/dev/null | head -20

echo ""
echo "📍 IMU数据样本:"
timeout 2 rostopic echo /imu -n 1 2>/dev/null | head -15

echo ""
echo "=================================="
echo "✅ 检查完成"
echo "=================================="
echo ""
echo "💡 提示:"
echo "  • 启动 RViz: rviz"
echo "  • 实时监控: rqt_graph"
echo "  • 记录数据: rosbag record -a"
echo ""
