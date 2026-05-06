#!/usr/bin/env bash
# tri_steer_odom 问题修复 - 用户行动清单
# 此文件用于追踪修复进度

echo "========================================================================"
echo "  tri_steer_odom 自动运动问题修复进度清单"
echo "========================================================================"
echo ""

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 打印检查项
check_item() {
    echo -e "${YELLOW}☐${NC} $1"
}

complete_item() {
    echo -e "${GREEN}✓${NC} $1"
}

problem_item() {
    echo -e "${RED}✗${NC} $1"
}

echo ""
echo "【第1阶段】代码修改验证"
echo "========================================================================"
echo ""

# 检查1：tri_steer_odom.py 修改
if grep -q "数据有效性检查" /home/cfl/RealSlamRos1/src/robot_navigation/scripts/tri_steer_odom.py; then
    complete_item "tri_steer_odom.py 已包含数据有效性检查"
else
    problem_item "tri_steer_odom.py 数据检查缺失！需要重新应用修改"
fi

# 检查2：角度规范化
if grep -q "math.atan2(math.sin(local_steer), math.cos(local_steer))" /home/cfl/RealSlamRos1/src/robot_navigation/scripts/tri_steer_odom.py; then
    complete_item "tri_steer_odom.py 已实现角度规范化"
else
    problem_item "tri_steer_odom.py 缺少角度规范化！"
fi

# 检查3：低通滤波参数
if grep -q "alpha = 0.4" /home/cfl/RealSlamRos1/src/robot_navigation/scripts/tri_steer_odom.py; then
    complete_item "tri_steer_odom.py 低通滤波参数已优化 (alpha=0.4)"
else
    problem_item "tri_steer_odom.py 滤波参数可能未更新！"
fi

# 检查4：诊断脚本
if [ -f /home/cfl/RealSlamRos1/src/robot_navigation/scripts/diagnose_joint_states.py ]; then
    complete_item "诊断脚本 diagnose_joint_states.py 已创建"
else
    problem_item "诊断脚本不存在！"
fi

echo ""
echo "【第2阶段】环境准备"
echo "========================================================================"
echo ""

check_item "在新终端中运行: cd /home/cfl/RealSlamRos1 && source devel/setup.bash"
check_item "重新编译（如需要）: cd /home/cfl/RealSlamRos1 && catkin_make"

echo ""
echo "【第3阶段】测试流程（按顺序执行）"
echo "========================================================================"
echo ""

echo "步骤 1️⃣ : 关节名称验证"
echo "  命令: rostopic echo /joint_states | grep -E 'name:|^  - joint' | head -20"
echo "  预期: 看到 joint11, joint12, joint21, joint22, joint31, joint32"
echo ""

echo "步骤 2️⃣ : 启动仿真（终端1）"
echo "  命令: cd /home/cfl/RealSlamRos1 && ./sim_lidar_slam.sh"
echo "  等待: 仿真完全启动（约10秒）"
echo ""

echo "步骤 3️⃣ : 启动导航节点（终端2）"
echo "  命令: cd /home/cfl/RealSlamRos1 && source devel/setup.bash"
echo "       roslaunch robot_navigation motionPlan_sim.launch"
echo "  观察: 应该看到 'tri_steer_odom 已启动' 的日志"
echo ""

echo "步骤 4️⃣ : 运行诊断脚本（终端3，可选）"
echo "  命令: rosrun robot_navigation diagnose_joint_states.py"
echo "  观察: 应该没有'异常'警告，或很少有警告"
echo ""

echo "步骤 5️⃣ : 监测速度输出（终端4）"
echo "  命令: timeout 30 rostopic echo /odom/twist/twist/linear"
echo "  预期结果（CRITICAL）:"
echo "    - 停车时输出应该为 x: 0.0, y: 0.0"
echo "    - 不应该持续输出 > 0.1 的速度值"
echo "    - ✓ 如果看到这个，修复成功！"
echo ""

echo "【第4阶段】问题排查（如修复不成功）"
echo "========================================================================"
echo ""

echo "如果停车时仍有大速度输出，按以下顺序排查:"
echo ""

echo "排查项 A: 关节数据异常"
echo "  命令: rostopic echo /joint_states -n 5 | grep -E 'joint1[23]|joint2[23]|joint3[23]' -A 1"
echo "  正常值范围:"
echo "    - position: 应该在 -3.14 ~ 3.14 之间"
echo "    - velocity: 应该在 -10 ~ 10 之间"
echo "  异常示例: position = -7353 或 velocity = -867.6"
echo ""

echo "排查项 B: 关节名称不匹配"
echo "  检查命令: cat /home/cfl/RealSlamRos1/src/robot_navigation/launch/motionPlan_sim.launch | grep steer_joint"
echo "  应该看到: steer_joint_1 = joint11, steer_joint_2 = joint21, etc."
echo ""

echo "排查项 C: URDF 关节定义问题"
echo "  检查文件: /home/cfl/RealSlamRos1/src/tri_steer_description/urdf/"
echo "  运行: grep -A 5 '<joint name=\"joint12\"' *.urdf"
echo "  查看是否有 <limit lower=\"...\" upper=\"...\"/> 标签"
echo ""

echo "排查项 D: 参数微调"
echo "  如需要，可调整 tri_steer_odom.py 中的参数:"
echo "    - alpha: 0.4 → 0.3（降低历史权重）"
echo "    - decay_factor: 0.8 → 0.6（加快衰减）"
echo "    - deadband 阈值: 0.02 → 0.05（增大死区）"
echo "    - 异常检查: 10.0 rad/s → 5.0 rad/s（更严格的检查）"
echo ""

echo "【第5阶段】验证修复成功"
echo "========================================================================"
echo ""

echo "修复成功的标志 ✓:"
echo "  ✓ 停车时 /odom/twist/twist/linear 输出 (0, 0, 0)"
echo "  ✓ 无命令时小车完全不动"
echo "  ✓ 有速度命令时小车正常响应"
echo "  ✓ 诊断脚本无或很少异常警告"
echo ""

echo "问题仍存在的标志 ✗:"
echo "  ✗ 停车时持续输出 > 0.1 m/s 的速度"
echo "  ✗ 小车自动转圈"
echo "  ✗ 诊断脚本显示关节'异常'"
echo ""

echo "========================================================================"
echo "【快速参考】常用命令"
echo "========================================================================"
echo ""
echo "# 完整测试（需要已启动仿真）"
echo "rosrun robot_navigation diagnose_joint_states.py &"
echo "sleep 2"
echo "rostopic echo /odom/twist/twist/linear -n 20"
echo ""
echo "# 查看完整修复文档"
echo "cat /home/cfl/RealSlamRos1/src/robot_navigation/SOLUTION_SUMMARY.md"
echo ""
echo "# 查看详细改动说明"
echo "cat /home/cfl/RealSlamRos1/src/robot_navigation/CHANGES_DETAILED.md"
echo ""

echo ""
echo "========================================================================"
echo "需要帮助？运行诊断脚本并分享以下信息："
echo "========================================================================"
echo ""
echo "  1. rostopic echo /joint_states -n 3 的完整输出"
echo "  2. 诊断脚本中的异常警告（如有）"
echo "  3. 是否修改过 URDF 或 Gazebo 配置"
echo ""
echo "========================================================================"
