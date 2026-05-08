#!/usr/bin/env bash
# tf 树连通性检查脚本

echo "════════════════════════════════════════════════════════════════════════"
echo "【tf 树连通性检查】tri_steer_odom 修复验证"
echo "════════════════════════════════════════════════════════════════════════"
echo ""

cd /home/cfl/RealSlamRos1
source devel/setup.bash

# 检查 1：tf_monitor 输出
echo "【检查 1】tf 树状态 (odom -> base_footprint)"
echo "─────────────────────────────────────────────────"
timeout 5 rosrun tf tf_monitor odom base_footprint 2>/dev/null || {
    echo "❌ tf_monitor 超时或出错"
    echo "   可能原因: 节点未启动 或 tf 树断裂"
}
echo ""

# 检查 2：tf_tree 输出
echo "【检查 2】完整 tf 树"
echo "─────────────────────────────────────────────────"
rosrun tf tf_tree > /tmp/tf_tree.pdf 2>/dev/null
echo "✅ tf 树已导出到: /tmp/tf_tree.pdf"
echo ""

# 检查 3：rostopic hz
echo "【检查 3】/odom 发布频率"
echo "─────────────────────────────────────────────────"
timeout 3 rostopic hz /odom 2>/dev/null || {
    echo "❌ /odom 话题无数据或频率为0"
}
echo ""

# 检查 4：/odom 内容
echo "【检查 4】/odom 消息内容"
echo "─────────────────────────────────────────────────"
echo "采样 3 个消息："
timeout 2 rostopic echo /odom -n 3 2>/dev/null | grep -E "^x:|^y:|linear" | head -10
echo ""

# 检查 5：tf_buffer 查询
echo "【检查 5】TF 缓冲区查询"
echo "─────────────────────────────────────────────────"
python3 << 'PYTHON_EOF'
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

try:
    rospy.init_node('tf_checker', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rospy.sleep(0.5)  # 等待缓冲区填充
    
    try:
        # 尝试查询最新的 tf
        transform = tf_buffer.lookup_transform('odom', 'base_footprint', rospy.Time(0), timeout=rospy.Duration(2.0))
        print("✅ TF 查询成功")
        print(f"   位置: x={transform.transform.translation.x:.3f}, y={transform.transform.translation.y:.3f}")
        print(f"   来源: {transform.header.frame_id} -> {transform.child_frame_id}")
    except tf2_ros.LookupException as e:
        print(f"❌ TF 查询失败: {e}")
        print("   这说明 tf 树可能断裂")
except Exception as e:
    print(f"⚠️ 检查出错: {e}")
PYTHON_EOF
echo ""

# 检查 6：诊断
echo "════════════════════════════════════════════════════════════════════════"
echo "【诊断总结】"
echo "════════════════════════════════════════════════════════════════════════"
echo ""
echo "✅ 如果上面都能看到数据，说明 tf 树已连通"
echo ""
echo "❌ 如果看到异常，按下列排查："
echo ""
echo "1️⃣ 检查节点是否在运行"
echo "   rosnode list | grep tri_steer_odom"
echo ""
echo "2️⃣ 检查 /odom 话题是否有发布者"
echo "   rostopic info /odom"
echo ""
echo "3️⃣ 查看节点日志"
echo "   roslaunch robot_navigation motionPlan_sim.launch"
echo "   观察是否有 [WARN] 异常消息"
echo ""
echo "4️⃣ 检查仿真是否运行"
echo "   rostopic list | grep /clock"
echo ""
