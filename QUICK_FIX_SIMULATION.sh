#!/bin/bash
# 快速修复：仿真启动 /clock 问题

cat << 'EOF'
========================================
仿真启动问题 - 快速修复
========================================

问题：WARNING: no messages received and simulated time is active. Is /clock being published?
意思：Gazebo 仿真没有启动或未正确连接到 ROS

解决方案有两种：

【方案 A】使用改进的启动脚本（最简单）
========================================
1. 打开终端，运行：
   cd /home/cfl/RealSlamRos1
   ./sim_minimal.sh

2. 这会自动启动：
   ✓ ROS Core
   ✓ Gazebo 仿真
   ✓ 导航节点（tri_steer_odom）

3. 等待3-5秒后，在新终端验证：
   rostopic list | grep -E "clock|joint_states|odom"
   
   应该看到：
   /clock
   /joint_states
   /odom

【方案 B】手动启动（最稳定）
========================================
打开4个终端，按顺序执行：

终端1：启动 ROS Core
  cd /home/cfl/RealSlamRos1
  source devel/setup.bash
  roscore

等待2秒...

终端2：启动 Gazebo
  cd /home/cfl/RealSlamRos1
  source devel/setup.bash
  roslaunch tri_steer_gazebo sim_gazebo_rviz.launch

等待5秒（观察 Gazebo GUI 是否打开）...

终端3：启动导航节点
  cd /home/cfl/RealSlamRos1
  source devel/setup.bash
  roslaunch robot_navigation motionPlan_sim.launch

等待2秒...

终端4：验证
  # 检查话题
  rostopic list | grep -E "clock|joint_states|odom"
  
  # 监测速度输出（关键测试）
  timeout 30 rostopic echo /odom/twist/twist/linear

========================================
快速诊断
========================================

如果仍然有问题，运行诊断脚本：

bash src/robot_navigation/scripts/diagnose_startup.sh

这会告诉你：
- ROS Master 是否运行
- 有多少话题被发布
- 关键话题是否存在

========================================
关键检查点
========================================

□ ROS Master 运行？
  pgrep roscore

□ Gazebo 启动？
  pgrep gzserver

□ /clock 话题存在？
  rostopic list | grep /clock

□ /joint_states 话题存在？
  rostopic list | grep joint_states

□ /odom 话题有数据？
  rostopic echo /odom -n 1

如果全都是✓，问题解决！

========================================
如果还是不行
========================================

1. 检查 Gazebo 错误日志
   查看 Gazebo GUI 窗口是否有错误消息

2. 重启一切
   killall -9 roscore rosmaster gzserver gzclient
   sleep 3
   ./sim_minimal.sh

3. 查看详细的启动文档
   cat /home/cfl/RealSlamRos1/SIMULATION_TROUBLESHOOTING.md

EOF
