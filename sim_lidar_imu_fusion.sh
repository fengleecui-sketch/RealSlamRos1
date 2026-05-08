#!/usr/bin/env bash
### 使用 IMU+激光融合里程计的仿真环境导航
#!/usr/bin/env bash
source ~/.bashrc
set -e

echo "=========================================="
echo "启动 IMU+激光融合里程计仿真系统"
echo "=========================================="

# 1) 启动 roscore
gnome-terminal --title="roscore" -- bash -c "roscore"
# 等待 roscore 就绪
until rostopic list >/dev/null 2>&1; do
  sleep 0.2
done

echo "✓ roscore 启动完成"

# 2) 启动 Gazebo + robot
gnome-terminal --title="gazebo_start" -- bash -c "source ~/RealSlamRos1/devel/setup.bash; roslaunch tri_steer_gazebo sim_gazebo_rviz.launch"
# 等待 Gazebo 节点起来（/gazebo/model_states 出现再继续）
until rostopic list 2>/dev/null | grep -q "/gazebo/model_states"; do
  sleep 0.5
done

echo "✓ Gazebo 启动完成"
sleep 2

# 3) 启动虚拟 IMU 发布器
gnome-terminal -t "virtual_imu" -- bash -lc "source ~/RealSlamRos1/devel/setup.bash; rosrun robot_navigation virtual_imu_publisher.py; exec bash"
sleep 1

echo "✓ 虚拟IMU发布器启动"

# 4) 启动 IMU+激光融合里程计节点
gnome-terminal -t "imu_lidar_fusion" -- bash -lc "source ~/RealSlamRos1/devel/setup.bash; roslaunch robot_navigation imu_lidar_fusion.launch; exec bash"
sleep 2

echo "✓ IMU+激光融合里程计启动"

# 5) 雷达点云反转成水平车体下的point-cloud2
gnome-terminal -t "laser to cloud" -- bash -lc "source ~/RealSlamRos1/devel/setup.bash; roslaunch read_laser_data laser_to_cloud.launch; exec bash"

# 等待雷达数据
until rostopic list 2>/dev/null | grep -q "/cloud"; do
  sleep 0.5
done

echo "✓ 激光数据转换启动"
sleep 1

# 6) 启动 Cartographer 定位
gnome-terminal -t "carto_relocalize" -- bash -lc "cd /home/cfl/Cartographer/Cartographer_Locatization;source install_isolated/setup.bash; roslaunch cartographer_ros my_robot_2d_localization.launch; exec bash"
# 等待 Cartographer 发布 /map
until rostopic list 2>/dev/null | grep -q "^/map$"; do
  sleep 0.5
done

echo "✓ Cartographer定位启动"
sleep 2

# 7) 启动地图处理节点
gnome-terminal -t "map start" -- bash -lc "source ~/RealSlamRos1/devel/setup.bash; roslaunch robot_costmap map_deal.launch;exec bash"

echo "✓ 地图处理节点启动"
sleep 1

# 8) 运动规划节点
gnome-terminal -t "motion Plan start" -- bash -lc "source ~/RealSlamRos1/devel/setup.bash; roslaunch robot_navigation motionPlan_sim.launch"

echo "✓ 运动规划节点启动"
sleep 1

# 9) 局部规划 PID 算法
gnome-terminal -t "PID_control" -- bash -c "source ~/RealSlamRos1/devel/setup.bash; roslaunch robot_pid_local_planner Omnidirectional_PID.launch; exec bash"

echo "✓ PID控制器启动"
sleep 1

# 10) 控制消息订阅（遥控/自主）
gnome-terminal -t "control start" -- bash -lc "source ~/RealSlamRos1/devel/setup.bash; roslaunch tri_steer_drive bringup.launch use_teleop:=true;exec bash"

echo "✓ 控制系统启动"
echo ""
echo "=========================================="
echo "系统启动完成！"
echo "=========================================="
echo ""
echo "📊 关键话题："
echo "  • /imu              - 虚拟IMU数据"
echo "  • /scan             - 激光扫描"
echo "  • /odom             - 融合里程计（新）"
echo "  • /cmd_vel_auto     - 自主运动命令"
echo ""
echo "🔧 调试命令："
echo "  rostopic echo /imu"
echo "  rostopic echo /odom"
echo "  rostopic hz /odom"
echo ""
