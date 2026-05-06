# 仿真启动故障排查指南

## 问题现象

```
WARNING: no messages received and simulated time is active.
Is /clock being published?
```

这说明 **Gazebo 仿真没有正确启动**或 ROS 的时间同步有问题。

---

## 问题诊断

### 检查1：ROS Master 是否运行？

```bash
# 查看 roscore 进程
pgrep -a rosmaster
# 或
ps aux | grep roscore
```

**预期**：应该看到 `roscore` 进程

**如果没有**：
```bash
# 手动启动
roscore &
# 等待3秒
sleep 3
```

---

### 检查2：Gazebo 是否启动？

```bash
# 查看话题列表
rostopic list

# 关键话题应该包括：
# /clock                    ← Gazebo 时钟
# /gazebo/model_states      ← Gazebo 模型状态
# /tf                       ← TF 变换

# 查看详细的话题
rostopic list | grep -E "clock|gazebo|tf"
```

**预期输出**：应该至少包含 `/clock`

**如果 `/clock` 不存在**：
```bash
# Gazebo 可能未启动或启动失败
# 检查 Gazebo 进程
pgrep -a gzserver
pgrep -a gzclient

# 如果没有，需要手动启动
roslaunch tri_steer_gazebo sim_gazebo_rviz.launch
```

---

### 检查3：/joint_states 是否发布？

```bash
# 监测 joint_states 话题
rostopic echo /joint_states -n 1

# 应该看到：
# header: 
# seq: ...
# stamp: ...
# name: [joint11, joint12, joint21, ...]
# position: [...]
# velocity: [...]
```

**如果没有 joint_states**：
- Gazebo 可能没有正确加载 URDF
- 检查 tri_steer_gazebo 的 launch 文件

---

### 检查4：导航节点是否发布 /odom？

```bash
# 启动导航节点后，检查是否发布 odom
rostopic list | grep odom

# 监测 odom 话题
rostopic echo /odom/twist/twist/linear -n 3
```

---

## 推荐的启动流程

### 方法 A：使用改进的启动脚本（推荐）

```bash
cd /home/cfl/RealSlamRos1

# 启动基础仿真（只有 roscore + Gazebo + 导航）
./sim_minimal.sh

# 在另一个终端检查
rostopic list
timeout 10 rostopic echo /odom/twist/twist/linear
```

### 方法 B：手动逐步启动

**终端1：ROS Core**
```bash
cd /home/cfl/RealSlamRos1
source devel/setup.bash
roscore
```

**终端2：Gazebo（等待终端1完全启动）**
```bash
cd /home/cfl/RealSlamRos1
source devel/setup.bash
roslaunch tri_steer_gazebo sim_gazebo_rviz.launch
```

**终端3：导航节点（等待终端2 Gazebo 完全启动）**
```bash
cd /home/cfl/RealSlamRos1
source devel/setup.bash
# 不运行完整的 motionPlan_sim.launch，改为只运行必要的部分
rosrun robot_navigation tri_steer_odom.py
```

**终端4：监测（等待所有节点启动）**
```bash
# 等待3秒
sleep 3

# 检查话题
rostopic list

# 监测 odom
rostopic echo /odom/twist/twist/linear -n 20
```

---

## 完整的诊断命令

### 一键诊断

```bash
cd /home/cfl/RealSlamRos1
bash src/robot_navigation/scripts/diagnose_startup.sh
```

这会输出：
- ROS Master 状态
- 所有发布的话题
- 关键话题是否存在

---

## 常见问题解决

### 问题 1：/clock 不存在

**原因**：Gazebo 未启动或未正确连接到 ROS

**解决**：
```bash
# 1. 检查 Gazebo 进程
pgrep gzserver

# 2. 如果没有，启动它
roslaunch tri_steer_gazebo sim_gazebo_rviz.launch

# 3. 等待 /clock 出现（最多30秒）
timeout 30 bash -c 'while ! rostopic list | grep -q /clock; do sleep 1; done; echo "✓ /clock found"'
```

---

### 问题 2：/joint_states 不存在

**原因**：URDF 未加载或 Gazebo 中没有关节

**解决**：
```bash
# 1. 检查 URDF 文件
cat ~/RealSlamRos1/src/tri_steer_description/urdf/*.urdf | head -20

# 2. 检查 launch 文件中是否正确指定了 URDF
cat ~/RealSlamRos1/src/tri_steer_gazebo/launch/sim_gazebo_rviz.launch | grep urdf

# 3. 重新启动 Gazebo
killall gzserver gzclient
roslaunch tri_steer_gazebo sim_gazebo_rviz.launch
```

---

### 问题 3：/odom 不发布

**原因**：tri_steer_odom.py 未启动或启动失败

**解决**：
```bash
# 1. 直接运行 tri_steer_odom 查看错误
rosrun robot_navigation tri_steer_odom.py

# 2. 应该会看到日志输出，如果有错误：
#    - 关节名称不匹配 → 检查 launch 文件参数
#    - 无法读取参数 → 检查 ROS 参数服务器
```

---

### 问题 4：仍然收不到 /odom 消息

**检查步骤**：

```bash
# 1. 确认 tri_steer_odom.py 运行中
ps aux | grep tri_steer_odom

# 2. 查看节点的详细信息
rosnode info /tri_steer_odom

# 3. 检查是否有发布者
rostopic info /odom

# 4. 看看 joint_states 数据是否异常
rostopic echo /joint_states -n 1

# 5. 查看 tri_steer_odom 的日志输出
# （需要在启动时用 output="screen" 查看）
```

---

## 快速测试清单

- [ ] ROS Master 运行中 (`pgrep roscore`)
- [ ] Gazebo 进程存在 (`pgrep gzserver`)
- [ ] /clock 话题存在 (`rostopic list | grep /clock`)
- [ ] /joint_states 话题存在 (`rostopic list | grep joint_states`)
- [ ] tri_steer_odom 节点运行 (`rosnode list | grep tri_steer_odom`)
- [ ] /odom 话题发布 (`rostopic list | grep odom`)
- [ ] /odom 有数据 (`rostopic echo /odom -n 1` 有输出)

---

## 核心关键路径

```
roscore
   ↓
Gazebo (sim_gazebo_rviz.launch)
   ↓ 发布：/clock, /gazebo/model_states, /joint_states
   ↓
tri_steer_odom.py
   ↓ 订阅：/joint_states
   ↓ 发布：/odom, TF(odom->base_footprint)
   ↓
导航模块订阅 /odom
```

如果某一环节缺失，后续都无法工作。

---

## 最后的建议

1. **确保每一步都等待充分**
   - roscore 启动需要 2-3 秒
   - Gazebo 启动需要 5-10 秒
   - 节点启动需要 1-2 秒

2. **使用诊断脚本验证**
   ```bash
   bash diagnose_startup.sh
   ```

3. **查看完整的日志输出**
   - 启动 launch 文件时加 `output="screen"`
   - 查看终端输出中的任何错误信息

4. **如果问题持续存在**
   - 查看 ROS 日志：`roslaunch tri_steer_gazebo sim_gazebo_rviz.launch --screen`
   - 检查 Gazebo GUI 是否有错误消息
