#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf_conversions

class TriSteerOdomNode:
    def __init__(self):
        rospy.init_node("tri_steer_odom")

        # ===================== 【参数读取：完美匹配 Xacro】 =====================
        self.R = float(rospy.get_param("~R", 0.35334))  # 真实舵轮分布半径
        self.wheel_radius = float(rospy.get_param("~wheel_radius", 0.05))

        # 舵轮安装角度
        self.thetas = [
            math.radians(-float(rospy.get_param("~wheel1_deg", 0.0))),
            math.radians(-float(rospy.get_param("~wheel2_deg", 120.0))),
            math.radians(-float(rospy.get_param("~wheel3_deg", 240.0))),
        ]

        # 关节名称 (带有默认值防止读取失败)
        self.steer_joints = [
            rospy.get_param("~steer_joint_1", "joint11"),
            rospy.get_param("~steer_joint_2", "joint21"),
            rospy.get_param("~steer_joint_3", "joint31"),
        ]
        self.drive_joints = [
            rospy.get_param("~drive_joint_1", "joint12"),
            rospy.get_param("~drive_joint_2", "joint22"),
            rospy.get_param("~drive_joint_3", "joint32"),
        ]

        # ===================== 【状态与滤波变量】 =====================
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = rospy.Time.now()

        # 低通滤波历史状态记录
        self.vx_last = 0.0
        self.vy_last = 0.0
        self.wz_last = 0.0

        # ROS 发布与订阅
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.sub = rospy.Subscriber("/joint_states", JointState, self.joint_cb, queue_size=10)

        rospy.loginfo("[tri_steer_odom] 🚀 工业级稳定版三舵轮里程计已启动！")
        
        # 调试模式标志
        self.debug_mode = rospy.get_param("~debug", False)
        self.frame_count = 0

    def joint_cb(self, msg: JointState):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # dt 保护：防止 Gazebo 刚启动或卡顿时时间跳变导致速度爆炸
        if dt < 1e-4 or dt > 0.1:
            return

        self.frame_count += 1
        
        # 每100帧打印一次调试信息
        if self.debug_mode and self.frame_count % 100 == 0:
            rospy.loginfo(f"[tri_steer_odom] 处理第 {self.frame_count} 帧")

        # 安全读取关节数据
        try:
            idx_s1 = msg.name.index(self.steer_joints[0])
            idx_s2 = msg.name.index(self.steer_joints[1])
            idx_s3 = msg.name.index(self.steer_joints[2])
            idx_w1 = msg.name.index(self.drive_joints[0])
            idx_w2 = msg.name.index(self.drive_joints[1])
            idx_w3 = msg.name.index(self.drive_joints[2])
            
            s1 = msg.position[idx_s1]
            s2 = msg.position[idx_s2]
            s3 = msg.position[idx_s3]

            w1 = msg.velocity[idx_w1]
            w2 = msg.velocity[idx_w2]
            w3 = msg.velocity[idx_w3]
        except (ValueError, IndexError) as e:
            rospy.logwarn_throttle(2.0, f"[tri_steer_odom] 关节读取失败: {e}. 可能是关节名称配置不匹配")
            return

        steers = [s1, s2, s3]
        wheels = [w1, w2, w3]

        # ===================== 【数据有效性检查】 =====================
        # 【重要】检查驱动轮速度和转向角是否在合理范围
        # 注意：某些情况下编码器会出现累积值，但大多数时候数据是正常的
        # 为了避免过度过滤，我们使用较宽松的阈值
        
        data_valid = True
        warning_msgs = []
        
        for i, w in enumerate(wheels):
            # 驱动轮速度过大说明可能有问题（通常 < 15 rad/s）
            if abs(w) > 100.0:  # 从 10.0 放宽到 100.0，减少误拦截
                data_valid = False
                warning_msgs.append(f"驱动轮{i+1}速度异常: {w:.2f} rad/s")
        
        for i, s in enumerate(steers):
            # 转向角过大说明可能有问题（通常 -π ~ π）
            if abs(s) > 10 * math.pi:  # 从 4π 放宽到 10π，减少误拦截
                data_valid = False
                warning_msgs.append(f"转向轮{i+1}角度异常: {s:.2f} rad")
        
        if not data_valid:
            rospy.logwarn_throttle(3.0, f"[tri_steer_odom] 数据异常，已忽略: {'; '.join(warning_msgs)}")
            return

        # ===================== 【核心数学：构造矩阵 + 最小二乘求解】 =====================
        A = []
        b = []
        for local_steer, w, mount_theta in zip(steers, wheels, self.thetas):
            # 🔑【核心修复】将从 Gazebo 读取的关节局部角度，还原为底盘全局角度
            # 归一化转向角到 [-π, π] 范围，消除编码器累积值的影响
            global_steer = math.atan2(math.sin(local_steer), math.cos(local_steer))
            
            # 轮子滚动方向单位向量
            nx = math.cos(global_steer)
            ny = math.sin(global_steer)
            
            # 轮子在底盘上的位置坐标
            rx = self.R * math.cos(mount_theta)
            ry = self.R * math.sin(mount_theta)
            
            # 轮子真实线速度
            v = w * self.wheel_radius
            
            # 构建超定方程组: v = vx*nx + vy*ny + wz*(-nx*ry + ny*rx)
            A.append([nx, ny, (-nx * ry + ny * rx)])
            b.append(v)

        # 最小二乘法求解最优底盘速度 (消除三轮运动干涉带来的误差)
        try:
            x_sol = np.linalg.lstsq(np.array(A), np.array(b), rcond=None)[0]
            vx_raw, vy_raw, wz_raw = x_sol
        except np.linalg.LinAlgError:
            return # 矩阵奇异保护

        # ===================== 【信号抗噪处理：死区、限幅、滤波】 =====================
        # 1. 死区防抖 (彻底砍掉停车时 Gazebo 物理引擎 1e-5 级别的幽灵漂移)
        def deadband(val, th=0.01):
            """增强死区阈值，防止幽灵漂移"""
            return 0.0 if abs(val) < th else val

        # 【关键】对原始速度应用严格的死区
        vx_raw = deadband(vx_raw, th=0.02)
        vy_raw = deadband(vy_raw, th=0.02)
        wz_raw = deadband(wz_raw, th=0.03)

        # 2. 速度限幅 (防止极端发散)
        max_v = 2.0
        max_w = 3.0
        vx_raw = max(min(vx_raw, max_v), -max_v)
        vy_raw = max(min(vy_raw, max_v), -max_v)
        wz_raw = max(min(wz_raw, max_w), -max_w)

        # 3. 低通滤波 (大幅平滑轨迹，过滤电机抖动)
        # 【关键修复】降低 alpha 值，防止历史速度积累导致的幽灵运动
        alpha = 0.4  # 从 0.5 进一步降低到 0.4，历史权重减小到40%，实时权重增大到60%
        vx = alpha * self.vx_last + (1 - alpha) * vx_raw
        vy = alpha * self.vy_last + (1 - alpha) * vy_raw
        wz = alpha * self.wz_last + (1 - alpha) * wz_raw

        # 【关键修复】当原始速度为0时，强制历史速度快速衰减到0
        decay_factor = 0.8  # 每周期衰减到上一周期的80%
        if abs(vx_raw) < 1e-6:
            vx = self.vx_last * decay_factor
        if abs(vy_raw) < 1e-6:
            vy = self.vy_last * decay_factor
        if abs(wz_raw) < 1e-6:
            wz = self.wz_last * decay_factor

        # 【超级关键】当速度已经很小时，强制设为0，防止残余振荡
        vx = 0.0 if abs(vx) < 0.005 else vx
        vy = 0.0 if abs(vy) < 0.005 else vy
        wz = 0.0 if abs(wz) < 0.005 else wz

        self.vx_last, self.vy_last, self.wz_last = vx, vy, wz

        # ===================== 【里程计积分与发布】 =====================
        self.yaw += wz * dt
        # 角度防爆：永远限制在 [-pi, pi] 之间，防止跑久了角度数值溢出
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        
        self.x += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        self.y += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt

        # 1. 发布 TF 树 (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.yaw)
        t.transform.rotation = Quaternion(*q)
        self.tf_broadcaster.sendTransform(t)

        # 2. 发布 标准 Odometry 话题
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(*q)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

if __name__ == "__main__":
    try:
        node = TriSteerOdomNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass