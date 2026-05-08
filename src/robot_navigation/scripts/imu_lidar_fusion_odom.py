#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
IMU + Lidar Fusion Odometry Node
融合IMU和激光雷达数据的里程计计算节点
提供更稳定和可靠的位姿估计
"""

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import tf_conversions
from collections import deque


class IMULidarFusionOdometry:
    def __init__(self):
        rospy.init_node('imu_lidar_fusion_odom', anonymous=False)
        
        # 参数设置
        self.update_rate = rospy.get_param('~update_rate', 50)
        self.fusion_mode = rospy.get_param('~fusion_mode', 'complementary')  # complementary or ekf
        self.lidar_weight = rospy.get_param('~lidar_weight', 0.6)
        self.imu_weight = rospy.get_param('~imu_weight', 0.4)
        
        # 位姿状态
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        
        # IMU状态缓冲
        self.angular_velocity_z = 0.0
        self.linear_accel_x = 0.0
        self.linear_accel_y = 0.0
        self.imu_buffer = deque(maxlen=5)
        
        # 激光数据缓冲
        self.prev_scan = None
        self.prev_scan_points = None
        self.scan_count = 0
        
        # 融合参数
        self.alpha_yaw = 0.7  # IMU在yaw融合中的权重
        self.alpha_vel = 0.6  # 激光在速度融合中的权重
        
        # 低通滤波参数
        self.vel_filter_alpha = 0.3
        self.yaw_filter_alpha = 0.5
        
        # 异常检测阈值
        self.max_linear_vel = 2.0  # m/s
        self.max_angular_vel = 3.0  # rad/s
        self.max_accel = 5.0  # m/s^2
        
        # 发布者和订阅者
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.tf_broadcaster = TransformBroadcaster()
        self.cmd_vel_pub = rospy.Publisher('/fusion_cmd_vel', Twist, queue_size=1)
        
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=5)
        
        # 时间戳
        self.last_time = rospy.Time.now()
        self.seq = 0
        
        rospy.loginfo("[IMU+Lidar Fusion] 节点初始化完成")
        rospy.loginfo("[IMU+Lidar Fusion] 融合模式: %s, Lidar权重: %.1f, IMU权重: %.1f" 
                     % (self.fusion_mode, self.lidar_weight, self.imu_weight))
        
    def imu_callback(self, msg):
        """IMU数据回调 - 提供角速度和加速度信息"""
        try:
            # 提取角速度 (z轴旋转)
            self.angular_velocity_z = msg.angular_velocity.z
            
            # 提取线性加速度
            self.linear_accel_x = msg.linear_acceleration.x
            self.linear_accel_y = msg.linear_acceleration.y
            
            # 存入缓冲
            self.imu_buffer.append({
                'time': msg.header.stamp,
                'wz': self.angular_velocity_z,
                'ax': self.linear_accel_x,
                'ay': self.linear_accel_y
            })
            
        except Exception as e:
            rospy.logwarn("[IMU Callback] 错误: %s" % str(e))
    
    def scan_callback(self, msg):
        """激光扫描回调 - 通过扫描匹配估计位移"""
        try:
            if self.prev_scan is None:
                # 第一帧扫描，仅记录
                self.prev_scan = msg
                self.prev_scan_points = self.scan_to_points(msg)
                self.scan_count = 1
                return
            
            # 获取当前扫描点云
            curr_points = self.scan_to_points(msg)
            
            if len(self.prev_scan_points) < 10 or len(curr_points) < 10:
                rospy.logwarn("[Lidar] 有效扫描点太少，跳过")
                return
            
            # 简单的点云对齐 - 计算两帧扫描的点云匹配偏移
            # 这里使用最近点对匹配(ICP)的简化版本
            delta_x, delta_y, delta_yaw = self.estimate_motion(
                self.prev_scan_points, curr_points
            )
            
            # 异常检测
            dt = (msg.header.stamp - self.prev_scan.header.stamp).to_sec()
            if dt > 0.001:  # 避免除零
                est_vx = delta_x / dt
                est_vy = delta_y / dt
                est_wz = delta_yaw / dt
                
                # 检查是否超过阈值
                if (abs(est_vx) > self.max_linear_vel or 
                    abs(est_vy) > self.max_linear_vel or 
                    abs(est_wz) > self.max_angular_vel):
                    rospy.logwarn("[Lidar] 运动估计超出阈值: vx=%.2f, vy=%.2f, wz=%.2f" 
                                % (est_vx, est_vy, est_wz))
                    delta_x = delta_y = delta_yaw = 0.0
                else:
                    # 融合IMU数据
                    if self.fusion_mode == 'complementary':
                        self.update_pose_complementary(delta_x, delta_y, delta_yaw, dt)
                    else:
                        self.update_pose_simple(delta_x, delta_y, delta_yaw, dt)
            
            self.prev_scan = msg
            self.prev_scan_points = curr_points
            self.scan_count += 1
            
            # 每20帧发布一次里程计
            if self.scan_count % 2 == 0:
                self.publish_odom(msg.header.stamp)
                
        except Exception as e:
            rospy.logerr("[Scan Callback] 错误: %s" % str(e))
    
    def scan_to_points(self, scan_msg):
        """将激光扫描消息转换为笛卡尔坐标点"""
        points = []
        angle = scan_msg.angle_min
        
        for r in scan_msg.ranges:
            if scan_msg.range_min < r < scan_msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])
            angle += scan_msg.angle_increment
        
        return np.array(points) if points else np.empty((0, 2))
    
    def estimate_motion(self, prev_points, curr_points):
        """估计两帧扫描之间的运动（位移和旋转）"""
        try:
            if len(prev_points) < 5 or len(curr_points) < 5:
                return 0.0, 0.0, 0.0
            
            # 简化的匹配：使用重心对齐
            prev_center = np.mean(prev_points, axis=0)
            curr_center = np.mean(curr_points, axis=0)
            
            delta_x = curr_center[0] - prev_center[0]
            delta_y = curr_center[1] - prev_center[1]
            
            # 简单的旋转估计：使用方向差异
            delta_yaw = 0.0  # 可通过SVD改进，这里简化处理
            
            return delta_x, delta_y, delta_yaw
            
        except Exception as e:
            rospy.logwarn("[Motion Estimation] 错误: %s" % str(e))
            return 0.0, 0.0, 0.0
    
    def update_pose_complementary(self, delta_x, delta_y, delta_yaw, dt):
        """互补滤波融合：结合激光和IMU数据更新位姿"""
        # 激光测量：位置
        # IMU测量：角速度
        
        # 由IMU推估角速度变化
        imu_yaw_rate = self.angular_velocity_z
        
        # 互补滤波：激光提供位置，IMU提供高频角速度
        fused_yaw_rate = (self.lidar_weight * (delta_yaw / dt if dt > 0 else 0) +
                         self.imu_weight * imu_yaw_rate)
        
        # 使用激光的位置增量
        self.x += delta_x * math.cos(self.yaw) - delta_y * math.sin(self.yaw)
        self.y += delta_x * math.sin(self.yaw) + delta_y * math.cos(self.yaw)
        
        # 使用融合的角速度更新yaw
        self.yaw += fused_yaw_rate * dt
        self.yaw = self.normalize_angle(self.yaw)
        
        # 低通滤波速度
        if dt > 0.001:
            est_vx = delta_x / dt
            est_vy = delta_y / dt
            self.vx = self.vel_filter_alpha * est_vx + (1 - self.vel_filter_alpha) * self.vx
            self.vy = self.vel_filter_alpha * est_vy + (1 - self.vel_filter_alpha) * self.vy
            self.wz = self.yaw_filter_alpha * fused_yaw_rate + (1 - self.yaw_filter_alpha) * self.wz
    
    def update_pose_simple(self, delta_x, delta_y, delta_yaw, dt):
        """简单融合：直接加权平均"""
        # 激光贡献位置变化
        self.x += delta_x
        self.y += delta_y
        
        # 融合yaw：激光 + IMU
        if dt > 0.001:
            lidar_yaw_rate = delta_yaw / dt
            fused_yaw = self.alpha_yaw * lidar_yaw_rate + (1 - self.alpha_yaw) * self.angular_velocity_z
            self.yaw += fused_yaw * dt
        
        self.yaw = self.normalize_angle(self.yaw)
        
        # 速度估计
        if dt > 0.001:
            self.vx = delta_x / dt
            self.vy = delta_y / dt
            self.wz = self.angular_velocity_z
    
    def publish_odom(self, timestamp):
        """发布里程计消息"""
        try:
            # 创建Odometry消息
            odom = Odometry()
            odom.header.seq = self.seq
            odom.header.stamp = timestamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'
            
            # 位置
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            
            # 姿态（四元数）
            quat = tf_conversions.transformations.quaternion_from_euler(0, 0, self.yaw)
            odom.pose.pose.orientation.x = quat[0]
            odom.pose.pose.orientation.y = quat[1]
            odom.pose.pose.orientation.z = quat[2]
            odom.pose.pose.orientation.w = quat[3]
            
            # 速度
            odom.twist.twist.linear.x = self.vx
            odom.twist.twist.linear.y = self.vy
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = self.wz
            
            # 发布
            self.odom_pub.publish(odom)
            
            # 发布TF变换
            self.publish_tf(timestamp, quat)
            
            self.seq += 1
            
        except Exception as e:
            rospy.logerr("[Publish Odom] 错误: %s" % str(e))
    
    def publish_tf(self, timestamp, quat):
        """发布TF变换 odom -> base_footprint"""
        try:
            tf = TransformStamped()
            tf.header.stamp = timestamp
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_footprint'
            
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            
            tf.transform.rotation.x = quat[0]
            tf.transform.rotation.y = quat[1]
            tf.transform.rotation.z = quat[2]
            tf.transform.rotation.w = quat[3]
            
            self.tf_broadcaster.sendTransform(tf)
            
        except Exception as e:
            rospy.logerr("[Publish TF] 错误: %s" % str(e))
    
    @staticmethod
    def normalize_angle(angle):
        """将角度规范化到 [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(self.update_rate)
        
        rospy.loginfo("[IMU+Lidar Fusion] 开始运行...")
        
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
        
        rospy.loginfo("[IMU+Lidar Fusion] 节点关闭")


if __name__ == '__main__':
    try:
        fusion_odom = IMULidarFusionOdometry()
        fusion_odom.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("致命错误: %s" % str(e))
