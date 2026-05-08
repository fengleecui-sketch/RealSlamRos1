#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
虚拟 IMU 发布器
从 Gazebo 的 joint_states 推导 IMU 角速度和加速度
"""

import rospy
import math
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from collections import deque


class VirtualIMUPublisher:
    def __init__(self):
        rospy.init_node('virtual_imu_publisher', anonymous=False)
        
        # IMU发布者
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
        
        # 订阅joint_states用于推导角速度
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback, queue_size=1)
        
        # 状态缓冲 - 用于计算导数
        self.joint_history = deque(maxlen=3)
        self.prev_angular_vel_z = 0.0
        self.seq = 0
        
        # IMU参数
        self.gyro_noise = rospy.get_param('~gyro_noise', 0.01)
        self.accel_noise = rospy.get_param('~accel_noise', 0.05)
        
        rospy.loginfo("[Virtual IMU] 发布器初始化完成")
    
    def joint_callback(self, msg):
        """从joint_states推导IMU数据"""
        try:
            # 查找旋转关节（转向电机）
            # 三轮全向: joint11, joint21, joint31 是转向电机
            # 通过转向速度的变化推导角加速度
            
            # 存储历史数据
            joint_data = {
                'time': msg.header.stamp,
                'positions': msg.position,
                'velocities': msg.velocity,
                'efforts': msg.effort,
                'names': msg.name
            }
            self.joint_history.append(joint_data)
            
            if len(self.joint_history) < 2:
                return
            
            # 计算角速度变化（作为角加速度）
            curr_data = self.joint_history[-1]
            prev_data = self.joint_history[-2]
            
            dt = (curr_data['time'] - prev_data['time']).to_sec()
            if dt < 0.001:  # 时间太短
                return
            
            # 从转向电机速度推估平面角速度
            # 这是一个简化的模型
            steering_velocities = []
            for i, name in enumerate(curr_data['names']):
                if name in ['joint11', 'joint21', 'joint31']:  # 转向电机
                    steering_velocities.append(curr_data['velocities'][i])
            
            # 平面角速度 = 平均转向速度的投影
            if steering_velocities:
                avg_steering_vel = sum(steering_velocities) / len(steering_velocities)
                # 简化关系：角速度 ≈ 转向速度 * 某个系数
                angular_vel_z = avg_steering_vel * 0.5  # 根据几何关系调整
            else:
                angular_vel_z = 0.0
            
            # 计算角加速度
            angular_accel_z = (angular_vel_z - self.prev_angular_vel_z) / dt
            self.prev_angular_vel_z = angular_vel_z
            
            # 创建IMU消息
            imu_msg = Imu()
            imu_msg.header.seq = self.seq
            imu_msg.header.stamp = curr_data['time']
            imu_msg.header.frame_id = 'base_link'
            
            # 角速度（围绕Z轴）
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = angular_vel_z + (math.random() * 2 - 1) * self.gyro_noise
            
            # 角加速度 -> 对角加速度的估计（线性加速度在XY平面）
            # a_tangential = r * angular_accel，其中r是电机到车体中心的距离
            r = 0.15  # 大约150mm（可根据实际调整）
            linear_accel = r * angular_accel_z
            
            imu_msg.linear_acceleration.x = linear_accel + (math.random() * 2 - 1) * self.accel_noise
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 0.0  # 重力补偿
            
            # 协方差（使用默认值）
            imu_msg.angular_velocity_covariance = [0.0] * 9
            imu_msg.linear_acceleration_covariance = [0.0] * 9
            imu_msg.orientation_covariance = [0.0] * 9
            
            # 发布
            self.imu_pub.publish(imu_msg)
            self.seq += 1
            
            if self.seq % 50 == 0:
                rospy.logdebug("[Virtual IMU] 发布: wz=%.3f, ax=%.3f" 
                             % (imu_msg.angular_velocity.z, imu_msg.linear_acceleration.x))
            
        except Exception as e:
            rospy.logerr("[Virtual IMU] 错误: %s" % str(e))


def main():
    try:
        imu_pub = VirtualIMUPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("致命错误: %s" % str(e))


if __name__ == '__main__':
    main()
