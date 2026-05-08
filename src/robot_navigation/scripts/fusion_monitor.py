#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
IMU+激光融合里程计 - 实时监控仪表板
"""

import rospy
import sys
import threading
import time
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
import math


class FusionMonitor:
    def __init__(self):
        rospy.init_node('fusion_monitor', anonymous=True)
        
        self.odom_data = None
        self.imu_data = None
        self.scan_data = None
        self.lock = threading.Lock()
        
        # 订阅话题
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        
        self.start_time = time.time()
        
        print("\n")
        print("╔════════════════════════════════════════════════════════╗")
        print("║   IMU+激光融合里程计 - 实时监控仪表板                 ║")
        print("╚════════════════════════════════════════════════════════╝")
        print("\n")
    
    def odom_callback(self, msg):
        with self.lock:
            self.odom_data = msg
    
    def imu_callback(self, msg):
        with self.lock:
            self.imu_data = msg
    
    def scan_callback(self, msg):
        with self.lock:
            self.scan_data = msg
    
    def display_dashboard(self):
        """显示实时仪表板"""
        while not rospy.is_shutdown():
            try:
                # 清屏
                print("\033[2J\033[H", end='')
                
                elapsed = time.time() - self.start_time
                
                # 标题
                print("╔════════════════════════════════════════════════════════╗")
                print("║   IMU+激光融合里程计 - 实时监控仪表板                 ║")
                print(f"║   运行时间: {int(elapsed)}s                            ║")
                print("╚════════════════════════════════════════════════════════╝")
                print()
                
                with self.lock:
                    # 里程计数据
                    if self.odom_data:
                        odom = self.odom_data
                        pos = odom.pose.pose.position
                        orient = odom.pose.pose.orientation
                        twist = odom.twist.twist
                        
                        # 四元数转欧拉角
                        yaw = self.quat_to_yaw(orient)
                        
                        print("📍 里程计数据 (Odometry)")
                        print("─" * 56)
                        print(f"  位置:  x={pos.x:8.4f}m  y={pos.y:8.4f}m")
                        print(f"  方向:  yaw={yaw:8.4f}rad ({math.degrees(yaw):7.2f}°)")
                        print(f"  线速度: vx={twist.linear.x:7.3f}m/s  vy={twist.linear.y:7.3f}m/s")
                        print(f"  角速度: wz={twist.angular.z:7.3f}rad/s")
                        print()
                    else:
                        print("⚠️  等待里程计数据...")
                        print()
                    
                    # IMU数据
                    if self.imu_data:
                        imu = self.imu_data
                        print("🧭 IMU传感器数据")
                        print("─" * 56)
                        print(f"  角速度(Z):     {imu.angular_velocity.z:8.4f}rad/s")
                        print(f"  线加速度(X):   {imu.linear_acceleration.x:8.4f}m/s²")
                        print(f"  线加速度(Y):   {imu.linear_acceleration.y:8.4f}m/s²")
                        print()
                    else:
                        print("⚠️  等待IMU数据...")
                        print()
                    
                    # 激光扫描数据
                    if self.scan_data:
                        scan = self.scan_data
                        valid_ranges = [r for r in scan.ranges 
                                      if scan.range_min < r < scan.range_max]
                        
                        print("📡 激光扫描数据")
                        print("─" * 56)
                        print(f"  扫描点数:      {len(scan.ranges)}")
                        print(f"  有效点数:      {len(valid_ranges)}")
                        print(f"  角度范围:      {math.degrees(scan.angle_min):7.2f}° ~ {math.degrees(scan.angle_max):7.2f}°")
                        print(f"  距离范围:      {scan.range_min:6.3f}m ~ {scan.range_max:6.3f}m")
                        if valid_ranges:
                            print(f"  有效距离:      {min(valid_ranges):6.3f}m ~ {max(valid_ranges):6.3f}m")
                        print()
                    else:
                        print("⚠️  等待激光扫描数据...")
                        print()
                
                # 系统状态
                print("✅ 系统状态")
                print("─" * 56)
                print(f"  ROS Master: 在线")
                print(f"  节点状态:   正在运行")
                print(f"  话题发布:   /odom (50Hz), /imu (~20Hz), /scan (~10Hz)")
                print()
                
                # 底部提示
                print("💡 快捷命令:")
                print("  • rostopic echo /odom         查看详细里程计")
                print("  • rostopic echo /imu          查看IMU数据")
                print("  • rostopic hz /odom           检查发布频率")
                print("  • rviz                        启动可视化")
                print()
                print("按 Ctrl+C 停止监控")
                print()
                
                time.sleep(0.5)  # 2Hz刷新率
                
            except KeyboardInterrupt:
                print("\n\n👋 监控已停止\n")
                break
            except Exception as e:
                rospy.logwarn("监控错误: %s" % str(e))
                time.sleep(0.5)
    
    @staticmethod
    def quat_to_yaw(quat):
        """四元数转欧拉角的yaw分量"""
        # q = [x, y, z, w]
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw


def main():
    try:
        monitor = FusionMonitor()
        
        # 等待一秒让订阅建立
        time.sleep(1)
        
        # 开始显示仪表板
        monitor.display_dashboard()
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\n❌ 错误: {e}\n")


if __name__ == '__main__':
    main()
