#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
诊断脚本：实时监测 joint_states 数据的异常情况
帮助确定是否存在编码器累积值或其他数据问题
"""

import rospy
from sensor_msgs.msg import JointState
import math

class JointDiagnoser:
    def __init__(self):
        rospy.init_node("joint_states_diagnoser")
        self.sub = rospy.Subscriber("/joint_states", JointState, self.callback, queue_size=10)
        rospy.loginfo("[诊断] 开始监测 /joint_states ...")

    def callback(self, msg: JointState):
        print("\n" + "="*80)
        print(f"【时间戳】 {msg.header.stamp.secs}.{msg.header.stamp.nsecs}")
        print("-"*80)
        
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 0
            vel = msg.velocity[i] if i < len(msg.velocity) else 0
            eff = msg.effort[i] if i < len(msg.effort) else 0
            
            # 检测异常
            warnings = []
            
            # 检查位置是否为极端值
            if abs(pos) > 4 * math.pi:
                warnings.append("⚠️  位置异常（>4π，可能是编码器累积值）")
            
            # 检查速度是否为极端值
            if abs(vel) > 10.0:
                warnings.append("⚠️  速度异常（>10 rad/s）")
            
            status = " " + " ".join(warnings) if warnings else ""
            print(f"  {name:12s} | pos: {pos:12.6f} | vel: {vel:10.6f} | eff: {eff:8.4f}{status}")
        
        print("="*80)

if __name__ == "__main__":
    try:
        diagnoser = JointDiagnoser()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
