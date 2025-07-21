#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
import math

class SimulateUSVInputs:
    def __init__(self):
        rospy.init_node("simulate_usv_inputs")

        self.pose_pub = rospy.Publisher("/usv/position_local", PoseStamped, queue_size=1)
        self.vel_pub = rospy.Publisher("/usv/velocity_local", TwistStamped, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.02), self.publish_cb)  # 20Hz
        self.start_time = rospy.Time.now().to_sec()

    def publish_cb(self, event):
        t = rospy.Time.now().to_sec() - self.start_time

        # 模拟圆形轨迹
        radius = 200.0
        speed = 5.0
        omega = speed / radius

        x = radius * math.cos(omega * t)
        y = radius * math.sin(omega * t)
        vx = -radius * omega * math.sin(omega * t)
        vy =  radius * omega * math.cos(omega * t)

        # 位姿消息
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        self.pose_pub.publish(pose_msg)

        # 速度消息
        vel_msg = TwistStamped()
        vel_msg.header = pose_msg.header
        vel_msg.twist.linear.x = vx
        vel_msg.twist.linear.y = vy
        self.vel_pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        SimulateUSVInputs()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
