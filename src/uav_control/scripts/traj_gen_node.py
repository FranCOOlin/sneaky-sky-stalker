#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from std_msgs.msg import Header

class TargetTrajectoryGenerator:
    def __init__(self):
        rospy.init_node("target_trajectory_generator")

        # 参数
        self.H = rospy.get_param("~H", 10.0)  # 前视距离因子（正东方向）
        self.unit_vector = np.array(rospy.get_param("~dir", [1.0, 0.0, 0.0]))  # 单位向量方向(默认正东方向)
        self.unit_vector = self.unit_vector / np.linalg.norm(self.unit_vector)
        # 状态
        self.Pusv = np.zeros(3)
        self.Vusv = np.zeros(3)
        self.Puav = np.zeros(3)

        # 订阅
        rospy.Subscriber("/usv/position_local", PoseStamped, self.usv_pose_cb)
        rospy.Subscriber("/usv/velocity_local", TwistStamped, self.usv_vel_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.uav_pose_cb)

        # 发布
        self.pub = rospy.Publisher("/uav/target_trajectory_local", Twist, queue_size=1)

        self.rate = rospy.Rate(50)
        self.run()

    def usv_pose_cb(self, msg):
        self.Pusv = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def usv_vel_cb(self, msg):
        self.Vusv = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

    def uav_pose_cb(self, msg):
        self.Puav = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def run(self):
        while not rospy.is_shutdown():
            pd = self.Pusv + self.H * self.unit_vector
            vd = self.Vusv

            rel_vec = self.Pusv - self.Puav
            yaw = np.arctan2(rel_vec[1], rel_vec[0])

            msg = Twist()
            msg.linear.x = pd[0]
            msg.linear.y = pd[1]
            msg.linear.z = yaw
            msg.angular.x = vd[0]
            msg.angular.y = vd[1]
            msg.angular.z = 0.0  # 可选设置 yaw_rate

            self.pub.publish(msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        TargetTrajectoryGenerator()
    except rospy.ROSInterruptException:
        pass