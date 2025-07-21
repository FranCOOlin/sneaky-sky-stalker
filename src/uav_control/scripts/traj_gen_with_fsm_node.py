#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from fsm.msg import FSMStateStamped
from std_msgs.msg import Bool, Int32

class TargetTrajectoryGenerator:
    def __init__(self):
        rospy.init_node("target_trajectory_generator")

        # 参数
        self.H = rospy.get_param("~H", 10.0)  # 前视距离因子
        self.unit_vector = np.array(rospy.get_param("~dir", [1.0, 0.0, 0.0]))  # 单位方向向量（默认正东）
        self.unit_vector = self.unit_vector / np.linalg.norm(self.unit_vector)

        # 状态量
        self.Pusv1 = np.zeros(3)
        self.Vusv1 = np.zeros(3)
        self.Pusv2 = np.zeros(3)
        self.Vusv2 = np.zeros(3)
        self.Puav = np.zeros(3)
        self.fsm_state = "UNKNOWN"
        self.target1_detected = False
        self.target2_detected = False
        self.target1_type = -1
        self.target2_type = -1
        self.target1_confirmed = False
        self.target2_confirmed = False

        # 订阅者
        rospy.Subscriber("/target1/hat_state", TwistStamped, self.usv1_state_cb)
        rospy.Subscriber("/target2/hat_state", TwistStamped, self.usv2_state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.uav_pose_cb)
        rospy.Subscriber("/fsm/state", FSMStateStamped, self.fsm_state_cb)
        rospy.Subscriber("/target1/detected", Bool, self.target1_detected_cb)
        rospy.Subscriber("/target2/detected", Bool, self.target2_detected_cb)
        rospy.Subscriber("/target1/type", Int32, self.target1_type_cb)
        rospy.Subscriber("/target2/type", Int32, self.target2_type_cb)
        rospy.Subscriber("/target1/confirm", Bool, self.target1_confirm_cb)
        rospy.Subscriber("/target2/confirm", Bool, self.target2_confirm_cb)

        # 发布者
        self.pub = rospy.Publisher("/uav/target_trajectory_local", Twist, queue_size=1)

        self.rate = rospy.Rate(50)
        self.run()

    def usv1_state_cb(self, msg):
        # 使用 TwistStamped 消息来包含目标1的位置（linear）和速度（angular）
        self.Pusv1 = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        self.Vusv1 = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])

    def usv2_state_cb(self, msg):
        # 使用 TwistStamped 消息来包含目标2的位置（linear）和速度（angular）
        self.Pusv2 = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        self.Vusv2 = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])

    def uav_pose_cb(self, msg):
        self.Puav = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def fsm_state_cb(self, msg):
        self.fsm_state = msg.state

    def target1_detected_cb(self, msg):
        self.target1_detected = msg.data

    def target2_detected_cb(self, msg):
        self.target2_detected = msg.data

    def target1_type_cb(self, msg):
        self.target1_type = msg.data

    def target2_type_cb(self, msg):
        self.target2_type = msg.data

    def target1_confirm_cb(self, msg):
        self.target1_confirmed = msg.data

    def target2_confirm_cb(self, msg):
        self.target2_confirmed = msg.data

    def determine_trajectory(self):
        pd = self.Puav
        vd = np.zeros(3)
        yaw_vec = np.array([1.0, 0.0])

        if self.fsm_state == "UAV_SEARCHING":
            # Searching状态下，UAV朝向目标1或目标2, 位置和速度为0
            pd = np.zeros(3)
            vd = np.zeros(3)
            yaw_vec = np.array([1.0, 0.0])  # 默认朝向正东  
            # yaw 朝向目标1（优先）或目标2
            if self.target1_type == -1:
                yaw_vec = self.Pusv1[:2] - self.Puav[:2]
            elif self.target2_type == -1:
                yaw_vec = self.Pusv2[:2] - self.Puav[:2]
        elif self.fsm_state == "UAV_WAIT_TARGET_CONFIRM":
            pass

        elif self.fsm_state in ["UAV_RELOCATING","UAV_TRACKING"]:
            if self.target1_confirmed: # 如果确认追踪目标1
                pd = self.Pusv1 + self.H * self.unit_vector
                vd = self.Vusv1
                yaw_vec = self.Pusv1[:2] - self.Puav[:2]
            else:
                pd = self.Pusv2 + self.H * self.unit_vector
                vd = self.Vusv2
                yaw_vec = self.Pusv2[:2] - self.Puav[:2]

        yaw = np.arctan2(yaw_vec[1], yaw_vec[0]) if np.linalg.norm(yaw_vec) > 1e-3 else 0.0
        return pd, vd, yaw

    def run(self):
        while not rospy.is_shutdown():
            pd, vd, yaw = self.determine_trajectory()
            msg = Twist()
            msg.linear.x = pd[0]
            msg.linear.y = pd[1]
            msg.linear.z = yaw
            msg.angular.x = vd[0]
            msg.angular.y = vd[1]
            msg.angular.z = 0.0
            self.pub.publish(msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        TargetTrajectoryGenerator()
    except rospy.ROSInterruptException:
        pass
