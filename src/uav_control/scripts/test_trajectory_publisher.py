#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from fsm.msg import FSMStateStamped

class Figure8TrajectoryPublisher:
    def __init__(self):
        rospy.init_node("figure8_trajectory_publisher")

        self.pub = rospy.Publisher("/uav/target_trajectory_local", Twist, queue_size=1)
        self.fsm_state = "UNKNOWN"

        rospy.Subscriber("/fsm/state", FSMStateStamped, self.fsm_cb)

        self.rate = rospy.Rate(20)
        self.start_time = rospy.Time.now().to_sec()

        self.running = True
        self.main_loop()

    def fsm_cb(self, msg):
        self.fsm_state = msg.state

    def main_loop(self):
        while not rospy.is_shutdown() and self.running:
            if self.fsm_state != "UAV_SEARCHING":
                self.rate.sleep()
                continue

            t = rospy.Time.now().to_sec() - self.start_time

            # Figure-8 trajectory: lemniscate of Gerono
            A = 100.0  # amplitude x
            B = 50.0   # amplitude y
            w = 0.06   # angular frequency (rad/s)

            x = A * np.sin(w * t)
            y = B * np.sin(w * t) * np.cos(w * t)
            dx = A * w * np.cos(w * t)
            dy = B * w * (np.cos(2 * w * t) - np.sin(2 * w * t)) / 2.0

            yaw = np.arctan2(dy, dx)
            dyaw = 0.0  # 可加入一阶导数但保持为0也可行

            msg = Twist()
            msg.linear.x = x
            msg.linear.y = y
            msg.linear.z = yaw  # 用 linear.z 表示 yaw
            msg.angular.x = dx
            msg.angular.y = dy
            msg.angular.z = dyaw  # yaw 速率

            self.pub.publish(msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        Figure8TrajectoryPublisher()
    except rospy.ROSInterruptException:
        pass
