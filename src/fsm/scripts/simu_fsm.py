#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import time

class FSMSignalPublisher:
    def __init__(self):
        rospy.init_node("fsm_signal_publisher")

        self.start_pub = rospy.Publisher("/fsm/start_signal", Bool, queue_size=1)
        self.hint_pub = rospy.Publisher("/fsm/enemy_hint", Bool, queue_size=1)
        self.search_pub = rospy.Publisher("/fsm/start_searching", Bool, queue_size=1)

        rospy.sleep(1.0)  # 等待 publisher 初始化
        self.publish_sequence()

    def publish_sequence(self):
        rospy.loginfo("[FSM Simulator] Publishing start_signal")
        self.start_pub.publish(Bool(data=True))
        rospy.sleep(2.0)

        rospy.loginfo("[FSM Simulator] Publishing enemy_hint")
        self.hint_pub.publish(Bool(data=True))
        rospy.sleep(2.0)

        rospy.loginfo("[FSM Simulator] Publishing start_searching")
        self.search_pub.publish(Bool(data=True))
        rospy.loginfo("[FSM Simulator] All messages sent.")

if __name__ == '__main__':
    try:
        FSMSignalPublisher()
    except rospy.ROSInterruptException:
        pass
