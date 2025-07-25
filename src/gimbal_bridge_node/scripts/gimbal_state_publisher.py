#!/usr/bin/env python3
import rospy
from gimbal_bridge_node.msg import GimbalState

def publish_gimbal_state():
    pub = rospy.Publisher('/gimbal/state', GimbalState, queue_size=10)
    rospy.init_node('gimbal_state_publisher')
    rate = rospy.Rate(1)  # 1Hz 发布频率

    while not rospy.is_shutdown():
        msg = GimbalState()
        msg.yaw = 10.5     # 示例数据
        msg.pitch = -5.3
        msg.roll = 2.7
        msg.json_string = '{"status":"normal"}'
        pub.publish(msg)
        rospy.loginfo("Published: yaw=%.1f, pitch=%.1f, roll=%.1f, json=%s", 
                      msg.yaw, msg.pitch, msg.roll, msg.json_string)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_gimbal_state()
    except rospy.ROSInterruptException:
        pass

