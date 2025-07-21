#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from fsm.msg import FSMStateStamped
from nav_msgs.msg import Odometry  # 用于当前高度判断

class TakeoffNode:
    def __init__(self):
        rospy.init_node("takeoff_node")

        # 状态变量
        self.fsm_state = "UNKNOWN"
        self.current_mode = ""
        self.armed = False
        self.connected = False
        self.current_altitude = 0.0

        # 参数
        self.target_height = rospy.get_param("~target_height", 50.0)
        self.alt_tolerance = rospy.get_param("~alt_tolerance", 0.2)

        # 订阅
        rospy.Subscriber("/fsm/state", FSMStateStamped, self.fsm_cb)
        rospy.Subscriber("/mavros/state", State, self.state_cb)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_cb)

        # 发布 setpoint
        self.setpoint_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)

        # 发布是否起飞完成
        self.takeoff_done_pub = rospy.Publisher("/fsm/uav_took_off", Bool, queue_size=1)

        # 服务客户端
        rospy.wait_for_service("/mavros/set_mode")
        rospy.wait_for_service("/mavros/cmd/arming")
        self.set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

        self.rate = rospy.Rate(20)

    def fsm_cb(self, msg):
        self.fsm_state = msg.state

    def state_cb(self, msg):
        self.current_mode = msg.mode
        self.armed = msg.armed
        self.connected = msg.connected

    def odom_cb(self, msg):
        self.current_altitude = msg.pose.pose.position.z

    def send_setpoint(self, z_height):
        sp = PoseStamped()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = "map"
        sp.pose.position.x = 0.0
        sp.pose.position.y = 0.0
        sp.pose.position.z = z_height
        sp.pose.orientation.w = 1.0
        self.setpoint_pub.publish(sp)

    def run(self):
        # rospy.loginfo("[Takeoff] Node running. Waiting for FSM=UAV_TAKEOFF...")

        # 必须先发送一段 setpoint，才能进入 OFFBOARD
        for _ in range(30):
            self.send_setpoint(self.target_height)
            self.rate.sleep()

        while not rospy.is_shutdown():
            if self.fsm_state != "UAV_TAKEOFF":
                self.rate.sleep()
                continue

            if not self.connected:
                rospy.logwarn_throttle(5.0, "[Takeoff] FCU not connected.")
                self.rate.sleep()
                continue

            # 切模式
            if self.current_mode != "OFFBOARD":
                try:
                    self.set_mode_srv(0, "OFFBOARD")
                    rospy.loginfo_throttle(5.0, "[Takeoff] Switching to OFFBOARD mode.")
                except rospy.ServiceException as e:
                    rospy.logerr(f"[Takeoff] set_mode service call failed: {e}")

            # 解锁
            if not self.armed:
                try:
                    self.arming_srv(True)
                    rospy.loginfo_throttle(5.0, "[Takeoff] Sending arm command.")
                except rospy.ServiceException as e:
                    rospy.logerr(f"[Takeoff] arming service call failed: {e}")

            # 发送 setpoint
            self.send_setpoint(self.target_height)

            # 检查高度
            if abs(self.current_altitude - self.target_height) < self.alt_tolerance:
                rospy.loginfo("[Takeoff] Reached target altitude. Takeoff complete.")
                self.takeoff_done_pub.publish(Bool(data=True))
                break

            self.rate.sleep()

        rospy.loginfo("[Takeoff] Node finished.")


if __name__ == '__main__':
    try:
        node = TakeoffNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
