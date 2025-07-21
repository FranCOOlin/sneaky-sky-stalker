#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from fsm.msg import FSMStateStamped
from collections import deque  # 用于缓存 GPS 数据

class InitCheckNode:
    def __init__(self):
        rospy.init_node("init_check_node")

        self.fsm_state = "UNKNOWN"
        self.gps_fix = False
        self.gps_queue = deque(maxlen=5)  # 保存最近5次GPS
        self.length = 5
        self.has_local_pose = False
        self.current_mode = ""
        self.connected = False
        self.armed = False

        # 订阅
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_cb)
        rospy.Subscriber("/mavros/state", State, self.state_cb)
        rospy.Subscriber("/fsm/state", FSMStateStamped, self.fsm_state_cb)

        # 发布
        self.start_pub = rospy.Publisher("/fsm/initialized", Bool, queue_size=1)
        self.gps_pub = rospy.Publisher("/fsm/uav_init_position", NavSatFix, queue_size=1)

        self.rate = rospy.Rate(2)

    # === 回调 ===
    def gps_cb(self, msg):
        if msg.status.status >= 0:
            self.gps_fix = True
            self.gps_queue.append(msg)  # 添加到缓存队列
        else:
            self.gps_fix = False
            self.gps_queue.clear()

    def local_pose_cb(self, msg):
        self.has_local_pose = True

    def state_cb(self, msg):
        self.current_mode = msg.mode
        self.connected = msg.connected
        self.armed = msg.armed

    def fsm_state_cb(self, msg):
        self.fsm_state = msg.state

    # === 平均 GPS ===
    def average_gps(self):
        if len(self.gps_queue) < self.length:
            return None
        avg = NavSatFix()
        avg.status.status = 0
        avg.header.stamp = rospy.Time.now()
        avg.header.frame_id = "init"

        avg.latitude = sum([p.latitude for p in self.gps_queue]) / self.length
        avg.longitude = sum([p.longitude for p in self.gps_queue]) / self.length
        avg.altitude = sum([p.altitude for p in self.gps_queue]) / self.length
        return avg

    # === 主循环 ===
    def run(self):
        # rospy.loginfo("[InitCheck] Node running. Waiting for FSM=INIT...")

        while not rospy.is_shutdown():
            if self.fsm_state != "INIT":
                self.rate.sleep()
                continue

            if not self.connected:
                rospy.logwarn_throttle(5.0, "[InitCheck] FCU not connected.")
                self.rate.sleep()
                continue

            gps_ready = self.gps_fix and len(self.gps_queue) == self.length
            mode_ok = self.current_mode in ["POSCTL", "OFFBOARD"]

            all_ready = gps_ready and self.has_local_pose and mode_ok

            if all_ready:
                avg_gps = self.average_gps()
                if avg_gps:
                    rospy.loginfo("[InitCheck] All checks passed. Publishing init position + start signal.")
                    self.gps_pub.publish(avg_gps)
                    self.start_pub.publish(Bool(data=True))
                    break
            else:
                rospy.loginfo_throttle(2.0,
                    f"[InitCheck] GPS OK: {gps_ready}, LocalPose: {self.has_local_pose}, Mode: {self.current_mode}")
                self.start_pub.publish(Bool(data=False))

            self.rate.sleep()

        rospy.loginfo("[InitCheck] Node finished.")

if __name__ == "__main__":
    try:
        node = InitCheckNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
