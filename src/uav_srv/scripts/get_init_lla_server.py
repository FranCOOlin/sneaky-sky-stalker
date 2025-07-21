#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from uav_srv.srv import GetInitLLA, GetInitLLAResponse
import numpy as np

class InitLLAServer:
    def __init__(self):
        rospy.init_node("get_init_lla_server")

        self.init_fix = None
        self.ref_acquired = False

        # 订阅 UAV 全球坐标
        rospy.Subscriber("/fsm/uav_init_position", NavSatFix, self.gps_cb)
        rospy.loginfo("[GetInitLLA] Waiting for initial GPS fix...")

        # 注册服务
        self.service = rospy.Service("/get_init_lla", GetInitLLA, self.handle_request)

        rospy.spin()

    def gps_cb(self, msg):
        if not self.ref_acquired and self.valid_fix(msg):
            self.init_fix = msg
            self.ref_acquired = True
            rospy.loginfo(f"[GetInitLLA] Initial GPS fix saved: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}")

    def valid_fix(self, msg):
        return (
            msg.status.status >= 0 and
            not any(np.isnan(x) for x in [msg.latitude, msg.longitude, msg.altitude])
        )

    def handle_request(self, req):
        if self.ref_acquired:
            return GetInitLLAResponse(gps=self.init_fix)
        else:
            rospy.logwarn("[GetInitLLA] Reference not yet initialized.")
            return GetInitLLAResponse()  # 空 GPS 消息

if __name__ == "__main__":
    try:
        InitLLAServer()
    except rospy.ROSInterruptException:
        pass
