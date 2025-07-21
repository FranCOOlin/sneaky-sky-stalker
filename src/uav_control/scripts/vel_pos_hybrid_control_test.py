#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode, CommandBool
import threading

class OffboardYawRateControlNode:
    def __init__(self):
        rospy.init_node("offboard_mixed_yawrate_node")

        # 当前飞控状态
        self.current_state = State()
        rospy.Subscriber("/mavros/state", State, self.state_cb)

        # 发布 setpoint
        self.sp_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)

        # 服务
        rospy.wait_for_service("/mavros/set_mode")
        rospy.wait_for_service("/mavros/cmd/arming")
        self.set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

        # 控制线程
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()

    def state_cb(self, msg):
        self.current_state = msg

    def control_loop(self):
        rate = rospy.Rate(20)  # 20 Hz
        rospy.sleep(1.0)  # 等待系统初始化

        # 准备 setpoint
        sp = PositionTarget()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW  # 不忽略 yaw_rate（启用）
        )

        sp.position.z = 25.0           # 控制高度为 25 米
        sp.velocity.x = 20.0        # 向东飞 1m/s,mavros下的local就是NEU
        sp.velocity.y = 0
        sp.yaw_rate = 0             # 匀速旋转 0.5 rad/s（约 28.6 deg/s）

        # 等待 FCU 连接
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.logwarn_throttle(2.0, "[OFFBOARD] Waiting for FCU connection...")
            rate.sleep()

        # 发送预热 setpoint 保证 PX4 接收到了
        rospy.loginfo("[OFFBOARD] Sending preflight setpoints...")
        for _ in range(20):
            sp.header.stamp = rospy.Time.now()
            self.sp_pub.publish(sp)
            rate.sleep()

        # 设置模式为 OFFBOARD
        if self.current_state.mode != "OFFBOARD":
            resp = self.set_mode_srv(0, "OFFBOARD")
            if resp.mode_sent:
                rospy.loginfo("[OFFBOARD] Mode set to OFFBOARD.")
            else:
                rospy.logwarn("[OFFBOARD] Failed to set mode.")

        # 解锁
        if not self.current_state.armed:
            resp = self.arming_srv(True)
            if resp.success:
                rospy.loginfo("[OFFBOARD] UAV armed.")
            else:
                rospy.logwarn("[OFFBOARD] Arming failed.")

        # 主循环：持续发布 setpoint
        rospy.loginfo("[OFFBOARD] Control loop started.")
        while not rospy.is_shutdown() and self.running:
            sp.header.stamp = rospy.Time.now()
            self.sp_pub.publish(sp)
            rate.sleep()

    def shutdown(self):
        rospy.loginfo("[OFFBOARD] Node shutdown initiated.")
        self.running = False
        self.control_thread.join()


if __name__ == "__main__":
    try:
        node = OffboardYawRateControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        node.shutdown()
