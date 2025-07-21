#!/usr/bin/env python3
from fsm.msg import FSMStateStamped
import rospy
from std_msgs.msg import String, Bool

class FSMNode:
    def __init__(self):
        rospy.init_node("fsm_node")

        # === 状态变量 ===
        self.current_state = "INIT"
        self.previous_state = "INIT"

        # === 触发条件变量（与你的逻辑完全对应） ===
        self.initialized = False  # 用于标记节点是否初始化
        self.start_signal_received = False
        self.enemy_hint_received = False
        self.uav_armed = False
        self.uav_took_off = False
        self.start_searching = False
        self.target_detected = False
        self.target_confirmed = False
        self.tracking_lost = False
        self.mission_completed = False
        self.returned_home = False
        self.landed = False

        # === 发布 FSM 状态 ===
        self.state_pub = rospy.Publisher("/fsm/state", FSMStateStamped, queue_size=10)

        # === 订阅消息 ===
        rospy.Subscriber("/fsm/initialized", Bool, self.cb_initialized)
        rospy.Subscriber("/fsm/start_signal", Bool, self.cb_start_signal)
        rospy.Subscriber("/fsm/enemy_hint", Bool, self.cb_enemy_hint)
        rospy.Subscriber("/fsm/uav_took_off", Bool, self.cb_uav_took_off)
        rospy.Subscriber("/fsm/start_searching", Bool, self.cb_start_searching)
        rospy.Subscriber("/fsm/target_detected", Bool, self.cb_target_detected)
        rospy.Subscriber("/fsm/target_confirmed", Bool, self.cb_target_confirmed)
        rospy.Subscriber("/fsm/mission_completed", Bool, self.cb_mission_completed)
        rospy.Subscriber("/fsm/returned_home", Bool, self.cb_returned_home)
        rospy.Subscriber("/fsm/landed", Bool, self.cb_landed)

        self.rate = rospy.Rate(10)

    # === 各种回调函数 ===
    def cb_initialized(self, msg): self.initialized = msg.data
    def cb_start_signal(self, msg): self.start_signal_received = msg.data
    def cb_enemy_hint(self, msg): self.enemy_hint_received = msg.data
    def cb_uav_took_off(self, msg): self.uav_took_off = msg.data
    def cb_start_searching(self, msg): self.start_searching = msg.data
    def cb_target_detected(self, msg): self.target_detected = msg.data
    def cb_target_confirmed(self, msg): self.target_confirmed = msg.data
    def cb_mission_completed(self, msg): self.mission_completed = msg.data
    def cb_returned_home(self, msg): self.returned_home = msg.data
    def cb_landed(self, msg): self.landed = msg.data

    # === 发布状态 ===
    def publish_state(self):
        msg = FSMStateStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "fsm"
        msg.state = self.current_state
        self.state_pub.publish(msg)

    # === 切换状态 ===
    def transition_to(self, new_state):
        if new_state != self.current_state:
            rospy.loginfo(f"[FSM] {self.current_state} → {new_state}")
            self.previous_state = self.current_state
            self.current_state = new_state

    # === 状态更新逻辑（核心） ===
    def update_state(self):
        s = self.current_state

        if s == "INIT":
            if self.initialized:
                self.transition_to("WAIT_FOR_START")

        elif s == "WAIT_FOR_START":
            if self.start_signal_received:
                self.transition_to("WAIT_FOR_ENEMY_ENTRY")

        elif s == "WAIT_FOR_ENEMY_ENTRY":
            if self.enemy_hint_received:
                self.transition_to("UAV_TAKEOFF")

        elif s == "UAV_TAKEOFF":
            if self.uav_took_off:
                self.transition_to("UAV_READY")

        elif s == "UAV_READY":
            if self.start_searching:
                self.transition_to("UAV_SEARCHING")

        elif s == "UAV_SEARCHING":
            if self.target_detected:
                self.transition_to("UAV_WAIT_TARGET_CONFIRM")

        elif s == "UAV_WAIT_TARGET_CONFIRM":
            if self.target_confirmed:
                self.transition_to("UAV_TRACKING")

        elif s == "UAV_TRACKING":
            if not self.target_detected:
                self.transition_to("UAV_TARGET_LOST")
            elif self.mission_completed:
                self.transition_to("UAV_RETURN_HOME")

        elif s == "UAV_TARGET_LOST":
            if self.target_detected:
                self.transition_to("UAV_TRACKING")

        elif s == "UAV_RETURN_HOME":
            if self.returned_home:
                self.transition_to("UAV_LAND")

        elif s == "UAV_LAND":
            if self.landed:
                self.transition_to("DONE")

        elif s == "DONE":
            rospy.loginfo_throttle(5.0, "[FSM] Mission complete. Waiting for reset...")
        self.publish_state()
        
    def run(self):
        rospy.loginfo("[FSM] Node started.")
        self.publish_state()
        while not rospy.is_shutdown():
            self.update_state()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = FSMNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
