#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import TwistStamped
from fsm.msg import FSMStateStamped
import threading
import tf.transformations as tf_trans

class DecoupledVelocityYawControlNode:
    def __init__(self):
        rospy.init_node("decoupled_velocity_yaw_controller")

        self.current_state = State()
        self.pose = None
        self.velocity = np.zeros(3)
        self.z_height = rospy.get_param("~target_z", 55.0)
        self.fsm_state = "UNKNOWN"

        self.goal_pos = np.zeros(3)   # x, y, yaw
        self.goal_vel = np.zeros(3)

        self.kp = rospy.get_param("~kp", [1.2, 1.2, 1.0])
        self.kv = rospy.get_param("~kv", [0.8, 0.8, 0.6])
        self.Ts = rospy.get_param("~T", [1.0, 1.0, 1.0])  # 每个分量独立 T

        rospy.Subscriber("/mavros/state", State, self.state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.velocity_cb)
        rospy.Subscriber("/uav/target_trajectory_local", Twist, self.target_cb)
        rospy.Subscriber("/fsm/state", FSMStateStamped, self.fsm_state_cb)

        self.sp_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)

        rospy.wait_for_service("/mavros/set_mode")
        rospy.wait_for_service("/mavros/cmd/arming")
        self.set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

        self.running = True
        self.thread = threading.Thread(target=self.control_loop)
        self.thread.start()

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        pos = msg.pose.position
        quat = msg.pose.orientation
        _, _, yaw = tf_trans.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.pose = np.array([pos.x, pos.y, yaw])

    def velocity_cb(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        wz = msg.twist.angular.z
        self.velocity = np.array([vx, vy, wz])

    def target_cb(self, msg):
        self.goal_pos[0] = msg.linear.x
        self.goal_pos[1] = msg.linear.y
        self.goal_pos[2] = msg.linear.z
        self.goal_vel[0] = msg.angular.x
        self.goal_vel[1] = msg.angular.y
        self.goal_vel[2] = msg.angular.z

    def fsm_state_cb(self, msg):
        self.fsm_state = msg.state

    def saturate(self, value, limit):
        return max(min(value, limit), -limit)

    def control_loop(self):
        rate = rospy.Rate(50)
        rospy.sleep(1.0)

        sp = PositionTarget()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW  # 注意：不要忽略 yaw_rate
        )
        sp.position.z = self.z_height
        sp.velocity.x = 0.0
        sp.velocity.y = 0.0
        sp.yaw_rate = 0.0

        while not rospy.is_shutdown():
            if self.fsm_state != "UAV_SEARCHING":
                # rospy.loginfo_throttle(2.0, f"[Controller] Waiting for FSM state 'UAV_SEARCHING', current: {self.fsm_state}")
                rate.sleep()
                continue

            if not self.current_state.connected:
                rospy.loginfo_throttle(2.0, "[Controller] Waiting for FCU connection...")
                rate.sleep()
                continue

            for _ in range(20):
                sp.header.stamp = rospy.Time.now()
                self.sp_pub.publish(sp)
                rate.sleep()

            if self.current_state.mode != "OFFBOARD":
                res = self.set_mode_srv(0, "OFFBOARD")
                if res.mode_sent:
                    rospy.loginfo("[Controller] OFFBOARD mode set.")

            if not self.current_state.armed:
                res = self.arming_srv(True)
                if res.success:
                    rospy.loginfo("[Controller] UAV armed.")

            rospy.loginfo("[Controller] Control loop started.")
            break

        while not rospy.is_shutdown() and self.running:

            if self.pose is None:
                rate.sleep()
                continue

            p = self.pose
            v = self.velocity

            # 独立控制 x
            zp_x = p[0] - self.goal_pos[0]
            dzp_x = v[0] - self.goal_vel[0]
            dvd_x = -self.kp[0] * dzp_x
            vd_x = self.goal_vel[0] - self.kp[0] * zp_x
            zv_x = v[0] - vd_x
            u_x = v[0] - self.Ts[0] * zp_x + self.Ts[0] * dvd_x - self.Ts[0] * self.kv[0] * zv_x
            u_x = self.saturate(u_x, 15.0)

            # 独立控制 y
            zp_y = p[1] - self.goal_pos[1]
            dzp_y = v[1] - self.goal_vel[1]
            dvd_y = -self.kp[1] * dzp_y
            vd_y = self.goal_vel[1] - self.kp[1] * zp_y
            zv_y = v[1] - vd_y
            u_y = v[1] - self.Ts[1] * zp_y + self.Ts[1] * dvd_y - self.Ts[1] * self.kv[1] * zv_y
            u_y = self.saturate(u_y, 15.0)

            # 独立控制 yaw
            yaw_error = (p[2] - self.goal_pos[2] + np.pi) % (2 * np.pi) - np.pi
            dzp_yaw = v[2] - self.goal_vel[2]
            dvd_yaw = -self.kp[2] * dzp_yaw
            vd_yaw = self.goal_vel[2] - self.kp[2] * yaw_error
            zv_yaw = v[2] - vd_yaw
            u_yaw = v[2] - self.Ts[2] * yaw_error + self.Ts[2] * dvd_yaw - self.Ts[2] * self.kv[2] * zv_yaw
            u_yaw = self.saturate(u_yaw, 6.0)

            sp.header.stamp = rospy.Time.now()
            sp.position.z = self.z_height
            sp.velocity.x = u_x
            sp.velocity.y = u_y
            sp.yaw_rate = u_yaw

            self.sp_pub.publish(sp)
            # rospy.loginfo(f"[Controller] u_x: {u_x:.2f}, u_y: {u_y:.2f}, u_yaw: {u_yaw:.2f}")
            rate.sleep()

    def shutdown(self):
        self.running = False
        self.thread.join()

if __name__ == '__main__':
    try:
        node = DecoupledVelocityYawControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        node.shutdown()