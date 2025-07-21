#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Int32, Header
from fsm.msg import FSMStateStamped
from scipy.integrate import odeint
from scipy.linalg import solve_continuous_are
from uav_srv.srv import LLA2ENU  # Replace with actual service if needed

class KalmanObserver:
    def __init__(self, target_id):
        self.target_id = rospy.get_param("~target_id", target_id)
        self.node_name = f"target{self.target_id}_observer"
        rospy.init_node(self.node_name)

        # 参数
        self.Q = np.eye(4) * rospy.get_param("~Q_scale", 0.01)
        self.R = np.eye(2) * rospy.get_param("~R_scale", 1.0)
        self.H = np.hstack([np.eye(2), np.zeros((2,2))])  # 只测量位置
        self.rate_hz = rospy.get_param("~rate", 20.0)
        self.dt = 1.0 / self.rate_hz

        # 系统模型
        self.A = np.block([
            [np.zeros((2, 2)), np.eye(2)],
            [np.zeros((2, 2)), np.zeros((2, 2))]
        ])

        # 求解连续李卡提方程
        P = solve_continuous_are(self.A.T, self.H.T, self.Q, self.R)
        self.K = P @ self.H.T @ np.linalg.inv(self.R)
        rospy.loginfo(f"[{self.node_name}] Kalman gain K: {self.K}")
        rospy.loginfo(f"[{self.node_name}] Riccati solution P: {P}")


        # 状态初始化
        self.x = np.zeros(4)
        self.fsm_state = ""
        self.initialized = False
        self.last_meas_time = None

        # 订阅
        rospy.Subscriber("/fsm/state", FSMStateStamped, self.fsm_cb)
        rospy.Subscriber(f"/target{self.target_id}/vision_position_local", PoseStamped, self.measure_cb)
        rospy.Subscriber(f"/command/target{self.target_id}_location", NavSatFix, self.loc_cb)
        rospy.Subscriber(f"/command/target{self.target_id}_velocity", TwistStamped, self.vel_cb)

        # 服务客户端
        rospy.wait_for_service("/lla2enu")
        self.lla2enu_srv = rospy.ServiceProxy("/lla2enu", LLA2ENU)

        # 发布估计
        self.pub = rospy.Publisher(f"/target{self.target_id}/state_estimate", TwistStamped, queue_size=1)
        self.rate = rospy.Rate(self.rate_hz)

        self.has_init_pos = False
        self.has_init_vel = False
        self.init_pos = np.zeros(2)
        self.init_vel = np.zeros(2)

        self.run()

    def fsm_cb(self, msg):
        self.fsm_state = msg.state

    def loc_cb(self, msg):
        enu = self.lla2enu_srv(msg).point
        self.init_pos = np.array([enu.x, enu.y])
        self.has_init_pos = True

    def vel_cb(self, msg):
        self.init_vel = np.array([msg.twist.linear.x, msg.twist.linear.y])
        self.has_init_vel = True

    def measure_cb(self, msg):
        self.latest_meas = np.array([msg.pose.Point.x, msg.pose.Point.y])
        self.has_meas = True
        self.last_meas_time = msg.header.stamp.to_sec()

    def dynamics(self, x, t):
        return (self.A @ x).tolist()

    def run(self):
        last_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            dt = current_time - last_time
            last_time = current_time

            if self.fsm_state == "UAV_SEARCHING" and not self.initialized and self.has_init_pos and self.has_init_vel:
                self.x = np.hstack([self.init_pos, self.init_vel])
                self.initialized = True
                rospy.loginfo(f"[{self.node_name}] Initialized observer")

            if not self.initialized:
                self.rate.sleep()
                continue

            if not hasattr(self, 'last_meas_time') or self.last_meas_time is None:
                self.rate.sleep()
                continue

            # 预测
            t_span = [0, dt]
            self.x = odeint(self.dynamics, self.x, t_span)[-1]

            if hasattr(self, 'has_meas') and self.has_meas:
                y = self.latest_meas
                y_hat = self.H @ self.x
                self.x = self.x + self.K @ (y - y_hat)
                self.has_meas = False

            # 发布估计
            msg = TwistStamped()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.twist.linear.x = self.x[0]
            msg.twist.linear.y = self.x[1]
            msg.twist.linear.z = 0.0
            msg.twist.angular.x = self.x[2]
            msg.twist.angular.y = self.x[3]
            msg.twist.angular.z = 0.0
            self.pub.publish(msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        KalmanObserver(target_id=1)
    except rospy.ROSInterruptException:
        pass
