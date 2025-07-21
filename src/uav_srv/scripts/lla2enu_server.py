#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from uav_srv.srv import LLA2ENU, LLA2ENUResponse
import pymap3d as pm

class LLA2ENUServer:
    def __init__(self):
        rospy.init_node("lla2enu_server")

        self.ref_received = False
        self.ref_lat = None
        self.ref_lon = None
        self.ref_alt = None

        rospy.Subscriber("/fsm/uav_init_position", NavSatFix, self.ref_callback)
        self.service = rospy.Service("/lla2enu", LLA2ENU, self.handle_request)

        rospy.loginfo("[LLA2ENU] Waiting for UAV initial position as reference...")
        rospy.spin()

    def ref_callback(self, msg):
        if not self.ref_received and self.valid_fix(msg):
            self.ref_lat = msg.latitude
            self.ref_lon = msg.longitude
            self.ref_alt = msg.altitude
            self.ref_received = True
            rospy.loginfo(f"[LLA2ENU] Set ENU origin at lat={self.ref_lat}, lon={self.ref_lon}, alt={self.ref_alt}")

    def valid_fix(self, msg):
        return not np.isnan(msg.latitude) and not np.isnan(msg.longitude)

    def handle_request(self, req):
        if not self.ref_received:
            rospy.logwarn("[LLA2ENU] Reference origin not yet initialized!")
            return LLA2ENUResponse(Point(x=0, y=0, z=0))

        gps = req.gps
        e, n, u = pm.geodetic2enu(gps.latitude, gps.longitude, gps.altitude,
                                  self.ref_lat, self.ref_lon, self.ref_alt)
        pt = Point(x=e, y=n, z=u)
        return LLA2ENUResponse(point=pt)

if __name__ == '__main__':
    try:
        LLA2ENUServer()
    except rospy.ROSInterruptException:
        pass

