#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

def image_callback(msg):
    bridge = CvBridge()
    try:
        print("Received image message")
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite("/home/nvidia/sneaky-sky-stalker/tmp/image.jpg", cv_image)
    except Exception as e:
        rospy.logerr("Failed to convert image message: %s", e)

def main():
    print("Starting image subscriber node")
    rospy.init_node('image_saver', anonymous=True)
    rospy.Subscriber("/gimbal_camera/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
