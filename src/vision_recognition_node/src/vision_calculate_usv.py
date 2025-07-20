#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from detection_msgs.msg import BoundingBoxes, BoundingBox

class BboxENUConverter:
    def __init__(self):
      
        self.enu_pose = None 

        self.mavros_pose_sub = rospy.Subscriber(
            "/mavros/local_position/local",
            PoseStamped,
            self.mavros_pose_callback,
            queue_size=1
        )


        self.bbox_sub = rospy.Subscriber(
            rospy.get_param("~input_bbox_topic", "/yolov5/detections"),
            BoundingBoxes,
            self.bbox_callback,
            queue_size=1
        )

        self.enu_bbox_pub = rospy.Publisher(
            rospy.get_param("~output_bbox_topic", "/yolov5/detections_enu"),
            BoundingBoxes,
            queue_size=10
        )

        rospy.loginfo("Bbox ENU converter initialized.")

    def mavros_pose_callback(self, msg):

        self.enu_pose = {
            "position": [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            "orientation": [  # Quaternion (x, y, z, w)
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]
        }

    def quaternion_to_rotation_matrix(self, q):
        
        x, y, z, w = q
        return np.array([
            [1-2*y**2-2*z**2, 2*x*y-2*z*w, 2*x*z+2*y*w],
            [2*x*y+2*z*w, 1-2*x**2-2*z**2, 2*y*z-2*x*w],
            [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x**2-2*y**2]
        ])

    def bbox_callback(self, msg):
        
        if not self.enu_pose:
            rospy.logwarn("Waiting for drone position information...")
            return

        
        enu_pos = np.array(self.enu_pose["position"]).reshape(3, 1)
     
        R_enu_body = self.quaternion_to_rotation_matrix(self.enu_pose["orientation"])

       
        enu_bboxes = BoundingBoxes()
        enu_bboxes.header = msg.header
        enu_bboxes.header.frame_id = "enu"  

        for bbox in msg.bounding_boxes:
            
            body_coords = np.array([bbox.x, bbox.y, bbox.z]).reshape(3, 1)


            enu_coords = enu_pos + np.dot(R_enu_body, body_coords)

            
            enu_bbox = BoundingBox()
            enu_bbox.Class = bbox.Class
            enu_bbox.probability = bbox.probability
            enu_bbox.xmin = bbox.xmin
            enu_bbox.ymin = bbox.ymin
            enu_bbox.xmax = bbox.xmax
            enu_bbox.ymax = bbox.ymax
            
            enu_bbox.x = enu_coords[0, 0]
            enu_bbox.y = enu_coords[1, 0]
            enu_bbox.z = enu_coords[2, 0]

            enu_bboxes.bounding_boxes.append(enu_bbox)

       
        self.enu_bbox_pub.publish(enu_bboxes)


if __name__ == "__main__":
    try:
        rospy.init_node("bbox_enu_converter_node", anonymous=True)
        converter = BboxENUConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass