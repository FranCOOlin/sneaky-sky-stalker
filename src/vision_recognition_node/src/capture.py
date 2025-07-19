#!/usr/bin/env python3
# -*- coding: utf-8 -*-  

import rospy
import cv2
from cv_bridge import CvBridge
from detection_msgs.msg import BoundingBoxes  
from sensor_msgs.msg import Image 


class ShapeCaptureNode:
    def __init__(self):
        self.cylinder_captured = False  # Flag to track if cylinder has been captured
        self.cone_captured = False      # Flag to track if cone has been captured
        self.bridge = CvBridge()
        
        # ROS parameters for object class names
        self.cylinder_class = rospy.get_param("~cylinder_class", "cylinder")
        self.cone_class = rospy.get_param("~cone_class", "cone")
        
        self.current_bboxes = []
        
        # Subscribe to bounding box detections
        self.bbox_sub = rospy.Subscriber(
            rospy.get_param("~bbox_topic", "/yolov5/detections"),  
            BoundingBoxes,
            self.bbox_callback,
            queue_size=1
        )

        # Subscribe to raw image topic
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", 
            Image, 
            self.image_callback
        )
        
        # Publish cylinder images with bounding boxes
        self.cylinder_image_pub = rospy.Publisher(
            "/capture/cylinder_image_with_boxes", 
            Image, 
            queue_size=1
        )
        
        # Publish cone images with bounding boxes
        self.cone_image_pub = rospy.Publisher(
            "/capture/cone_image_with_boxes", 
            Image, 
            queue_size=1
        )
               
        self.cv_image = None  
        rospy.loginfo("Shape capture node initialized. Waiting for shapes...")

    def image_callback(self, msg):
        """Convert ROS Image message to OpenCV format"""
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", e)

    def bbox_callback(self, msg):
        """Process bounding boxes and publish images for first-time detections"""
        self.current_bboxes = msg.bounding_boxes
        
        # Flags to ensure each shape is published only once per callback
        cylinder_published = False
        cone_published = False
        
        for bbox in msg.bounding_boxes:
            class_name = bbox.Class.lower()
            
            # Publish cylinder image if detected for the first time
            if (self.cylinder_class in class_name 
                and not self.cylinder_captured 
                and self.cv_image is not None 
                and not cylinder_published):
                self.save_and_publish_image("cylinder", self.cylinder_image_pub)
                self.cylinder_captured = True
                cylinder_published = True  # Mark cylinder as published
                
                
            
            # Publish cone image if detected for the first time
            elif (self.cone_class in class_name 
                  and not self.cone_captured 
                  and self.cv_image is not None 
                  and not cone_published):
                self.save_and_publish_image("cone", self.cone_image_pub)
                self.cone_captured = True
                cone_published = True  # Mark cone as published
                

    def save_and_publish_image(self, shape_type, publisher):
        """Draw bounding boxes on image, save to disk, and publish to ROS topic"""
        img_with_bbox = self.cv_image.copy()
        
        # Draw bounding boxes and labels on the image
        for bbox in self.current_bboxes:
            xmin = bbox.xmin
            ymin = bbox.ymin
            xmax = bbox.xmax
            ymax = bbox.ymax
            class_name = bbox.Class
            
            # Draw bounding box rectangle
            cv2.rectangle(img_with_bbox, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            
            # Add class name text
            cv2.putText(
                img_with_bbox, 
                class_name, 
                (xmin, ymin - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.5, 
                (255, 255, 255), 
                2
            )
        
        # Save image to disk and publish to ROS topic
        save_path = f"/home/nvidia/sneaky-sky-stalker/tmp/{shape_type}_image.jpg"
        try:
            cv2.imwrite(save_path, img_with_bbox)
            rospy.loginfo(f"{shape_type} saved: {save_path}")
            
            # Convert to ROS Image message and publish
            img_msg = self.bridge.cv2_to_imgmsg(img_with_bbox, "bgr8")
            publisher.publish(img_msg)
            rospy.loginfo(f"Published {shape_type} image to topic: {publisher.name}")
        except Exception as e:
            rospy.logerr(f"{shape_type} failed: {e}")
            
            
    def publish_target_detection(self, target_id, detected):
        """发布目标检测状态"""
        msg = Bool()
        msg.data = detected
        
        if target_id == 1:
            self.cylinder_pub.publish(msg)
            rospy.loginfo(f"cylinder has been found")
        elif target_id == 2:
            self.cone_pub.publish(msg)
            rospy.loginfo(f"cone has been found")
            

if __name__ == "__main__":
    try:
        rospy.init_node("shape_capture_node", anonymous=True)
        node = ShapeCaptureNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass