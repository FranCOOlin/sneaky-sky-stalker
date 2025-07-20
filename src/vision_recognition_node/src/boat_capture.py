#!/usr/bin/env python3
import rospy
from detection_msgs.msg import BoundingBoxes 
from std_msgs.msg import Bool, Int8  

class ShipDetectionController:
    def __init__(self):
        
        rospy.init_node('ship_detection_controller', anonymous=True)
        
        
        self.detection_topic = rospy.get_param("~detection_topic", "/yolov5/detections")
        self.ship_class = rospy.get_param("~ship_class", "ship")
        self.cone_class = rospy.get_param("~cone_class", "cone")
        self.cylinder_class = rospy.get_param("~cylinder_class", "cylinder")
        
       
        self.first_ship_detected = False
        self.second_ship_detected = False
        self.current_ship_count = 0
        
        
        self.target1_detected_pub = rospy.Publisher("/target1/detected", Bool, queue_size=1)
        self.target1_type_pub = rospy.Publisher("/target1/type", Int8, queue_size=1)
        self.target2_detected_pub = rospy.Publisher("/target2/detected", Bool, queue_size=1)
        self.target2_type_pub = rospy.Publisher("/target2/type", Int8, queue_size=1)
        
        
        self.detection_sub = rospy.Subscriber(
            self.detection_topic, 
            BoundingBoxes, 
            self.detection_callback, 
            queue_size=1
        )
        
        rospy.loginfo("Ship detection controller initialized. Waiting for detections...")
    
    def detection_callback(self, msg):
        
        
        ship_detected = False
        cone_detected = False
        cylinder_detected = False
        current_ships = []
        
        
        for box in msg.bounding_boxes:
            if box.Class == self.ship_class:
                ship_detected = True
                current_ships.append(box)
            elif box.Class == self.cone_class:
                cone_detected = True
            elif box.Class == self.cylinder_class:
                cylinder_detected = True
        
       
        if ship_detected:
            self.current_ship_count = len(current_ships)
            
            
            if not self.first_ship_detected:
                self.first_ship_detected = True
                rospy.loginfo("First ship detected!")
                
                
                self.target1_detected_pub.publish(Bool(True))
                
                
                target_type = 0 if cone_detected else 1 if cylinder_detected else -1
                self.target1_type_pub.publish(Int8(target_type))
                rospy.loginfo(f"First ship type: {'cone' if target_type == 0 else 'cylinder' if target_type == 1 else 'unknown'}")
            
            
            elif self.first_ship_detected and not self.second_ship_detected and self.current_ship_count >= 2:
                self.second_ship_detected = True
                rospy.loginfo("Second ship detected!")
                
                
                self.target2_detected_pub.publish(Bool(True))
                
                
                target_type = 0 if cone_detected else 1 if cylinder_detected else -1
                self.target2_type_pub.publish(Int8(target_type))
                rospy.loginfo(f"Second ship type: {'cone' if target_type == 0 else 'cylinder' if target_type == 1 else 'unknown'}")

if __name__ == "__main__":
    try:
        controller = ShipDetectionController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass