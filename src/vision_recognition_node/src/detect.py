#!/usr/bin/env python3
import platform
import pathlib
plt =platform.system()
if plt != 'Windows':
    pathlib.WindowsPath =pathlib.PosixPath
import rospy,math
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
from rostopic import get_topic_type

from sensor_msgs.msg import Image, CompressedImage
from detection_msgs.msg import BoundingBox, BoundingBoxes
from gimbal_bridge_node.msg import GimbalCmd,GimbalState


from nav_msgs.msg import Odometry


# add yolov5 submodule to path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "yolov5"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

# import from yolov5 submodules
from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    check_requirements,
    non_max_suppression,
    scale_coords
)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox
from geometry_msgs.msg import PoseStamped


@torch.no_grad()
class Yolov5Detector:
    def __init__(self):
        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.agnostic_nms = rospy.get_param("~agnostic_nms")
        self.max_det = rospy.get_param("~maximum_detections")
        self.classes = rospy.get_param("~classes", None)
        self.line_thickness = rospy.get_param("~line_thickness")
        self.view_image = rospy.get_param("~view_image")
        self.mavros_pose_sub = rospy.Subscriber("/mavros/local_position/local",  # mavros本地位置话题
                                                Odometry,                     # 消息类型
                                                self.mavros_pose_callback,       # 回调函数
                                                queue_size=1)
       
        self.camera_pose_sub = rospy.Subscriber("/gimbal/state",
                                                 GimbalState,self.camera_pose_callback,
                                                 queue_size=1)
        
        k=1
        self.current_z = 0.0
        # Initialize weights 
        weights = rospy.get_param("~weights")
        # Initialize model
        self.device = select_device(str(rospy.get_param("~device","")))
        self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn"), data=rospy.get_param("~data"))
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )

        # Setting inference size
        self.img_size = [rospy.get_param("~inference_size_w",1920), rospy.get_param("~inference_size_h",1080)]
        self.img_size = check_img_size(self.img_size, s=self.stride)

        # Half
        self.half = rospy.get_param("~half", False)
        self.half &= (
            self.pt or self.jit or self.onnx or self.engine
        ) and self.device.type != "cpu"  # FP16 supported on limited backends with CUDA
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        bs = 1  # batch_size
        cudnn.benchmark = True  # set True to speed up constant image size inference
        self.model.warmup()  # warmup        
        
        # Initialize subscriber to Image/CompressedImage topic
        input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking = True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic, CompressedImage, self.callback, queue_size=1
            )
        else:
            self.image_sub = rospy.Subscriber(
                input_image_topic, Image, self.callback, queue_size=1
            )

        # Initialize prediction publisher
        self.pred_pub = rospy.Publisher(
            rospy.get_param("~output_topic"), BoundingBoxes, queue_size=10
        )
        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic"), Image, queue_size=5
            )

        self.gimbal_cmd_pub = rospy.Publisher(
            "/gimbal/cmd",  # 匹配手动发布的话题
            GimbalCmd,
            queue_size=1
        )
    
        # 新增：船的类别名称（根据你的模型类别修改，如"boat"或"ship"）
        self.ship_class_name = rospy.get_param("~ship_class_name", "boat")  # 关键：指定船的类别名
        
        # 状态机相关：记录上一帧是否检测到船（初始状态为未检测到）
        self.prev_ship_detected = False  # 上一帧状态
        # 定义状态（简化为两种：未检测到船、已检测到船）
        self.states = ["NO_SHIP", "DETECTED_SHIP"]
        self.current_state = self.states[0]  # 初始状态


        # Initialize CV_Bridge
        self.bridge = CvBridge()
        self.K = np.array(rospy.get_param("~camera_intrinsic_matrix",[1931.5616206*k, 0.0, 956.5344838, 0.0, 1931.7249071*k, 531.7483834, 0.0, 0.0, 1.0])).reshape((3, 3))
        #self.K = np.array(rospy.get_param("~camera_intrinsic_matrix",[369.502083, 0.0, 640.0, 0.0, 369.502083, 360.0, 0.0, 0.0, 1.0])).reshape((3, 3))
        #self.R = np.array(rospy.get_param("~camera_rotation_matrix",[0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0])).reshape((3, 3))
        self.t = np.array(rospy.get_param("~camera_translation_vector",[-0.0, 0.0, 0.0])).reshape((3, 1))

        # self.odom_sub = rospy.Subscriber(
        #     "/standard_vtol_0/mavros/local_position/odom", Odometry, self.odom_callback)

    def mavros_pose_callback(self, msg):
        # 提取机体在本地坐标系的位置（x, y, z）
        pos_x = msg.pose.position.x
        pos_y = msg.pose.position.y
        pos_z = msg.pose.position.z
        self.current_z = pos_z
    
    def camera_pose_callback(self,msg):
        camera_yaw = msg.yaw
        camera_pitch = msg.pitch
        camera_roll = msg.roll
        A = math.sin(camera_yaw)
        B = math.cos(camera_yaw)
        C = math.sin(camera_pitch)
        D = math.cos(camera_pitch)
        E = math.sin(camera_roll)
        F = math.cos(camera_roll)

        self.R = np.array([
            [B*D,B*C*E-A*F,B*C*F+A*E],
            [A*D,A*C*E+B*F,A*C*F-B*E],
            [-C,D*E,D*F]
        ])





    # def odom_callback(self, msg):
    #     """处理 odom 消息"""
    #     # 从 odom 消息中提取 z 值
    #     self.current_x = msg.pose.pose.position.x
    #     self.current_y = msg.pose.pose.position.y
    #     self.current_z = msg.pose.pose.position.z


    def camera_to_world(self, x, y, z):
        """
        将相机坐标转换为机体坐标
        """
        # 将像素坐标 (x, y) 转换为归一化设备坐标 (X, Y, Z)
        X = (x - self.K[0, 2]) / self.K[0, 0]
        Y = (y - self.K[1, 2]) / self.K[1, 1]

        # 将 (X, Y, Z) 转换为相机坐标系下的三维坐标
        camera_coords = np.array([[X * z], [Y * z], [z]])

        # 使用外参矩阵将相机坐标转换为机体坐标
        world_coords = np.dot(self.R.T, camera_coords - self.t)

        return world_coords.flatten()


    def update_state(self, current_ship_detected):
        """状态机核心逻辑：根据当前检测结果更新状态并发送指令"""
        # 1. 确定当前状态
        new_state = "DETECTED_SHIP" if current_ship_detected else "NO_SHIP"

        # 2. 状态未变化时，不发送指令（避免冗余）
        if new_state == self.current_state:
            return

        # 3. 状态变化时，更新状态并发送对应指令
        self.current_state = new_state
        gimbal_msg = GimbalCmd()

        if new_state == "DETECTED_SHIP":
            # 检测到船时，设置gimbal_state_machine为3
            gimbal_msg.zoom_in_state = 1
            k=2  
            rospy.loginfo("状态切换：检测到船，发送gimbal_state_machine: 3")
        else:
            # 未检测到船时，设置gimbal_state_machine为1（或其他需求值）
            gimbal_msg.zoom_in_state = 0
            k=1  
            rospy.loginfo("状态切换：未检测到船，发送gimbal_state_machine: 0")

        # 发布指令
        self.gimbal_cmd_pub.publish(gimbal_msg)
        
        # 更新上一帧状态
        self.prev_ship_detected = current_ship_detected
        self.K = np.array([1931.5616206*k, 0.0, 956.5344838, 0.0, 1931.7249071*k, 531.7483834, 0.0, 0.0, 1.0]).reshape((3, 3))


    def callback(self, data):
        """adapted from yolov5/detect.py"""
        # print(data.header)
        if self.compressed_input:
            im = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        else:
            im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        im, im0 = self.preprocess(im)
        # print(im.shape)
        # print(img0.shape)
        # print(img.shape)

        # Run inference
        im = torch.from_numpy(im).to(self.device) 
        im = im.half() if self.half else im.float()
        im /= 255
        if len(im.shape) == 3:
            im = im[None]

        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression(
            pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det
        )

        ### To-do move pred to CPU and fill BoundingBox messages
        
        # Process predictions 
        det = pred[0].cpu().numpy()

        bounding_boxes = BoundingBoxes()
        bounding_boxes.header = data.header
        bounding_boxes.image_header = data.header
        
        ship_detected = False

        annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

            # Write results
            for *xyxy, conf, cls in reversed(det):
                bounding_box = BoundingBox()
                c = int(cls)

                # Fill in bounding box message
                bounding_box.Class = self.names[c]
                bounding_box.probability = conf 
                bounding_box.xmin = int(xyxy[0])
                bounding_box.ymin = int(xyxy[1])
                bounding_box.xmax = int(xyxy[2])
                bounding_box.ymax = int(xyxy[3])
             #   bounding_box.zx = int(xyxy[0])+int(xyxy[2])-1260
             #   bounding_box.zy = int(xyxy[1])+int(xyxy[3])-700

                cx = (xyxy[0] + xyxy[2]) / 2
                cy = (xyxy[1] + xyxy[3]) / 2

                z = self.current_z + 0.8

                world_coords = self.camera_to_world(cx, cy, z)
                bounding_box.x = -world_coords[1] -0.05
                bounding_box.y = -world_coords[0] + 0.3
                bounding_box.z = world_coords[2]


                bounding_boxes.bounding_boxes.append(bounding_box)
                
                if bounding_box.Class == self.ship_class_name:  # 匹配船的类别名
                   ship_detected = True
                # Annotate the image
                if self.publish_image or self.view_image:  # Add bbox to image
                      # integer class
                    label = f"{self.names[c]} {conf:.2f}"
                    annotator.box_label(xyxy, label, color=colors(c, True))       


                
                ### POPULATE THE DETECTION MESSAGE HERE

            # Stream results
            im0 = annotator.result()

        # Publish prediction
        self.pred_pub.publish(bounding_boxes)
        self.update_state(ship_detected)


        # Publish & visualize images
        if self.view_image:
            cv2.imshow(str(0), im0)
            cv2.waitKey(1)  # 1 millisecond
        if self.publish_image:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im0, "bgr8"))
        

    def preprocess(self, img):
        """
        Adapted from yolov5/utils/datasets.py LoadStreams class
        """
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        return img, img0 


if __name__ == "__main__":

    check_requirements(exclude=("tensorboard", "thop"))
    
    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector()
    
    rospy.spin()