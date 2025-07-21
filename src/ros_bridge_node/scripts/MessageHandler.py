#!/usr/bin/env python
import importlib
import math
import rospy
import json
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, Vector3

class MessageHandler:
    """消息处理基类"""
    def __init__(self, publisher, log_name):
        self.publisher = publisher
        self.log_name = log_name
    
    def create_publisher(self, topic, msg_type):
        """动态创建ROS发布器"""
        try:
            # 解析消息类型 (e.g., "std_msgs/String")
            parts = msg_type.split('/')
            if len(parts) != 2:
                raise ValueError(f"无效的消息类型格式: {msg_type}")
            
            pkg = parts[0]
            msg_class = parts[1]
            
            # 动态导入消息模块
            module = importlib.import_module(f"{pkg}.msg")
            msg_class_obj = getattr(module, msg_class)
            
            # 创建发布器
            return rospy.Publisher(topic, msg_class_obj, queue_size=10)
        
        except ImportError:
            rospy.logerr(f"无法导入消息模块: {pkg}.msg")
        except AttributeError:
            rospy.logerr(f"消息类型 {msg_class} 在模块 {pkg}.msg 中不存在")
        except Exception as e:
            rospy.logerr(f"创建发布器失败: {str(e)}")
        return None
    def handle(self, payload):
        """处理消息并发布到ROS"""
        raise NotImplementedError("子类必须实现此方法")

class ControlCommandHandler(MessageHandler):
    """控制命令处理器"""
    def handle(self, payload):
        try:
            ros_msg = String()
            payload = json.loads(payload)
            ros_msg.data = payload['data']
            self.publisher.publish(ros_msg)
            rospy.logdebug(f"转发{self.log_name}: {payload}")
            return True
        except Exception as e:
            rospy.logerr(f"处理{self.log_name}错误: {str(e)}")
            ros_msg.data = payload
            # print(ros_msg)
            self.publisher.publish(ros_msg)
            return False

class StrikeCommandHandler(MessageHandler):
    """打击命令处理器"""
    def handle(self, payload):
        try:
            # 验证JSON格式
            json.loads(payload)
            
            ros_msg = String()
            ros_msg.data = payload
            self.publisher.publish(ros_msg)
            print(ros_msg)
            rospy.logdebug(f"转发{self.log_name}: {payload}")
            return True
        except json.JSONDecodeError:
            rospy.logwarn(f"无效的JSON格式: {payload}")
            return False
        except Exception as e:
            rospy.logerr(f"处理{self.log_name}错误: {str(e)}")
            return False

class LocationHandler(MessageHandler):
    """位置信息处理器"""
    def handle(self, payload):
        try:
            gps_data = json.loads(payload)
            # print(gps_data)
            topicid = gps_data["targetId"]

            self.publisher1 = self.create_publisher(f'command/target{topicid}_location', 'sensor_msgs/NavSatFix')
            self.publisher2 = self.create_publisher(f'command/target{topicid}_velocity', 'geometry_msgs/TwistStamped')
            
            msg = NavSatFix()
            # msg.header.frame_id = gps_data["header"]["frame_id"]
            # msg.header.stamp = rospy.Time(
            #     secs=gps_data["header"]["stamp"]["secs"],
            #     nsecs=gps_data["header"]["stamp"]["nsecs"]
            # )

            # # 填充状态信息
            # msg.status.status = gps_data["status"]["status"]
            # msg.status.service = gps_data["status"]["service"]

            # 填充位置信息
            msg.latitude = gps_data["latitude"]
            msg.longitude = gps_data["longitude"]
            msg.altitude = gps_data["altitude"]

            # 填充协方差信息
            # msg.position_covariance = gps_data["position_covariance"]
            # msg.position_covariance_type = gps_data["position_covariance_type"]
            # print(msg)
            self.publisher1.publish(msg)
            
            
            msg2 = TwistStamped()
            speed = gps_data['speed']
            course = gps_data['course']
            course_rad = math.radians(course)
            vx = speed * math.sin(course_rad)  # 东向分量
            vy = speed * math.cos(course_rad)  # 北向分量
            # 设置线速度
            # print(msg2)
            msg2.twist.linear = Vector3(vx, vy, 0.0)          
              
            self.publisher2.publish(msg2)
            rospy.logdebug(f"转发{self.log_name}: {gps_data}")
            return True
        except (json.JSONDecodeError, ValueError, TypeError) as e:
            rospy.logerr(f"位置数据解析错误: {str(e)}")
            return False
        except Exception as e:
            rospy.logerr(f"处理{self.log_name}错误: {str(e)}")
            return False