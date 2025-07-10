#!/usr/bin/env python3
import json
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import random
# 现在可以直接导入
# from MessageHandler import ControlCommandHandler, StrikeCommandHandler, LocationHandler
from MessageHandler import ControlCommandHandler, StrikeCommandHandler, LocationHandler
from paho.mqtt import client as mqtt_client
import rospy
import importlib
from rospy_message_converter import json_message_converter 

class MqttRosBridge:
    def __init__(self):
        rospy.init_node('ros_bridge_node', anonymous=True)
        
        # 存储所有处理器
        self.handlers = {}
        print("12344444")
        # 从参数服务器获取MQTT配置
        self.mqtt_broker = rospy.get_param('~mqtt_broker', '')
        
        self.mqtt_port = rospy.get_param('~mqtt_port', '')
        self.mqtt_user = rospy.get_param('~mqtt_username', '')
        self.mqtt_pass = rospy.get_param('~mqtt_password', '')
        print(self.mqtt_broker,self.mqtt_port,self.mqtt_user)
        # 加载网络到ROS的映射配置
        self.load_sub_mappings()
        client_id = f'python-mqtt-{random.randint(0, 1000)}'
        # MQTT客户端设置
        self.mqtt_client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1,client_id)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.tls_set(ca_certs='/home/ipac-ros-server/rostest/src/ros_bridge_node/scripts/emqxsl-ca.crt')
        if self.mqtt_user:
            self.mqtt_client.username_pw_set(self.mqtt_user, self.mqtt_pass)
        self.setup_ros_to_network()
        # 连接MQTT代理
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
        rospy.loginfo(f"连接到MQTT代理 {self.mqtt_broker}:{self.mqtt_port}")
        
        # 在ROS节点关闭时断开MQTT连接
        rospy.on_shutdown(self.shutdown_handler)
        
        # 启动MQTT循环
        self.mqtt_client.loop_start()
        
        
            
    
    def setup_ros_to_network(self):
        mappings = rospy.get_param('~ros_to_network', {})
        if not mappings:
            rospy.logwarn("未找到网络到ROS的映射配置，使用默认配置")
            mappings = {
                "/uav/gps": {
                    "url": "uav/gps",
                    "msg_type": "sensor_msgs/NavSatFix"
                },
                "/uav/velocity_heading": {
                    "url": "uav/velocity_heading",
                    "msg_type": "geometry_msgs/TwistStamped"
                },
                "/uav/status": {
                    "url": "uav/status",
                    "msg_type": "std_msgs/Int32"
                },
                "/uav/target_detection": {
                    "url": "uav/target_detection",
                    "msg_type": "geometry_msgs/Point"
                },
                "/target/gps": {
                    "url": "target/gps",
                    "msg_type": "sensor_msgs/NavSatFix"
                },
                "/target/velocity_heading": {
                    "url": "target/velocity_heading",
                    "msg_type": "geometry_msgs/TwistStamped"
                },
                "/target/type": {
                    "url": "target/type",
                    "msg_type": "std_msgs/Int32"
                },
                "/uav/mission_feedback": {
                    "url": "uav/mission_feedback",
                    "msg_type": "std_msgs/String"
                }
            }
        self.submappings = mappings
        for topic, config in mappings.items():
            msg_type = config.get('msg_type')

            # self.mqtt_client.subscribe(config["url"], qos=1)
            # rospy.loginfo(f"订阅MQTT主题: {topic}")
            parts = msg_type.split('/')
            pkg = parts[0]
            msg_class = parts[1]
            module = importlib.import_module(f"{pkg}.msg")
            msg_class_obj = getattr(module, msg_class)
            rospy.Subscriber(topic, msg_class_obj, self.ros_callback, callback_args=config["url"])
    
    def ros_callback(self, msg, url):
        try:
            json_data = json_message_converter.convert_ros_message_to_json(msg)
            result = self.mqtt_client.publish(url,json_data)
            status = result[0]
            # if status == 0:
                # print(f"Send `{msg}` to topic `{url}`")
            # else:
                # print(f"Failed to send message to topic {url}")
        except Exception as e:
            rospy.logerr(f"Failed to POST to {url}: {e}")
        
    

    def load_sub_mappings(self):
        """从参数服务器加载网络到ROS的映射配置"""
        try:
            mappings = rospy.get_param('~network_to_ros', {})
            
            if not mappings:
                rospy.logwarn("未找到网络到ROS的映射配置，使用默认配置")
                mappings = {
                    "/command/target_location": {
                        "url": "command/target_location",
                        "msg_type": "sensor_msgs/NavSatFix"
                    },
                    "/command/mission": {
                        "url": "command/mission",
                        "msg_type": "std_msgs/String"
                    },
                }
            
            # 处理每个映射
            for ros_topic, config in mappings.items():
                mqtt_topic = config.get('url')  # 从配置中获取MQTT主题
                msg_type = config.get('msg_type')
                
                if not all([mqtt_topic, ros_topic, msg_type]):
                    rospy.logerr(f"无效映射配置: {ros_topic} - {config}")
                    continue
                
                # 创建ROS发布器
                publisher = self.create_publisher(ros_topic, msg_type)
                if not publisher:
                    continue
                
                # 创建对应的消息处理器
                handler = self.create_handler(publisher, msg_type, f"{mqtt_topic}->{ros_topic}")
                if not handler:
                    continue
                
                # 注册处理器
                self.handlers[mqtt_topic] = handler
                rospy.loginfo(f"注册映射: MQTT主题 '{mqtt_topic}' -> ROS话题 '{ros_topic}' ({msg_type})")
        
        except Exception as e:
            rospy.logerr(f"加载映射配置失败: {str(e)}")

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

    def create_handler(self, publisher, msg_type, log_name):
        """根据消息类型创建处理器"""
        # 简化处理：使用特定处理器处理常见类型
        if msg_type == "std_msgs/String":
            return ControlCommandHandler(publisher, log_name)
        elif msg_type == "sensor_msgs/NavSatFix":
            return LocationHandler(publisher, log_name)
        
        # 对于其他类型，尝试使用通用JSON处理器
        try:
            rospy.logwarn(f"使用通用JSON处理器处理 {msg_type}")
            return StrikeCommandHandler(publisher, log_name)
        except Exception as e:
            rospy.logerr(f"创建处理器失败: {str(e)}")
            return None

    def on_mqtt_connect(self, client, userdata, flags, rc):
        rospy.loginfo(f"MQTT连接状态: {rc}")
        print(self.handlers)
        if rc == 0:
            # 订阅所有处理器对应的主题
            # for topic in self.submappings.keys():
            #     client.subscribe(topic, qos=1)
            #     rospy.loginfo(f"订阅MQTT主题: {topic}")
            for topic in self.handlers.keys():
                client.subscribe(topic, qos=1)
                rospy.loginfo(f"订阅MQTT主题: {topic}")
            rospy.loginfo(f"已订阅 {len(self.handlers)} 个MQTT主题")
        else:
            rospy.logerr(f"MQTT连接失败: 错误码 {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            payload = msg.payload.decode('utf-8')
        except UnicodeDecodeError:
            rospy.logerr(f"无法解码MQTT消息 [{topic}]: 非UTF-8编码")
            return
        
        rospy.logdebug(f"收到MQTT消息 [{topic}]: {payload[:100]}{'...' if len(payload) > 100 else ''}")
        
        # 查找对应的处理器
        handler = self.handlers.get(topic)
        if handler:
            handler.handle(payload)
        else:
            rospy.logwarn(f"未注册的MQTT主题: {topic}")

    def register_mapping(self, url, ros_topic, msg_type):
        """动态注册新的映射关系"""
        # 创建ROS发布器
        publisher = self.create_publisher(ros_topic, msg_type)
        if not publisher:
            return False
        
        # 创建消息处理器
        handler = self.create_handler(publisher, msg_type, f"{url}->{ros_topic}")
        if not handler:
            return False
        
        # 注册处理器并订阅主题
        self.handlers[url] = handler
        self.mqtt_client.subscribe(url, qos=1)
        rospy.loginfo(f"动态注册映射: {url} -> {ros_topic} ({msg_type})")
        return True

    def shutdown_handler(self):
        rospy.loginfo("节点关闭，断开MQTT连接...")
        self.mqtt_client.disconnect()
        self.mqtt_client.loop_stop()

    def run(self):
        rospy.spin()
if __name__ == '__main__':
    try:
        bridge = MqttRosBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass