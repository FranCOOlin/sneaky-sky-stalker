### ros转发接受消息

主要功能：
这个包实现了一个双向的ROS-MQTT桥接节点，允许在ROS（机器人操作系统）和MQTT（消息队列遥测传输）协议之间进行消息转换和传输。系统支持动态配置消息映射关系，可以灵活地在ROS话题和MQTT主题之间建立双向通信通道。

#### 相关配置

在config 文件夹下的config.yaml中


#### 安装依赖

``` 
pip install -r requirement.txt
```

#### 启动

```
roslaunch ros_bridge_node bridge.launch
```