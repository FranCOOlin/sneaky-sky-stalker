#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <deque>
#include <algorithm>
#include <queue>
#include <vector>
#include <gimbal_bridge_node/siyi_zr10_protocol.h>
#include <gimbal_bridge_node/GimbalState.h>
#include <gimbal_bridge_node/GimbalCmd.h>
#include <gimbal_bridge_node/GimbalDebug.h>
#include <sensor_msgs/Imu.h>
#include <detection_msgs/BoundingBoxes.h>
#include "control_lib/math_lib.hpp"

constexpr int RECV_BUF_SIZE = 64;
constexpr int SERVER_PORT = 37260;
constexpr char SERVER_IP[] = "192.168.144.25";
constexpr size_t MAX_QUEUE_SIZE = 5;

// 创建 Publisher，Topic 名字和消息类型
ros::Publisher pub;

// 当前俯仰角
float drone_pitch_deg = 0.0f;

// 过去一秒钟收到的三种消息的数量
int target_msg_count = 0;
int state_msg_count = 0;
int gimbal_cmd_count = 0;
double last_time_sec = 0.0;

gimbal_bridge_node::GimbalDebug debug_msg;

// 定义目标结构体
struct TargetInfo
{
    std::string class_name;
    double probability;
    double center_x;
    double center_y;
    double center_z;
    int64_t xmin;
    int64_t ymin;
    int64_t xmax;
    int64_t ymax;
    friend bool operator>(const TargetInfo &a, const TargetInfo &b)
    {
        return a.probability > b.probability; // 按概率排序
    }
};

// 定义识别结果结构体
struct TargetsInfo
{
    // 小根堆，当堆中的元素超过2个时，自动删除概率最低的元素，最终维持堆中的元素个数不超过2个，留下的就是概率最高的两个目标
    std::priority_queue<TargetInfo, std::vector<TargetInfo>, std::greater<TargetInfo>> targets;
    uint32_t nsec;
};

// 定义俯仰角数据结构体
struct pitchInfo
{
    double pitch;
    uint32_t nsec;
};

std::deque<gimbal_bridge_node::GimbalCmd> gimbal_cmd_history_queue;
std::deque<gimbal_bridge_node::GimbalState> gimbal_state_history_queue;

class gimbal_camera_state_bridge
{
public:
    // 构造函数，可指定IP和端口，默认使用题目给定的参数
    gimbal_camera_state_bridge(const std::string &server_ip = "192.168.144.25", uint16_t server_port = 37260)
        : server_ip_(server_ip), server_port_(server_port)
    {
        memset(&send_addr_, 0, sizeof(send_addr_));
        send_addr_.sin_family = AF_INET;
        send_addr_.sin_port = htons(server_port_);

        // 转换IP地址
        if (inet_pton(AF_INET, server_ip_.c_str(), &send_addr_.sin_addr) <= 0)
        {
            throw std::invalid_argument("Invalid IP address: " + server_ip_);
        }
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0)
        {
            ROS_ERROR("Failed to create socket: %s", strerror(errno));
        }
        ROS_INFO("Socket created successfully, ready to send data to %s:%d", server_ip_.c_str(), server_port_);
    }

    ~gimbal_camera_state_bridge()
    {
        close(sockfd);
    }

    int ping()
    {
        // 定义请求数据
        uint8_t send_buf[RECV_BUF_SIZE] = {0};
        uint16_t request_length = siyi_pack_get_firmware_version(send_buf, RECV_BUF_SIZE, 0x0000);
        if (request_length == 0)
        {
            ROS_ERROR("Failed to pack request data");
            return 0;
        }

        // 发送数据
        if (sendto(sockfd, send_buf, request_length, 0,
                   (struct sockaddr *)&send_addr_, sizeof(send_addr_)) < 0)
        {
            ROS_ERROR("sendto failed: %s", strerror(errno));
            return 0;
        }
        // 接收响应
        struct sockaddr_in recv_addr;
        socklen_t addr_len = sizeof(recv_addr);
        unsigned char recv_buf[RECV_BUF_SIZE] = {0};
        int recv_len = recvfrom(sockfd, recv_buf, RECV_BUF_SIZE, 0,
                                (struct sockaddr *)&recv_addr, &addr_len);
        if (recv_len < 0)
        {
            ROS_ERROR("recvfrom failed: %s", strerror(errno));
            return 0;
        }
        FirmwareVersionResponse response;

        // 解析响应
        if (!siyi_unpack_firmware_version_response(recv_buf, recv_len, &response))
        {
            ROS_ERROR("Failed to unpack response data");
            return 0;
        }
        // 打印接收到的数据
        ROS_INFO("Connected to gimbal, Get firmware version response:");
        ROS_INFO("Code Board Version: %u", response.code_board_ver);
        ROS_INFO("Gimbal Firmware Version: %u", response.gimbal_firmware_ver);
        ROS_INFO("Zoom Firmware Version: %u", response.zoom_firmware_ver);

        return 1;
    }

    void set_func_mode(FunctionType mode)
    {
        uint8_t send_buf[RECV_BUF_SIZE] = {0};
        uint16_t request_length = siyi_pack_function_control(send_buf, RECV_BUF_SIZE, mode, 0x0000);
        if (request_length == 0)
        {
            ROS_ERROR("Failed to pack request data");
            return;
        }

        // 发送数据
        if (sendto(sockfd, send_buf, request_length, 0,
                   (struct sockaddr *)&send_addr_, sizeof(send_addr_)) < 0)
        {
            ROS_ERROR("sendto failed: %s", strerror(errno));
            return;
        }
    }

    void request_gimbal_state()
    {
        // 定义请求数据
        uint8_t send_buf[RECV_BUF_SIZE] = {0};
        uint16_t request_length = siyi_pack_request_data_stream(send_buf, RECV_BUF_SIZE, 0x01, DATA_FREQ_50HZ, 0x0000);
        if (request_length == 0)
        {
            ROS_ERROR("Failed to pack request data");
            return;
        }
        ROS_INFO("Packed request data length: %d", request_length);

        // 发送数据
        if (sendto(sockfd, send_buf, request_length, 0,
                   (struct sockaddr *)&send_addr_, sizeof(send_addr_)) < 0)
        {
            ROS_ERROR("sendto failed: %s", strerror(errno));
            return;
        }
    }

    bool receive_and_publish()
    {
        // 使用select函数检查套接字是否可读
        fd_set read_fds;
        struct timeval timeout;
        FD_ZERO(&read_fds);
        FD_SET(sockfd, &read_fds);
        timeout.tv_sec = 0; // 设置超时时间为0，实现非阻塞
        timeout.tv_usec = 0;

        int activity = select(sockfd + 1, &read_fds, nullptr, nullptr, &timeout);
        if (activity < 0)
        {
            ROS_ERROR("select failed: %s", strerror(errno));
            return false;
        }
        else if (activity == 0)
        {
            // 超时，没有数据可读
            return false;
        }

        // 接收响应
        struct sockaddr_in recv_addr;
        socklen_t addr_len = sizeof(recv_addr);
        unsigned char bag[RECV_BUF_SIZE] = {0};
        int bag_len = recvfrom(sockfd, bag, RECV_BUF_SIZE, 0,
                               (struct sockaddr *)&recv_addr, &addr_len);
        if (bag_len < 0)
        {
            ROS_ERROR("recvfrom failed: %s", strerror(errno));
            return false;
        }

        AttitudeDataResponse response;
        // 解析响应
        if (!siyi_unpack_attitude_data_response(bag, bag_len, &response))
        {
            ROS_ERROR("Failed to unpack response data");
            ROS_INFO("Received data length: %d", bag_len);
            ROS_INFO("Received data: ");
            for (int i = 0; i < bag_len; ++i)
            {
                ROS_INFO("%02x ", bag[i]);
            }
            return false;
        }
        this->gimbal_yaw_feed = static_cast<float>(response.yaw) / 10.0f;
        this->gimbal_pitch_feed = static_cast<float>(response.pitch) / 10.0f;
        gimbal_bridge_node::GimbalState msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "gimbal_state";
        msg.yaw = this->gimbal_yaw_feed;
        msg.pitch = this->gimbal_pitch_feed;
        msg.roll = static_cast<float>(response.roll) / 10.0f;
        msg.json_string = "{\"status\":\"normal\"}";
        // 发布消息
        pub.publish(msg);
        debug_msg.gimbal_state.push_back(msg);
        while (debug_msg.gimbal_state.size() > MAX_QUEUE_SIZE)
        {
            debug_msg.gimbal_state.erase(debug_msg.gimbal_state.begin());
        }
        state_msg_count++;
        return true;
    }

    bool execute_turn_command(const gimbal_bridge_node::GimbalCmd &cmd)
    {
        // 打包云台转向命令
        uint8_t send_buf[RECV_BUF_SIZE] = {0};
        uint16_t request_length = siyi_pack_control_angle(send_buf, RECV_BUF_SIZE,
                                                          static_cast<int16_t>(cmd.yaw * 10),
                                                          static_cast<int16_t>(cmd.pitch * 10), 0x0000);
        if (request_length == 0)
        {
            ROS_ERROR("Failed to pack gimbal rotation command");
            return false;
        }

        // 发送数据
        if (sendto(sockfd, send_buf, request_length, 0,
                   (struct sockaddr *)&send_addr_, sizeof(send_addr_)) < 0)
        {
            ROS_ERROR("sendto failed: %s", strerror(errno));
            return false;
        }
        return true;
    }

    bool execute_zoom_in_command()
    {
        // 打包云台变倍命令
        uint8_t send_buf[RECV_BUF_SIZE] = {0};
        uint16_t request_length = siyi_pack_absolute_zoom(send_buf, RECV_BUF_SIZE, 0x02, 0x00, 0x0000);
        if (request_length == 0)
        {
            ROS_ERROR("Failed to pack zoom in command");
            return false;
        }

        // 发送数据
        if (sendto(sockfd, send_buf, request_length, 0,
                   (struct sockaddr *)&send_addr_, sizeof(send_addr_)) < 0)
        {
            ROS_ERROR("sendto failed: %s", strerror(errno));
            return false;
        }
        return true;
    }

    bool execute_zoom_out_command()
    {
        // 打包云台变倍命令
        uint8_t send_buf[RECV_BUF_SIZE] = {0};
        uint16_t request_length = siyi_pack_absolute_zoom(send_buf, RECV_BUF_SIZE, 0x01, 0x00, 0x0000);
        if (request_length == 0)
        {
            ROS_ERROR("Failed to pack zoom out command");
            return false;
        }

        // 发送数据
        if (sendto(sockfd, send_buf, request_length, 0,
                   (struct sockaddr *)&send_addr_, sizeof(send_addr_)) < 0)
        {
            ROS_ERROR("sendto failed: %s", strerror(errno));
            return false;
        }
        return true;
    }

    int get_state_machine() const
    {
        return state_machine;
    }

    void set_state_machine(int state)
    {
        if (state < 0 || state > 2)
        {
            ROS_ERROR("Invalid gimbal state machine value: %d. It must be 0, 1, or 2.", state);
            return;
        }
        state_machine = state;
    }

    bool is_zoom_in() const
    {
        return zoom_in;
    }

    void set_zoom_in(bool zoom)
    {
        zoom_in = zoom;
    }

    void set_debug_publisher(ros::Publisher &&debug_pub)
    {
        this->debug_pub = std::move(debug_pub);
    }

    ros::Publisher debug_pub;
    float gimbal_pitch_feed = 0.0f; // 云台俯仰角
    float gimbal_yaw_feed = 0.0f;   // 云台偏航角

private:
    static constexpr int RECV_BUF_SIZE = 64;
    std::string server_ip_;
    uint16_t server_port_;
    struct sockaddr_in send_addr_;
    int sockfd;
    int state_machine = 0; // 云台状态机，0:前下方，1:正下方，2:自由控制
    bool zoom_in = false;  // 是否处于变倍状态
};

// 初始化云台桥接类
gimbal_camera_state_bridge gimbal_bridge(SERVER_IP, SERVER_PORT);

void gimbal_cmd_callback(const gimbal_bridge_node::GimbalCmd::ConstPtr &msg)
{
    // 打印接收到的命令信息
    ROS_INFO("Received Gimbal Command: yaw=%.2f, pitch=%.2f, gimbal_state_machine = %d, json_string=%s",
             msg->yaw, msg->pitch, msg->gimbal_state_machine, msg->json_string.c_str());
    if (msg->gimbal_state_machine < 0 || msg->gimbal_state_machine > 3)
    {
        ROS_ERROR("Invalid gimbal state machine value: %d. It must be 0, 1, 2, or 3.", msg->gimbal_state_machine);
        return;
    }
    gimbal_bridge_node::GimbalCmd gimbal_cmd = *msg;
    ROS_INFO("gimbal bridge state machine: %d", gimbal_bridge.get_state_machine());

    debug_msg.gimbal_cmd.push_back(gimbal_cmd);
    while (debug_msg.gimbal_cmd.size() > MAX_QUEUE_SIZE)
    {
        debug_msg.gimbal_cmd.erase(debug_msg.gimbal_cmd.begin());
    }
    gimbal_cmd_count++;

    if ((gimbal_cmd.gimbal_state_machine == 2) != (gimbal_bridge.get_state_machine() == 2))
    {
        gimbal_bridge.set_state_machine(gimbal_cmd.gimbal_state_machine);
        if (gimbal_cmd.gimbal_state_machine != 2)
        {
            gimbal_bridge.set_func_mode(FUNC_MODE_FPV);
            ROS_INFO("Gimbal is set to FPV mode.");
        }
        else
        {
            gimbal_bridge.set_func_mode(FUNC_MODE_FOLLOW);
            ROS_INFO("Gimbal is set to follow mode.");
        }
    }

    if (gimbal_cmd.gimbal_state_machine == 0)
    {
        // 云台状态机为0时，云台指向前下方（pitch = -45， yaw=0）
        ROS_INFO("Gimbal is set to point forward downwards (pitch=-45, yaw=0).");
        gimbal_cmd.pitch = drone_pitch_deg - 45;
        gimbal_cmd.yaw = 0;
    }
    else if (gimbal_cmd.gimbal_state_machine == 1)
    {
        // 云台状态机为1时，云台指向正下方（pitch=-90，yaw=0）
        ROS_INFO("Gimbal is set to point directly downwards (pitch=-90, yaw=0).");
        gimbal_cmd.pitch = -90;
        gimbal_cmd.yaw = 0;
    }
    else
    {
        // 这里仅仅进行有效性判断，平滑性等更高层次的要求交给控制器进行处理。
        if (gimbal_cmd.yaw < -135 || gimbal_cmd.yaw > 135 ||
            gimbal_cmd.pitch < -90 || gimbal_cmd.pitch > 25)
        {
            ROS_ERROR("Invalid gimbal command: yaw or pitch out of range.");
            return;
        }
    }
    if (!gimbal_bridge.execute_turn_command(gimbal_cmd))
    {
        ROS_ERROR("Failed to execute gimbal command.");
    }
    else
    {
        ROS_INFO("Gimbal command executed successfully.");
    }

    if (gimbal_cmd.zoom_in_state == 0 && gimbal_bridge.is_zoom_in())
    {
        if (gimbal_bridge.execute_zoom_out_command())
        {
            ROS_INFO("Zoom out command executed successfully.");
            gimbal_bridge.set_zoom_in(false);
        }
        else
        {
            ROS_ERROR("Failed to execute zoom out command.");
        }
    }
    else if (gimbal_cmd.zoom_in_state == 1 && !gimbal_bridge.is_zoom_in())
    {
        if (gimbal_bridge.execute_zoom_in_command())
        {
            ROS_INFO("Zoom in command executed successfully.");
            gimbal_bridge.set_zoom_in(true);
        }
        else
        {
            ROS_ERROR("Failed to execute zoom in command.");
        }
    }

    // debug
    debug_msg.gimbal_yaw = gimbal_bridge.gimbal_yaw_feed;
    debug_msg.gimbal_pitch = gimbal_bridge.gimbal_pitch_feed;
    debug_msg.gimbal_state_machine = gimbal_bridge.get_state_machine();
    debug_msg.zoom_in_state = gimbal_bridge.is_zoom_in() ? 1 : 0;
    debug_msg.drone_pitch = drone_pitch_deg;
    debug_msg.gimbal_yaw_cmd = gimbal_cmd.yaw;
    debug_msg.gimbal_pitch_cmd = gimbal_cmd.pitch;
}

void drone_attitude_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // 从IMU消息中获取四元数
    double qx = msg->orientation.x;
    double qy = msg->orientation.y;
    double qz = msg->orientation.z;
    double qw = msg->orientation.w;

    // 计算俯仰角 (pitch)，单位为弧度
    // 使用四元数到欧拉角的转换公式
    double pitch = atan2(2.0 * (qw * qy - qz * qx),
                         1.0 - 2.0 * (qy * qy + qx * qx));

    // 可选：转换为角度（方便人类阅读）
    drone_pitch_deg = pitch * 180.0 / M_PI;

    debug_msg.imu.push_back(*msg);
    while (debug_msg.imu.size() > MAX_QUEUE_SIZE)
    {
        debug_msg.imu.erase(debug_msg.imu.begin());
    }
}

void bounding_boxes_callback(const detection_msgs::BoundingBoxes::ConstPtr &msg)
{
    // 显示检测到的目标数量
    ROS_INFO("Received %zu bounding boxes", msg->bounding_boxes.size());

    // 处理当前帧的目标
    std::vector<TargetInfo> current_frame_targets;
    TargetsInfo targets_info;
    targets_info.nsec = msg->header.stamp.toNSec();

    for (const auto &box : msg->bounding_boxes)
    {
        // 计算目标中心坐标
        double center_x = (box.xmin + box.xmax) / 2.0;
        double center_y = (box.ymin + box.ymax) / 2.0;

        TargetInfo target;
        target.class_name = box.Class;
        target.probability = box.probability;
        target.center_x = center_x;
        target.center_y = center_y;
        target.center_z = box.z;
        target.xmin = box.xmin;
        target.ymin = box.ymin;
        target.xmax = box.xmax;
        target.ymax = box.ymax;

        targets_info.targets.push(target);

        while (targets_info.targets.size() > 2)
        {
            targets_info.targets.pop(); // 保持最多两个目标
        }
    }
}

void input_state_check()
{
    // 根据三个queue中最新元素的时间和当前时间的差来进行安全检查
    double current_time_sec = ros::Time::now().toSec();
    if (current_time_sec - last_time_sec >= 1.0)
    {
        std::cout << "=====================" << std::endl;
        // 阈值检查
        if (target_msg_count < 15)
        {
            ROS_ERROR("Target messages count too low: %d (threshold: 15)", target_msg_count);
        }
        else
        {
            ROS_INFO("Target messages received in the last second: %d", target_msg_count);
        }

        if (state_msg_count < 40)
        {
            ROS_ERROR("State bags count too low: %d (threshold: 80)", state_msg_count);
        }
        else
        {
            ROS_INFO("State bags received in the last second: %d", state_msg_count);
        }

        if (gimbal_cmd_count < 5)
        {
            ROS_ERROR("Gimbal commands count too low: %d (threshold: 5)", gimbal_cmd_count);
        }
        else
        {
            ROS_INFO("Gimbal commands received in the last second: %d", gimbal_cmd_count);
        }
        std::cout << "=====================" << std::endl
                  << std::endl;
        // 重置计数器
        target_msg_count = 0;
        state_msg_count = 0;
        gimbal_cmd_count = 0;

        last_time_sec = current_time_sec;
    }
}

void debug_msg_publish()
{
    debug_msg.header.stamp = ros::Time::now();
    debug_msg.header.frame_id = "gimbal_debug";
    if (gimbal_bridge.debug_pub)
    {
        gimbal_bridge.debug_pub.publish(debug_msg);
    }
}

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "gimbal_state_publisher");
    ros::NodeHandle nh;
    pub = nh.advertise<gimbal_bridge_node::GimbalState>("/gimbal/state", 10);
    ros::Subscriber gimbal_attitude_sub = nh.subscribe("/gimbal/cmd", 3, gimbal_cmd_callback);
    ros::Subscriber drone_attitude_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 3, drone_attitude_callback);
    ros::Subscriber bounding_boxes_sub = nh.subscribe<detection_msgs::BoundingBoxes>("/yolov5/detections", 3, bounding_boxes_callback);

    gimbal_bridge.set_debug_publisher(nh.advertise<gimbal_bridge_node::GimbalDebug>("/gimbal/debug", 10));

    // 设置循环频率 (100Hz)
    ros::Rate loop_rate(100);

    while (gimbal_bridge.ping() == 0)
    {
    };
    gimbal_bridge.set_func_mode(FUNC_MODE_FPV);
    gimbal_bridge.request_gimbal_state();

    last_time_sec = ros::Time::now().toSec();

    while (ros::ok())
    {
        ros::spinOnce(); // 处理 ROS 事件队列
        gimbal_bridge.receive_and_publish();
        loop_rate.sleep();
        input_state_check();
        debug_msg_publish();
    }

    return 0;
}
