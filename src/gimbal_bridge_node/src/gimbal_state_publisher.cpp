#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <gimbal_bridge_node/siyi_zr10_protocol.h>
#include <gimbal_bridge_node/GimbalState.h>
#include <gimbal_bridge_node/GimbalCmd.h>

constexpr int RECV_BUF_SIZE = 64;
constexpr int SERVER_PORT = 37260;
constexpr char SERVER_IP[] = "192.168.144.25";

// 创建 Publisher，Topic 名字和消息类型
ros::Publisher pub;

// 创建 Subscriber，订阅云台控制指令
ros::Subscriber sub;

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
    void ping()
    {
        // 定义请求数据
        uint8_t send_buf[RECV_BUF_SIZE] = {0};
        uint16_t request_length = siyi_pack_get_firmware_version(send_buf, RECV_BUF_SIZE, 0x0000);
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
            close(sockfd);
            return;
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
            close(sockfd);
            return;
        }
        FirmwareVersionResponse response;

        // 解析响应
        if (!siyi_unpack_firmware_version_response(recv_buf, recv_len, &response))
        {
            ROS_ERROR("Failed to unpack response data");
            return;
        }
        // 打印接收到的数据
        ROS_INFO("Get firmware version response:");
        ROS_INFO("Code Board Version: %u", response.code_board_ver);
        ROS_INFO("Gimbal Firmware Version: %u", response.gimbal_firmware_ver);
        ROS_INFO("Zoom Firmware Version: %u", response.zoom_firmware_ver);
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
            close(sockfd);
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
        gimbal_bridge_node::GimbalState msg;
        msg.yaw = static_cast<float>(response.yaw) / 10.0f;
        msg.pitch = static_cast<float>(response.pitch) / 10.0f;
        msg.roll = static_cast<float>(response.roll) / 10.0f;
        msg.json_string = "{\"status\":\"normal\"}";
        // 发布消息
        pub.publish(msg);
        return true;
    }

    bool execute_command(const gimbal_bridge_node::GimbalCmd &cmd)
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

private:
    static constexpr int RECV_BUF_SIZE = 64;
    std::string server_ip_;
    uint16_t server_port_;
    struct sockaddr_in send_addr_;
    int sockfd;
    int state_machine = 0; // 云台状态机，0:前下方，1:正下方，2:自由控制
};

// 初始化云台桥接类
gimbal_camera_state_bridge gimbal_bridge(SERVER_IP, SERVER_PORT);

void gimbal_cmd_callback(const gimbal_bridge_node::GimbalCmd::ConstPtr &msg)
{
    // 打印接收到的命令信息
    ROS_INFO("Received Gimbal Command: yaw=%.2f, pitch=%.2f, gimbal_state_machine = %d, json_string=%s",
             msg->yaw, msg->pitch, msg->gimbal_state_machine, msg->json_string.c_str());
    if (msg->gimbal_state_machine < 0 || msg->gimbal_state_machine > 2)
    {
        ROS_ERROR("Invalid gimbal state machine value: %d. It must be 0, 1, or 2.", msg->gimbal_state_machine);
        return;
    }
    gimbal_bridge_node::GimbalCmd gimbal_cmd = *msg;
    ROS_INFO("gimbal bridge state machine: %d", gimbal_bridge.get_state_machine());

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
        gimbal_cmd.pitch = -45;
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
    if (!gimbal_bridge.execute_command(gimbal_cmd))
    {
        ROS_ERROR("Failed to execute gimbal command.");
    }
    else
    {
        ROS_INFO("Gimbal command executed successfully.");
    }
}

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "gimbal_state_publisher");
    ros::NodeHandle nh;
    pub = nh.advertise<gimbal_bridge_node::GimbalState>("/gimbal/state", 10);
    sub = nh.subscribe("/gimbal/cmd", 3, gimbal_cmd_callback);

    // 设置循环频率 (100Hz)
    ros::Rate loop_rate(101);

    // gimbal_camera_state_bridge gimbal_bridge(SERVER_IP, SERVER_PORT);
    std::cout << 1 << std::endl;
    gimbal_bridge.ping();
    std::cout << 2 << std::endl;
    gimbal_bridge.set_func_mode(FUNC_MODE_FPV);
    std::cout << 3 << std::endl;
    gimbal_bridge.request_gimbal_state();
    std::cout << 4 << std::endl;
    // ros::spin();

    while (ros::ok())
    {
        ros::spinOnce(); // 处理 ROS 事件队列
        gimbal_bridge.receive_and_publish();
        loop_rate.sleep();
    }

    return 0;
}
