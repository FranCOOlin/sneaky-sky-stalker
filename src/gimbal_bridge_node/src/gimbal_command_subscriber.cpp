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
#include <gimbal_bridge_node/GimbalCmd.h>

constexpr int RECV_BUF_SIZE = 64;
constexpr int SERVER_PORT = 37260;
constexpr char SERVER_IP[] = "192.168.144.25";

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

private:
    static constexpr int RECV_BUF_SIZE = 64;
    std::string server_ip_;
    uint16_t server_port_;
    struct sockaddr_in send_addr_;
    int sockfd;
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

    if (msg->gimbal_state_machine == 0)
    {
        // 云台状态机为0时，云台指向前下方（pitch = 45， yaw=0）
        ROS_INFO("Gimbal is set to point forward downwards (pitch=45, yaw=0).");
        gimbal_cmd.pitch = 45;
        gimbal_cmd.yaw = 0;
        return;
    }
    else if (msg->gimbal_state_machine == 1)
    {
        // 云台状态机为1时，云台指向正下方（pitch=90，yaw=0）
        ROS_INFO("Gimbal is set to point directly downwards (pitch=90, yaw=0).");
        gimbal_cmd.pitch = 90;
        gimbal_cmd.yaw = 0;
        return;
    }
    // 这里仅仅进行有效性判断，平滑性等更高层次的要求交给控制器进行处理。
    if (msg->yaw < -135 || msg->yaw > 135 ||
        msg->pitch < -90 || msg->pitch > 25)
    {
        ROS_ERROR("Invalid gimbal command: yaw or pitch out of range.");
        return;
    }
    if (!gimbal_bridge.execute_turn_command(gimbal_cmd))
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
    ros::init(argc, argv, "gimbal_command_subscriber");
    ros::NodeHandle nh;

    // 测试 gimbal_bridge 连通性
    gimbal_bridge.ping();

    // 创建 Subscriber，订阅云台控制指令
    ros::Subscriber sub = nh.subscribe("/gimbal/cmd", 1, gimbal_cmd_callback);

    ros::spin();

    return 0;
}