/**
 * @file siyi_zr10_protocol.c
 * @brief SIYI ZR10云台相机通信协议实现
 * @copyright Copyright 2025 SIYI 思翼科技All Rights Reserved.
 */

#include "gimbal_bridge_node/siyi_zr10_protocol.h"
#include <string.h>
#include <stdio.h>

/**
 * @brief CRC16校验表
 */
static const uint16_t crc16_tab[256] = {
<<<<<<< HEAD
    0x0,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
    0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    0x1231,0x210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
    0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
    0x2462,0x3443,0x420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
    0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
    0x3653,0x2672,0x1611,0x630,0x76d7,0x66f6,0x5695,0x46b4,
    0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
    0x48c4,0x58e5,0x6886,0x78a7,0x840,0x1861,0x2802,0x3823,
    0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
    0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0xa50,0x3a33,0x2a12,
    0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
    0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0xc60,0x1c41,
    0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
    0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0xe70,
    0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
    0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
    0x1080,0xa1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
    0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
    0x2b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
    0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
    0x34e2,0x24c3,0x14a0,0x481,0x7466,0x6447,0x5424,0x4405,
    0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
    0x26d3,0x36f2,0x691,0x16b0,0x6657,0x7676,0x4615,0x5634,
    0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
    0x5844,0x4865,0x7806,0x6827,0x18c0,0x8e1,0x3882,0x28a3,
    0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
    0x4a75,0x5a54,0x6a37,0x7a16,0xaf1,0x1ad0,0x2ab3,0x3a92,
    0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
    0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0xcc1,
    0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
    0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0xed1,0x1ef0
};
=======
    0x0, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0xa50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0xc60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0xe70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0xa1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x2b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x8e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0xaf1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0xcc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0xed1, 0x1ef0};
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc

/**
 * @brief 计算CRC16校验值
 * @param data 数据bag
 * @param len 数据长度
 * @return CRC16校验值
 */
uint16_t CRC16_cal(uint8_t *data, uint32_t len)
{
    uint16_t crc, oldcrc16;
    uint8_t temp;
    crc = 0;
<<<<<<< HEAD
    while (len--!= 0)
=======
    while (len-- != 0)
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    {
        temp = (crc >> 8) & 0xff;
        oldcrc16 = crc16_tab[*data ^ temp];
        crc = (crc << 8) ^ oldcrc16;
        data++;
    }
    return (crc);
<<<<<<< HEAD
} 
=======
}
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc

/**
 * @brief 封装协议数据包
 * @param cmd_id 命令ID
 * @param need_ack 是否需要应答
 * @param data 数据bag
 * @param bag_len 数据长度
 * @param seq 序列号
 * @param bag 输出bag
 * @param bag_size 输出bag大小
 * @return 封装后的数据包长度，失败返回0
 */
<<<<<<< HEAD
uint16_t siyi_pack_data(uint8_t cmd_id, bool need_ack, const uint8_t *data, uint16_t data_len, 
                        uint16_t seq, uint8_t *bag, uint16_t max_bag_len) {
    // 计算总长度：包头(8字节) + 数据(data_len) + CRC16(2字节)
    uint16_t bag_len = 8 + data_len + 2;
    
    // 检查bag大小是否足够
    if (bag_len > max_bag_len) {
        return 0;
    }
    
    // 填充包头
    bag[0] = SIYI_PROTOCOL_HEAD & 0xFF;         // STX低字节
    bag[1] = (SIYI_PROTOCOL_HEAD >> 8) & 0xFF;  // STX高字节
    bag[2] = need_ack ? 0x01 : 0x00;            // CTRL
    bag[3] = data_len & 0xFF;                   // data_len低字节
    bag[4] = (data_len >> 8) & 0xFF;            // data_len高字节
    bag[5] = seq & 0xFF;                        // SEQ低字节
    bag[6] = (seq >> 8) & 0xFF;                 // SEQ高字节
    bag[7] = cmd_id;                            // CMD_ID
    
    // 填充数据
    if (data_len > 0 && data != NULL) {
        memcpy(bag + 8, data, data_len);
    }
    
    // 计算CRC16校验值
    uint16_t crc = CRC16_cal(bag, 8 + data_len);
    
    // 填充CRC16校验值
    bag[8 + data_len] = crc & 0xFF;           // CRC16低字节
    bag[8 + data_len + 1] = (crc >> 8) & 0xFF; // CRC16高字节
    
=======
uint16_t siyi_pack_data(uint8_t cmd_id, bool need_ack, const uint8_t *data, uint16_t data_len,
                        uint16_t seq, uint8_t *bag, uint16_t max_bag_len)
{
    // 计算总长度：包头(8字节) + 数据(data_len) + CRC16(2字节)
    uint16_t bag_len = 8 + data_len + 2;

    // 检查bag大小是否足够
    if (bag_len > max_bag_len)
    {
        return 0;
    }

    // 填充包头
    bag[0] = SIYI_PROTOCOL_HEAD & 0xFF;        // STX低字节
    bag[1] = (SIYI_PROTOCOL_HEAD >> 8) & 0xFF; // STX高字节
    bag[2] = need_ack ? 0x01 : 0x00;           // CTRL
    bag[3] = data_len & 0xFF;                  // data_len低字节
    bag[4] = (data_len >> 8) & 0xFF;           // data_len高字节
    bag[5] = seq & 0xFF;                       // SEQ低字节
    bag[6] = (seq >> 8) & 0xFF;                // SEQ高字节
    bag[7] = cmd_id;                           // CMD_ID

    // 填充数据
    if (data_len > 0 && data != NULL)
    {
        memcpy(bag + 8, data, data_len);
    }

    // 计算CRC16校验值
    uint16_t crc = CRC16_cal(bag, 8 + data_len);

    // 填充CRC16校验值
    bag[8 + data_len] = crc & 0xFF;            // CRC16低字节
    bag[8 + data_len + 1] = (crc >> 8) & 0xFF; // CRC16高字节

>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    return bag_len;
}

/**
 * @brief 解析协议数据包
 * @param bag 输入bag
 * @param bag_len 输入bag长度
 * @param cmd_id 输出命令ID
 * @param is_ack 输出是否为应答包
 * @param seq 输出序列号
 * @param data 输出数据bag
 * @param bag_len 输出数据长度
 * @return 解析成功返回true，失败返回false
 */
<<<<<<< HEAD
bool siyi_unpack_data(const uint8_t *bag, uint16_t bag_len, uint8_t *cmd_id, 
                     bool *is_ack, uint16_t *seq, uint8_t *data, uint16_t *data_len) {
    if (bag_len < 10) {
        return false;
    }
    
    // Verify STX
    uint16_t stx = bag[0] | (bag[1] << 8);
    if (stx != SIYI_PROTOCOL_HEAD) {
        return false;
    }
    
    // Extract data len
    uint16_t len = bag[3] | (bag[4] << 8);
    
    // Verify bag contains complete packet
    if (bag_len < 8 + len + 2) {
        return false;
    }
    
    // Calculate and verify CRC
    uint16_t crc_calc = CRC16_cal(bag, 8 + len);
    uint16_t crc_recv = bag[8 + len] | (bag[8 + len + 1] << 8);
    if (crc_calc != crc_recv) {
        return false;
    }
    
    // Extract metadata
    if (cmd_id) *cmd_id = bag[7];
    if (is_ack) *is_ack = (bag[2] & 0x02) != 0;
    if (seq) *seq = bag[5] | (bag[6] << 8);
    
    // Set data section
    if (data && data_len) {
        *data = bag + 8;
=======
bool siyi_unpack_data(const uint8_t *bag, uint16_t bag_len, uint8_t *cmd_id,
                      bool *is_ack, uint16_t *seq, uint8_t *data, uint16_t *data_len)
{
    if (bag_len < 10)
    {
        return false;
    }

    // Verify STX
    uint16_t stx = bag[0] | (bag[1] << 8);
    if (stx != SIYI_PROTOCOL_HEAD)
    {
        return false;
    }

    // Extract data len
    uint16_t len = bag[3] | (bag[4] << 8);

    // Verify bag contains complete packet
    if (bag_len < 8 + len + 2)
    {
        return false;
    }

    // Calculate and verify CRC
    uint16_t crc_calc = CRC16_cal(bag, 8 + len);
    uint16_t crc_recv = bag[8 + len] | (bag[8 + len + 1] << 8);
    if (crc_calc != crc_recv)
    {
        return false;
    }

    // Extract metadata
    if (cmd_id)
        *cmd_id = bag[7];
    if (is_ack)
        *is_ack = (bag[2] & 0x02) != 0;
    if (seq)
        *seq = bag[5] | (bag[6] << 8);

    // Set data section
    if (data && data_len)
    {
        memcpy(data, bag + 8, len);
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        *data_len = len;
    }

    return true;
}

/**
 * @brief 封装请求云台相机固件版本号命令
 * @param bag 输出bag
 * @param max_bag_len bag最大长度
 * @param seq 序列号
 * @return 封装后的数据包长度，失败返回0
 */
<<<<<<< HEAD
uint16_t siyi_pack_get_firmware_version(uint8_t *bag, uint16_t max_bag_len, uint16_t seq) {
=======
uint16_t siyi_pack_get_firmware_version(uint8_t *bag, uint16_t max_bag_len, uint16_t seq)
{
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    return siyi_pack_data(CMD_GET_FIRMWARE_VERSION, true, NULL, 0, seq, bag, max_bag_len);
}

/**
 * @brief 解析固件版本号响应
 * @param data 数据bag
 * @param bag_len 数据长度
 * @param response 输出响应结构体
 * @return 解析成功返回true，失败返回false
 */
<<<<<<< HEAD
bool siyi_unpack_firmware_version_response(const uint8_t *bag, uint16_t bag_len, FirmwareVersionResponse *response) {
    if (!bag || !response || bag_len < sizeof(FirmwareVersionResponse)) {
=======
bool siyi_unpack_firmware_version_response(const uint8_t *bag, uint16_t bag_len, FirmwareVersionResponse *response)
{
    if (!bag || !response || bag_len < sizeof(FirmwareVersionResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    uint8_t data[sizeof(FirmwareVersionResponse)];
    uint16_t data_len = 0;
<<<<<<< HEAD
    if(!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len)) {
        return false;
    }
    if (data_len != sizeof(FirmwareVersionResponse))  // 确保数据长度正确
=======
    if (!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len))
    {
        return false;
    }
    if (data_len != sizeof(FirmwareVersionResponse)) // 确保数据长度正确
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    {
        return false;
    }
    memcpy(response, data + 8, sizeof(FirmwareVersionResponse));
    return true;
}

/**
 * @brief 封装请求云台相机硬件ID命令
 * @param bag 输出bag
 * @param max_bag_len bag最大长度
 * @param seq 序列号
 * @return 封装后的数据包长度，失败返回0
 */
<<<<<<< HEAD
uint16_t siyi_pack_get_hardware_id(uint8_t *bag, uint16_t max_bag_len, uint16_t seq) {
=======
uint16_t siyi_pack_get_hardware_id(uint8_t *bag, uint16_t max_bag_len, uint16_t seq)
{
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    return siyi_pack_data(CMD_GET_HARDWARE_ID, true, NULL, 0, seq, bag, max_bag_len);
}

/**
 * @brief 解析硬件ID响应
 * @param bag 数据bag
 * @param bag_len 数据长度
 * @param response 输出响应结构体
 * @return 解析成功返回true，失败返回false
 */
<<<<<<< HEAD
bool siyi_unpack_hardware_id_response(const uint8_t *bag, uint16_t bag_len, HardwareIdResponse *response) {
    if (!bag || !response || bag_len < sizeof(HardwareIdResponse)) {
=======
bool siyi_unpack_hardware_id_response(const uint8_t *bag, uint16_t bag_len, HardwareIdResponse *response)
{
    if (!bag || !response || bag_len < sizeof(HardwareIdResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    uint8_t data[sizeof(HardwareIdResponse)];
    uint16_t data_len = 0;
<<<<<<< HEAD
    if(!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len)) {
        return false;
    }
    if (data_len != sizeof(HardwareIdResponse)) {
=======
    if (!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len))
    {
        return false;
    }
    if (data_len != sizeof(HardwareIdResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    memcpy(response, data + 8, sizeof(HardwareIdResponse));
    return true;
}

/**
 * @brief 封装请求云台相机当前工作模式命令
 * @param bag 输出bag
 * @param max_bag_len bag最大长度
 * @param seq 序列号
 * @return 封装后的数据包长度，失败返回0
 */
<<<<<<< HEAD
uint16_t siyi_pack_get_gimbal_mode(uint8_t *bag, uint16_t max_bag_len, uint16_t seq) {
=======
uint16_t siyi_pack_get_gimbal_mode(uint8_t *bag, uint16_t max_bag_len, uint16_t seq)
{
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    return siyi_pack_data(CMD_GET_GIMBAL_MODE, true, NULL, 0, seq, bag, max_bag_len);
}

/**
 * @brief 解析云台工作模式响应
 * @param bag 数据bag
 * @param bag_len 数据长度
 * @param response 输出响应结构体
 * @return 解析成功返回true，失败返回false
 */
<<<<<<< HEAD
bool siyi_unpack_gimbal_mode_response(const uint8_t *bag, uint16_t bag_len, GimbalModeResponse *response) {
    if (!bag || !response || bag_len < sizeof(GimbalModeResponse)) {
=======
bool siyi_unpack_gimbal_mode_response(const uint8_t *bag, uint16_t bag_len, GimbalModeResponse *response)
{
    if (!bag || !response || bag_len < sizeof(GimbalModeResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    uint8_t data[sizeof(GimbalModeResponse)];
    uint16_t data_len = 0;
<<<<<<< HEAD
    if(!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len)) {
        return false;
    }
    if (data_len != sizeof(GimbalModeResponse)) {
=======
    if (!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len))
    {
        return false;
    }
    if (data_len != sizeof(GimbalModeResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    memcpy(response, data + 8, sizeof(GimbalModeResponse));
    return true;
}

/**
 * @brief 封装自动对焦命令
 * @param bag 输出bag
 * @param max_bag_len bag最大长度
 * @param auto_focus 是否自动对焦
 * @param touch_x 触摸点x坐标
 * @param touch_y 触摸点y坐标
 * @param seq 序列号
 * @return 封装后的数据包长度，失败返回0
 */
<<<<<<< HEAD
uint16_t siyi_pack_auto_focus(uint8_t *bag, uint16_t max_bag_len, uint8_t auto_focus, 
                              uint16_t touch_x, uint16_t touch_y, uint16_t seq) {
=======
uint16_t siyi_pack_auto_focus(uint8_t *bag, uint16_t max_bag_len, uint8_t auto_focus,
                              uint16_t touch_x, uint16_t touch_y, uint16_t seq)
{
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    AutoFocusRequest request;
    request.auto_focus = auto_focus;
    request.touch_x = touch_x;
    request.touch_y = touch_y;
<<<<<<< HEAD
    
=======

>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    return siyi_pack_data(CMD_AUTO_FOCUS, true, (const uint8_t *)&request, sizeof(request), seq, bag, max_bag_len);
}

/**
 * @brief 解析通用状态响应
 * @param bag 数据bag
 * @param bag_len 数据长度
 * @param response 输出响应结构体
 * @return 解析成功返回true，失败返回false
 */
<<<<<<< HEAD
bool siyi_unpack_status_response(const uint8_t *bag, uint16_t bag_len, StatusResponse *response) {
    if (!bag || !response || bag_len < sizeof(StatusResponse)) {
=======
bool siyi_unpack_status_response(const uint8_t *bag, uint16_t bag_len, StatusResponse *response)
{
    if (!bag || !response || bag_len < sizeof(StatusResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    uint8_t data[sizeof(StatusResponse)];
    uint16_t data_len = 0;
<<<<<<< HEAD
    if(!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len)) {
        return false;
    }
    if (data_len != sizeof(StatusResponse)) {
=======
    if (!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len))
    {
        return false;
    }
    if (data_len != sizeof(StatusResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    memcpy(response, data + 8, sizeof(StatusResponse));
    return true;
}

/**
 * @brief 封装手动变倍自动对焦命令
 * @param bag 输出bag
 * @param max_bag_len bag最大长度
 * @param zoom 变焦值 1:放大 0:停止缩放 -1:缩小
 * @param seq 序列号
 * @return 封装后的数据包长度，失败返回0
 */
<<<<<<< HEAD
uint16_t siyi_pack_manual_zoom(uint8_t *bag, uint16_t max_bag_len, int8_t zoom, uint16_t seq) {
    ManualZoomRequest request;
    request.zoom = zoom;
    
=======
uint16_t siyi_pack_manual_zoom(uint8_t *bag, uint16_t max_bag_len, int8_t zoom, uint16_t seq)
{
    ManualZoomRequest request;
    request.zoom = zoom;

>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    return siyi_pack_data(CMD_MANUAL_ZOOM_AUTO_FOCUS, true, (const uint8_t *)&request, sizeof(request), seq, bag, max_bag_len);
}

/**
 * @brief 解析变焦倍数响应
 * @param bag 数据bag
 * @param bag_len 数据长度
 * @param response 输出响应结构体
 * @return 解析成功返回true，失败返回false
 */
<<<<<<< HEAD
bool siyi_unpack_zoom_multiple_response(const uint8_t *bag, uint16_t bag_len, ZoomMultipleResponse *response) {
    if (!bag || !response || bag_len < sizeof(ZoomMultipleResponse)) {
=======
bool siyi_unpack_zoom_multiple_response(const uint8_t *bag, uint16_t bag_len, ZoomMultipleResponse *response)
{
    if (!bag || !response || bag_len < sizeof(ZoomMultipleResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    uint8_t data[sizeof(ZoomMultipleResponse)];
    uint16_t data_len = 0;
<<<<<<< HEAD
    if(!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len)) {
        return false;
    }
    if (data_len != sizeof(ZoomMultipleResponse)) {
=======
    if (!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len))
    {
        return false;
    }
    if (data_len != sizeof(ZoomMultipleResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    memcpy(response, data + 8, sizeof(ZoomMultipleResponse));
    return true;
}

/**
 * @brief 封装绝对变倍自动对焦命令
 * @param bag 输出bag
 * @param max_bag_len bag最大长度
 * @param int_part 整数部分
 * @param float_part 小数部分
 * @param seq 序列号
 * @return 封装后的数据包长度，失败返回0
 */
<<<<<<< HEAD
uint16_t siyi_pack_absolute_zoom(uint8_t *bag, uint16_t max_bag_len, uint8_t int_part, 
                                 uint8_t float_part, uint16_t seq) {
    AbsoluteZoomRequest request;
    request.absolute_movement_int = int_part;
    request.absolute_movement_float = float_part;
    
=======
uint16_t siyi_pack_absolute_zoom(uint8_t *bag, uint16_t max_bag_len, uint8_t int_part,
                                 uint8_t float_part, uint16_t seq)
{
    AbsoluteZoomRequest request;
    request.absolute_movement_int = int_part;
    request.absolute_movement_float = float_part;

>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    return siyi_pack_data(CMD_ABSOLUTE_ZOOM, true, (const uint8_t *)&request, sizeof(request), seq, bag, max_bag_len);
}

/**
 * @brief 解析绝对变倍自动对焦响应
 * @param bag 数据bag
 * @param bag_len 数据长度
 * @param response 输出响应结构体
 * @return 解析成功返回true，失败返回false
 */
<<<<<<< HEAD
bool siyi_unpack_absolute_zoom_response(const uint8_t *bag, uint16_t bag_len, AbsoluteZoomResponse *response) {
    if (!bag || !response || bag_len < sizeof(AbsoluteZoomResponse)) {
=======
bool siyi_unpack_absolute_zoom_response(const uint8_t *bag, uint16_t bag_len, AbsoluteZoomResponse *response)
{
    if (!bag || !response || bag_len < sizeof(AbsoluteZoomResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    uint8_t data[sizeof(AbsoluteZoomResponse)];
    uint16_t data_len = 0;
<<<<<<< HEAD
    if(!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len)) {
        return false;
    }
    if (data_len != sizeof(AbsoluteZoomResponse)) {
=======
    if (!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len))
    {
        return false;
    }
    if (data_len != sizeof(AbsoluteZoomResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    memcpy(response, data + 8, sizeof(AbsoluteZoomResponse));
    return true;
}

/**
 * @brief 封装请求当前状态最大变倍值命令
 * @param bag 输出bag
 * @param max_bag_len bag最大长度
 * @param seq 序列号
 * @return 封装后的数据包长度，失败返回0
 */
<<<<<<< HEAD
uint16_t siyi_pack_get_max_zoom(uint8_t *bag, uint16_t max_bag_len, uint16_t seq) {
=======
uint16_t siyi_pack_get_max_zoom(uint8_t *bag, uint16_t max_bag_len, uint16_t seq)
{
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    return siyi_pack_data(CMD_GET_MAX_ZOOM, true, NULL, 0, seq, bag, max_bag_len);
}

/**
 * @brief 解析最大变倍值响应
 * @param bag 数据bag
 * @param bag_len 数据长度
 * @param response 输出响应结构体
 * @return 解析成功返回true，失败返回false
 */
<<<<<<< HEAD
bool siyi_unpack_max_zoom_response(const uint8_t *bag, uint16_t bag_len, MaxZoomResponse *response) {
    if (!bag || !response || bag_len < sizeof(MaxZoomResponse)) {
=======
bool siyi_unpack_max_zoom_response(const uint8_t *bag, uint16_t bag_len, MaxZoomResponse *response)
{
    if (!bag || !response || bag_len < sizeof(MaxZoomResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    uint8_t data[sizeof(MaxZoomResponse)];
    uint16_t data_len = 0;
<<<<<<< HEAD
    if(!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len)) {
        return false;
    }
    if (data_len != sizeof(MaxZoomResponse)) {
=======
    if (!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len))
    {
        return false;
    }
    if (data_len != sizeof(MaxZoomResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    memcpy(response, data + 8, sizeof(MaxZoomResponse));
    return true;
}

/**
 * @brief 封装请求当前变倍值命令
 * @param bag 输出bag
 * @param max_bag_len bag最大长度
 * @param seq 序列号
 * @return 封装后的数据包长度，失败返回0
 */
<<<<<<< HEAD
uint16_t siyi_pack_get_current_zoom(uint8_t *bag, uint16_t max_bag_len, uint16_t seq) {
=======
uint16_t siyi_pack_get_current_zoom(uint8_t *bag, uint16_t max_bag_len, uint16_t seq)
{
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    return siyi_pack_data(CMD_GET_CURRENT_ZOOM, true, NULL, 0, seq, bag, max_bag_len);
}

/**
 * @brief 解析当前变倍值响应
 * @param bag 数据bag
 * @param bag_len 数据长度
 * @param response 输出响应结构体
 * @return 解析成功返回true，失败返回false
 */
<<<<<<< HEAD
bool siyi_unpack_current_zoom_response(const uint8_t *bag, uint16_t bag_len, CurrentZoomResponse *response) {
    if (!bag || !response || bag_len < sizeof(CurrentZoomResponse)) {
=======
bool siyi_unpack_current_zoom_response(const uint8_t *bag, uint16_t bag_len, CurrentZoomResponse *response)
{
    if (!bag || !response || bag_len < sizeof(CurrentZoomResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    uint8_t data[sizeof(CurrentZoomResponse)];
    uint16_t data_len = 0;
<<<<<<< HEAD
    if(!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len)) {
        return false;
    }
    if (data_len != sizeof(CurrentZoomResponse)) {
=======
    if (!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len))
    {
        return false;
    }
    if (data_len != sizeof(CurrentZoomResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    memcpy(response, data + 8, sizeof(CurrentZoomResponse));
    return true;
}

/**
 * @brief 封装手动对焦命令
 * @param bag 输出bag
 * @param max_bag_len bag最大长度
 * @param focus 对焦值 1:远景 0:停止对焦 -1:近景
 * @param seq 序列号
 * @return 封装后的数据包长度，失败返回0
 */
<<<<<<< HEAD
uint16_t siyi_pack_manual_focus(uint8_t *bag, uint16_t max_bag_len, int8_t focus, uint16_t seq) {
    ManualFocusRequest request;
    request.focus = focus;
    
=======
uint16_t siyi_pack_manual_focus(uint8_t *bag, uint16_t max_bag_len, int8_t focus, uint16_t seq)
{
    ManualFocusRequest request;
    request.focus = focus;

>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    return siyi_pack_data(CMD_MANUAL_FOCUS, true, (const uint8_t *)&request, sizeof(request), seq, bag, max_bag_len);
}

/**
 * @brief 封装云台转向命令
 * @param bag 输出bag
 * @param max_bag_len bag最大长度
 * @param turn_yaw 偏航转向值 -100~0~100
 * @param turn_pitch 俯仰转向值 -100~0~100
 * @param seq 序列号
 * @return 封装后的数据包长度，失败返回0
 */
<<<<<<< HEAD
uint16_t siyi_pack_gimbal_rotation(uint8_t *bag, uint16_t max_bag_len, int8_t turn_yaw, 
                                   int8_t turn_pitch, uint16_t seq) {
    GimbalRotationRequest request;
    request.turn_yaw = turn_yaw;
    request.turn_pitch = turn_pitch;
    
=======
uint16_t siyi_pack_gimbal_rotation(uint8_t *bag, uint16_t max_bag_len, int8_t turn_yaw,
                                   int8_t turn_pitch, uint16_t seq)
{
    GimbalRotationRequest request;
    request.turn_yaw = turn_yaw;
    request.turn_pitch = turn_pitch;

>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    return siyi_pack_data(CMD_GIMBAL_ROTATION, true, (const uint8_t *)&request, sizeof(request), seq, bag, max_bag_len);
}

/**
 * @brief 封装一键回中命令
 * @param bag 输出bag
 * @param max_bag_len bag最大长度
 * @param seq 序列号
 * @return 封装后的数据包长度，失败返回0
 */
<<<<<<< HEAD
uint16_t siyi_pack_gimbal_center(uint8_t *bag, uint16_t max_bag_len, uint16_t seq) {
    GimbalCenterRequest request;
    request.center_pos = 1;  // 触发回中
    
=======
uint16_t siyi_pack_gimbal_center(uint8_t *bag, uint16_t max_bag_len, uint16_t seq)
{
    GimbalCenterRequest request;
    request.center_pos = 1; // 触发回中

>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    return siyi_pack_data(CMD_GIMBAL_CENTER, true, (const uint8_t *)&request, sizeof(request), seq, bag, max_bag_len);
}

/**
 * @brief 封装请求云台配置信息命令
 * @param bag 输出bag
 * @param max_bag_len bag最大长度
 * @param seq 序列号
 * @return 封装后的数据包长度，失败返回0
 */
<<<<<<< HEAD
uint16_t siyi_pack_get_gimbal_config(uint8_t *bag, uint16_t max_bag_len, uint16_t seq) {
=======
uint16_t siyi_pack_get_gimbal_config(uint8_t *bag, uint16_t max_bag_len, uint16_t seq)
{
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    return siyi_pack_data(CMD_GET_GIMBAL_CONFIG, true, NULL, 0, seq, bag, max_bag_len);
}

/**
 * @brief 解析云台配置信息响应
 * @param bag 数据bag
 * @param bag_len 数据长度
 * @param response 输出响应结构体
 * @return 解析成功返回true，失败返回false
 */
<<<<<<< HEAD
bool siyi_unpack_gimbal_config_response(const uint8_t *bag, uint16_t bag_len, GimbalConfigResponse *response) {
    if (!bag || !response || bag_len < sizeof(GimbalConfigResponse)) {
=======
bool siyi_unpack_gimbal_config_response(const uint8_t *bag, uint16_t bag_len, GimbalConfigResponse *response)
{
    if (!bag || !response || bag_len < sizeof(GimbalConfigResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    uint8_t data[sizeof(GimbalConfigResponse)];
    uint16_t data_len = 0;
<<<<<<< HEAD
    if(!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len)) {
        return false;
    }
    if (data_len != sizeof(GimbalConfigResponse)) {
=======
    if (!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len))
    {
        return false;
    }
    if (data_len != sizeof(GimbalConfigResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    memcpy(response, data + 8, sizeof(GimbalConfigResponse));
    return true;
}

/**
 * @brief 解析功能反馈信息响应
 * @param bag 数据bag
 * @param bag_len 数据长度
 * @param response 输出响应结构体
 * @return 解析成功返回true，失败返回false
 */
<<<<<<< HEAD
bool siyi_unpack_function_feedback_response(const uint8_t *bag, uint16_t bag_len, FunctionFeedbackResponse *response) {
    if (!bag || !response || bag_len < sizeof(FunctionFeedbackResponse)) {
=======
bool siyi_unpack_function_feedback_response(const uint8_t *bag, uint16_t bag_len, FunctionFeedbackResponse *response)
{
    if (!bag || !response || bag_len < sizeof(FunctionFeedbackResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    uint8_t data[sizeof(FunctionFeedbackResponse)];
    uint16_t data_len = 0;
<<<<<<< HEAD
    if(!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len)) {
        return false;
    }
    if (data_len != sizeof(FunctionFeedbackResponse)) {
=======
    if (!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len))
    {
        return false;
    }
    if (data_len != sizeof(FunctionFeedbackResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    memcpy(response, data + 8, sizeof(FunctionFeedbackResponse));
    return true;
}

/**
 * @brief 封装拍照、录像等命令
 * @param bag 输出bag
 * @param max_bag_len bag最大长度
 * @param func_type 功能类型
 * @param seq 序列号
 * @return 封装后的数据包长度，失败返回0
 */
<<<<<<< HEAD
uint16_t siyi_pack_function_control(uint8_t *bag, uint16_t max_bag_len, uint8_t func_type, uint16_t seq) {
    FunctionControlRequest request;
    request.func_type = func_type;
    
=======
uint16_t siyi_pack_function_control(uint8_t *bag, uint16_t max_bag_len, uint8_t func_type, uint16_t seq)
{
    FunctionControlRequest request;
    request.func_type = func_type;

>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    return siyi_pack_data(CMD_PHOTO_VIDEO_CONTROL, false, (const uint8_t *)&request, sizeof(request), seq, bag, max_bag_len);
}

/**
 * @brief 封装发送控制角度到云台命令
 * @param bag 输出bag
 * @param max_bag_len bag最大长度
 * @param yaw 偏航角度
 * @param pitch 俯仰角度
 * @param seq 序列号
 * @return 封装后的数据包长度，失败返回0
 */
<<<<<<< HEAD
uint16_t siyi_pack_control_angle(uint8_t *bag, uint16_t max_bag_len, int16_t yaw, int16_t pitch, uint16_t seq) {
    ControlAngleRequest request;
    request.yaw = yaw;
    request.pitch = pitch;
    
=======
uint16_t siyi_pack_control_angle(uint8_t *bag, uint16_t max_bag_len, int16_t yaw, int16_t pitch, uint16_t seq)
{
    ControlAngleRequest request;
    request.yaw = yaw;
    request.pitch = pitch;

>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    return siyi_pack_data(CMD_CONTROL_ANGLE, true, (const uint8_t *)&request, sizeof(request), seq, bag, max_bag_len);
}

/**
 * @brief 解析角度响应
 * @param bag 数据bag
 * @param bag_len 数据长度
 * @param response 输出响应结构体
 * @return 解析成功返回true，失败返回false
 */
<<<<<<< HEAD
bool siyi_unpack_angle_response(const uint8_t *bag, uint16_t bag_len, AngleResponse *response) {
    if (!bag || !response || bag_len < sizeof(AngleResponse)) {
=======
bool siyi_unpack_angle_response(const uint8_t *bag, uint16_t bag_len, AngleResponse *response)
{
    if (!bag || !response || bag_len < sizeof(AngleResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    uint8_t data[sizeof(AngleResponse)];
    uint16_t data_len = 0;
<<<<<<< HEAD
    if(!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len)) {
        return false;
    }
    if (data_len != sizeof(AngleResponse)) {
=======
    if (!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len))
    {
        return false;
    }
    if (data_len != sizeof(AngleResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    memcpy(response, data + 8, sizeof(AngleResponse));
    return true;
}

/**
 * @brief 解析姿态数据响应
 * @param bag 数据bag
 * @param bag_len 数据长度
 * @param response 输出响应结构体
 * @return 解析成功返回true，失败返回false
 */
<<<<<<< HEAD
bool siyi_unpack_attitude_data_response(const uint8_t *bag, uint16_t bag_len, AttitudeDataResponse *response) {
    if (!bag || !response || bag_len < sizeof(AttitudeDataResponse)) {
=======
bool siyi_unpack_attitude_data_response(const uint8_t *bag, uint16_t bag_len, AttitudeDataResponse *response)
{
    if (!bag || !response || bag_len < sizeof(AttitudeDataResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    uint8_t data[sizeof(AttitudeDataResponse)];
    uint16_t data_len = 0;
<<<<<<< HEAD
    if(!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len)) {
        return false;
    }
    if (data_len != sizeof(AttitudeDataResponse)) {
        return false;
    }
    printf("%d\n", data_len);
    for (uint8_t *prt = bag + 8; prt < bag + 8 + data_len; prt++)
    {
        printf("%02x ", *prt);
    }
    printf("\n");

    memcpy(response, data + 8, sizeof(AttitudeDataResponse));
    for (uint8_t *prt = response; prt < response + sizeof(AttitudeDataResponse); prt++)
    {
        printf("%02x ", *prt);
    }
    printf("\n");
=======
    if (!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len))
    {
        return false;
    }
    if (data_len != sizeof(AttitudeDataResponse))
    {
        return false;
    }
    memcpy(response, data, data_len);
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    return true;
}

/**
 * @brief 封装请求云台发送数据流命令
 * @param bag 输出bag
 * @param max_bag_len bag最大长度
 * @param data_type 数据类型
 * @param data_freq 数据频率
 * @param seq 序列号
 * @return 封装后的数据包长度，失败返回0
 */
<<<<<<< HEAD
uint16_t siyi_pack_request_data_stream(uint8_t *bag, uint16_t max_bag_len, uint8_t data_type, 
                                       uint8_t data_freq, uint16_t seq) {
    RequestDataStreamRequest request;
    request.data_type = data_type;
    request.data_freq = data_freq;
    
=======
uint16_t siyi_pack_request_data_stream(uint8_t *bag, uint16_t max_bag_len, uint8_t data_type,
                                       uint8_t data_freq, uint16_t seq)
{
    RequestDataStreamRequest request;
    request.data_type = data_type;
    request.data_freq = data_freq;

>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
    return siyi_pack_data(CMD_REQUEST_GIMBAL_DATA_STREAM, true, (const uint8_t *)&request, sizeof(request), seq, bag, max_bag_len);
}

/**
 * @brief 解析请求云台发送数据流响应
 * @param bag 数据bag
 * @param bag_len 数据长度
 * @param response 输出响应结构体
 * @return 解析成功返回true，失败返回false
 */
<<<<<<< HEAD
bool siyi_unpack_request_data_stream_response(const uint8_t *bag, uint16_t bag_len, RequestDataStreamResponse *response) {
    if (!bag || !response || bag_len < sizeof(RequestDataStreamResponse)) {
=======
bool siyi_unpack_request_data_stream_response(const uint8_t *bag, uint16_t bag_len, RequestDataStreamResponse *response)
{
    if (!bag || !response || bag_len < sizeof(RequestDataStreamResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    uint8_t data[sizeof(RequestDataStreamResponse)];
    uint16_t data_len = 0;
<<<<<<< HEAD
    if(!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len)) {
        return false;
    }
    if (data_len != sizeof(RequestDataStreamResponse)) {
=======
    if (!siyi_unpack_data(bag, bag_len, NULL, NULL, NULL, (uint8_t *)data, &data_len))
    {
        return false;
    }
    if (data_len != sizeof(RequestDataStreamResponse))
    {
>>>>>>> 72f602e1751dfffabca6b5833a9f397d87336ccc
        return false;
    }
    memcpy(response, data + 8, sizeof(RequestDataStreamResponse));
    return true;
}
