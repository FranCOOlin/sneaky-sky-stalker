Header header

# 云台控制指令
float32 yaw     # 偏航角
float32 pitch   # 俯仰角
int32 gimbal_state_machine # 云台状态机
int32 zoom_in_state
string json_string  # 附加的JSON字符串

# 当云台状态机为 0 的时候，云台指向前下方（pitch = 45， yaw=0），当云台状态机为 1 的时候，云台指向正下方（pitch=90，yaw=0），当云台状态机为2的时候，云台指向yaw和pitch定义的方向，当云台状态机为 3 的时候，云台指向正下方（pitch=90，yaw=0），且进入zoomin模式，放大倍率为2，当退出状态3的时候，退出zoomin模式，放大倍率恢复为1。
