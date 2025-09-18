# M3508电机控制消息
# 用于控制单个或多个M3508电机

# 控制命令类型
uint8 MODE_CURRENT=1     # 电流控制模式
uint8 MODE_SPEED=2       # 速度控制模式
uint8 mode               # 控制模式

# 电机ID (1-4)
uint8 motor_id           # 要控制的电机ID

# 控制数据
int16 current            # 电流值 (-16384~16384)
int16 speed              # 速度值 (RPM)

# 原始数据，用于直接发送CAN帧
uint8[8] data            # 8字节数据

# 当从CAN总线接收电机反馈时使用的字段
uint16 angle             # 电机角度值 (0~8191)
uint8 temperature        # 电机温度

# 通用命令字段
int16 cmd                # 通用命令值