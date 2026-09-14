# GripperRegisters.msg - 实时寄存器数据 (0x0000-0x0009)
# 以100Hz发布夹爪的完整寄存器状态信息

std_msgs/Header header

# 0x0000-0x0005: 力传感器数据 X1, Y1, Z1, X2, Y2, Z2
int16 x1_value        # 0x0000
int16 y1_value        # 0x0001
int16 z1_value        # 0x0002
int16 x2_value        # 0x0003
int16 y2_value        # 0x0004
int16 z2_value        # 0x0005

# 0x0006-0x0009: 状态与控制寄存器
uint16 status         # 0x0006: 状态寄存器 (00/01/02/03)
uint16 softness       # 0x0007: 软硬度寄存器 (0~6000)
uint16 position       # 0x0008: 位置寄存器 (0~100)
uint16 current        # 0x0009: 电流寄存器 (0~1400)
