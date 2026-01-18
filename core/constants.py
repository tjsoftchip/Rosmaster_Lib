#!/usr/bin/env python3
# coding: utf-8

"""
Rosmaster_Lib 常量定义

包含所有车型常量、功能码常量和物理参数
"""

# ============================================================================
# 车型常量
# ============================================================================
CARTYPE_X3 = 0x01
CARTYPE_X3_PLUS = 0x02
CARTYPE_X1 = 0x04
CARTYPE_R2 = 0x05
CARTYPE_R2_MSSD = 0x06

# 车型名称映射
CAR_TYPE_NAMES = {
    0x01: "X3",
    0x02: "X3_PLUS",
    0x04: "X1",
    0x05: "R2",
    0x06: "R2_MSSD"
}

# ============================================================================
# 功能码常量
# ============================================================================
FUNC_AUTO_REPORT = 0x01
FUNC_BEEP = 0x02
FUNC_PWM_SERVO = 0x03
FUNC_PWM_SERVO_ALL = 0x04
FUNC_RGB = 0x05
FUNC_RGB_EFFECT = 0x06

FUNC_REPORT_SPEED = 0x0A
FUNC_REPORT_MPU_RAW = 0x0B
FUNC_REPORT_IMU_ATT = 0x0C
FUNC_REPORT_ENCODER = 0x0D
FUNC_REPORT_ICM_RAW = 0x0E
FUNC_RESET_STATE = 0x0F

FUNC_MOTOR = 0x10
FUNC_CAR_RUN = 0x11
FUNC_MOTION = 0x12
FUNC_SET_MOTOR_PID = 0x13
FUNC_SET_YAW_PID = 0x14
FUNC_SET_CAR_TYPE = 0x15

FUNC_UART_SERVO = 0x20
FUNC_UART_SERVO_ID = 0x21
FUNC_UART_SERVO_TORQUE = 0x22
FUNC_ARM_CTRL = 0x23
FUNC_ARM_OFFSET = 0x24

FUNC_AKM_DEF_ANGLE = 0x30
FUNC_AKM_STEER_ANGLE = 0x31

FUNC_LIMIT_SWITCH = 0x33

FUNC_REQUEST_DATA = 0x50
FUNC_VERSION = 0x51

FUNC_RESET_FLASH = 0xA0

# ============================================================================
# 物理参数常量
# ============================================================================
# R2 机器人参数
WHEELBASE_R2 = 1.355  # 轴距 1355mm = 1.355m
TRACK_WIDTH_R2 = 1.140  # 轮距 1140mm = 1.140m
TIRE_DIAMETER_R2 = 0.65  # 轮胎直径 650mm = 0.65m
MSSD_GEAR_RATIO = 41  # MSSD 减速比 41:1

# 物理常量
GRAVITY = 9.81  # 重力加速度 (m/s²)
WATER_DENSITY = 1000.0  # 水的密度 (kg/m³)

# ============================================================================
# 串口配置常量
# ============================================================================
# 默认串口路径
DEFAULT_STM32_PORT = "/dev/myserial"
DEFAULT_MSSD_PORT = "/dev/mssd"
DEFAULT_RELAY_PORT = "/dev/modbus"

# 默认波特率
DEFAULT_BAUDRATE = 115200
DEFAULT_MODBUS_BAUDRATE = 9600  # Modbus设备默认波特率（水位传感器、电池库仑计等）

# ============================================================================
# 设备地址常量
# ============================================================================
MSSD_DEVICE_ID = 1  # MSSD 驱动器地址
RELAY_DEVICE_ID = 5  # 继电器模块地址
WATER_SENSOR_DEVICE_ID = 6  # 水位传感器地址
BATTERY_MONITOR_DEVICE_ID = 7  # 电池库仑计地址

# ============================================================================
# 其他常量
# ============================================================================
# STM32 通信协议常量
HEAD = 0xFF
DEVICE_ID = 0xFC
COMPLEMENT = 257 - DEVICE_ID
CAR_ADJUST = 0x80

# MSSD 寄存器地址
MSSD_REG_CURRENT = 0x0007  # 电流
MSSD_REG_SPEED = 0x0009  # 速度
MSSD_REG_ENCODER = 0x0016  # 编码器
MSSD_REG_CONTROL = 0x0014  # 控制寄存器
MSSD_REG_TARGET_SPEED = 0x0021  # 目标速度

# 继电器线圈地址
RELAY_COIL_LEFT_ARM = 0  # 左侧展臂
RELAY_COIL_LEFT_VALVE = 1  # 左侧水阀
RELAY_COIL_RIGHT_ARM = 2  # 右侧展臂
RELAY_COIL_RIGHT_VALVE = 3  # 右侧水阀
RELAY_COIL_PUMP_POWER = 4  # 水泵电源
RELAY_COIL_PUMP_SPRAY = 5  # 水泵喷水开关
RELAY_COIL_RAISE_MOUNT = 6  # 升起喷水支架
RELAY_COIL_LOWER_MOUNT = 7  # 降下喷水支架