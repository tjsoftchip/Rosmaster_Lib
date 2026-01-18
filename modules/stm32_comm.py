#!/usr/bin/env python3
# coding: utf-8

"""
STM32 通信模块

用于与 STM32 底盘控制器通信
支持运动控制、IMU、编码器、限位开关等功能

注意：此模块目前仅作为接口定义，实际功能仍在 Rosmaster_Lib.py 中实现
后续可以逐步迁移 STM32 通信逻辑到此模块

作者：nx-ros2
日期：2026-01-16
"""

import serial
import time
import struct
import threading
from ..core.constants import (
    HEAD, DEVICE_ID, COMPLEMENT, CAR_ADJUST,
    DEFAULT_BAUDRATE
)


class STM32Communicator:
    """
    STM32 通信控制器
    
    功能：
    - 运动控制
    - IMU 数据读取
    - 编码器数据读取
    - 限位开关检测
    - 舵机控制
    - 机械臂控制
    """
    
    def __init__(self, port="/dev/myserial", baudrate=DEFAULT_BAUDRATE, 
                 delay=0.002, debug=False):
        """
        初始化 STM32 通信控制器
        
        Args:
            port: 串口设备路径（默认 /dev/myserial）
            baudrate: 波特率（默认 115200）
            delay: 指令延迟时间（秒）
            debug: 调试模式
        """
        self.port = port
        self.baudrate = baudrate
        self.delay = delay
        self.debug = debug
        self.ser = None
        self.connected = False
        
        # 数据缓存
        self.vx = 0  # 线速度
        self.vy = 0  # 转向角速度
        self.vz = 0  # 角速度
        self.ax = 0  # 加速度 X
        self.ay = 0  # 加速度 Y
        self.az = 0  # 加速度 Z
        self.gx = 0  # 陀螺仪 X
        self.gy = 0  # 陀螺仪 Y
        self.gz = 0  # 陀螺仪 Z
        self.mx = 0  # 磁力计 X
        self.my = 0  # 磁力计 Y
        self.mz = 0  # 磁力计 Z
        self.roll = 0  # 横滚角
        self.pitch = 0  # 俯仰角
        self.yaw = 0  # 偏航角
        self.encoder_m1 = 0  # 编码器1
        self.encoder_m2 = 0  # 编码器2
        self.encoder_m3 = 0  # 编码器3
        self.encoder_m4 = 0  # 编码器4
        self.battery_voltage = 0  # 电池电压
        self.limit_switch_state = 0  # 限位开关状态
        self.version_H = 0  # 版本号高位
        self.version_L = 0  # 版本号低位
        
        # 阿克曼转向参数
        self.AKM_SERVO_ID = 0x01  # 阿克曼舵机 ID
        
        # 接收线程
        self.receive_thread = None
        self.uart_state = 0
        
        # 限位开关状态变化回调
        self.limit_switch_callback = None
        
        # 连接串口
        self._connect()
    
    def _connect(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.connected = True
            
            if self.debug:
                print(f"[STM32] 串口连接成功: {self.port}")
                print(f"[STM32] 波特率: {self.baudrate}")
        except Exception as e:
            self.connected = False
            if self.debug:
                print(f"[STM32] 串口连接失败: {e}")
    
    def disconnect(self):
        """断开串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.connected = False
            if self.debug:
                print("[STM32] 串口已断开")
    
    def start_receive_thread(self):
        """启动接收线程"""
        if self.uart_state == 0:
            self.receive_thread = threading.Thread(
                target=self._receive_data,
                name="stm32_receive_thread"
            )
            self.receive_thread.setDaemon(True)
            self.receive_thread.start()
            self.uart_state = 1
            
            if self.debug:
                print("[STM32] 接收线程已启动")
    
    def _receive_data(self):
        """
        接收数据线程
        
        从串口接收数据并解析
        """
        from ..core.constants import (
            FUNC_REPORT_SPEED, FUNC_REPORT_MPU_RAW, FUNC_REPORT_ICM_RAW,
            FUNC_REPORT_IMU_ATT, FUNC_REPORT_ENCODER, FUNC_VERSION,
            FUNC_LIMIT_SWITCH
        )
        
        while True:
            try:
                # 读取帧头
                head1 = bytearray(self.ser.read(1))[0]
                if head1 == HEAD:
                    head2 = bytearray(self.ser.read(1))[0]
                    
                    if head2 == DEVICE_ID - 1:
                        # 读取长度和功能码
                        ext_len = bytearray(self.ser.read(1))[0]
                        ext_type = bytearray(self.ser.read(1))[0]
                        
                        # 读取数据（完全按照原始代码的逻辑）
                        data_len = ext_len - 2
                        ext_data = []
                        check_sum = ext_len + ext_type
                        rx_check_num = 0
                        
                        while len(ext_data) < data_len:
                            value = bytearray(self.ser.read(1))[0]
                            ext_data.append(value)
                            if len(ext_data) == data_len:
                                # 最后一个字节是校验和
                                rx_check_num = value
                            else:
                                # 其他字节是数据，累加到校验和
                                check_sum = check_sum + value
                        
                        # 校验和验证
                        if check_sum % 256 == rx_check_num:
                            if self.debug and ext_type in [0x0A, 0x0C, 0x0B, 0x33]:  # 只打印关键数据
                                print(f"[STM32] 接收成功: type={ext_type}, len={ext_len}")
                            self._parse_data(ext_type, ext_data)
                        else:
                            if self.debug:
                                print(f"[STM32] 校验和错误: type={ext_type}, len={ext_len}, calc={check_sum%256}, recv={rx_check_num}, data={ext_data[:5] if ext_data else []}")
                    
            except serial.SerialException as e:
                if self.debug:
                    print(f"[STM32] 串口接收错误: {e}")
                time.sleep(0.01)
            except Exception as e:
                if self.debug:
                    print(f"[STM32] 接收数据错误: {e}")
                time.sleep(0.01)
    
    def _parse_data(self, ext_type, ext_data):
        """
        解析接收到的数据
        
        Args:
            ext_type: 功能码
            ext_data: 数据列表
        """
        import struct
        from ..core.constants import (
            FUNC_REPORT_SPEED, FUNC_REPORT_MPU_RAW, FUNC_REPORT_ICM_RAW,
            FUNC_REPORT_IMU_ATT, FUNC_REPORT_ENCODER, FUNC_VERSION,
            FUNC_LIMIT_SWITCH, FUNC_UART_SERVO, FUNC_ARM_CTRL,
            FUNC_SET_MOTOR_PID, FUNC_SET_YAW_PID, FUNC_ARM_OFFSET,
            FUNC_AKM_DEF_ANGLE, FUNC_SET_CAR_TYPE
        )
        
        if ext_type == FUNC_REPORT_SPEED:
            # 解析速度和电池数据
            self.vx = int(struct.unpack('h', bytearray(ext_data[0:2]))[0]) / 1000.0
            self.vy = int(struct.unpack('h', bytearray(ext_data[2:4]))[0]) / 1000.0
            self.vz = int(struct.unpack('h', bytearray(ext_data[4:6]))[0]) / 1000.0
            self.battery_voltage = struct.unpack('B', bytearray(ext_data[6:7]))[0]
        
        elif ext_type == FUNC_REPORT_MPU_RAW:
            # 解析 MPU9250 原始数据
            gyro_ratio = 1 / 3754.9
            self.gx = struct.unpack('h', bytearray(ext_data[0:2]))[0] * gyro_ratio
            self.gy = struct.unpack('h', bytearray(ext_data[2:4]))[0] * -gyro_ratio
            self.gz = struct.unpack('h', bytearray(ext_data[4:6]))[0] * -gyro_ratio
            
            accel_ratio = 1 / 1671.84
            self.ax = struct.unpack('h', bytearray(ext_data[6:8]))[0] * accel_ratio
            self.ay = struct.unpack('h', bytearray(ext_data[8:10]))[0] * accel_ratio
            self.az = struct.unpack('h', bytearray(ext_data[10:12]))[0] * accel_ratio
            
            mag_ratio = 1
            self.mx = struct.unpack('h', bytearray(ext_data[12:14]))[0] * mag_ratio
            self.my = struct.unpack('h', bytearray(ext_data[14:16]))[0] * mag_ratio
            self.mz = struct.unpack('h', bytearray(ext_data[16:18]))[0] * mag_ratio
        
        elif ext_type == FUNC_REPORT_ICM_RAW:
            # 解析 ICM20948 原始数据
            gyro_ratio = 1 / 1000.0
            self.gx = struct.unpack('h', bytearray(ext_data[0:2]))[0] * gyro_ratio
            self.gy = struct.unpack('h', bytearray(ext_data[2:4]))[0] * gyro_ratio
            self.gz = struct.unpack('h', bytearray(ext_data[4:6]))[0] * gyro_ratio
            
            accel_ratio = 1 / 1000.0
            self.ax = struct.unpack('h', bytearray(ext_data[6:8]))[0] * accel_ratio
            self.ay = struct.unpack('h', bytearray(ext_data[8:10]))[0] * accel_ratio
            self.az = struct.unpack('h', bytearray(ext_data[10:12]))[0] * accel_ratio
            
            mag_ratio = 1 / 1000.0
            self.mx = struct.unpack('h', bytearray(ext_data[12:14]))[0] * mag_ratio
            self.my = struct.unpack('h', bytearray(ext_data[14:16]))[0] * mag_ratio
            self.mz = struct.unpack('h', bytearray(ext_data[16:18]))[0] * mag_ratio
        
        elif ext_type == FUNC_REPORT_IMU_ATT:
            # 解析姿态角
            self.roll = struct.unpack('h', bytearray(ext_data[0:2]))[0] / 10000.0
            self.pitch = struct.unpack('h', bytearray(ext_data[2:4]))[0] / 10000.0
            self.yaw = struct.unpack('h', bytearray(ext_data[4:6]))[0] / 10000.0
        
        elif ext_type == FUNC_REPORT_ENCODER:
            # 解析编码器数据
            self.encoder_m1 = struct.unpack('i', bytearray(ext_data[0:4]))[0]
            self.encoder_m2 = struct.unpack('i', bytearray(ext_data[4:8]))[0]
            self.encoder_m3 = struct.unpack('i', bytearray(ext_data[8:12]))[0]
            self.encoder_m4 = struct.unpack('i', bytearray(ext_data[12:16]))[0]
        
        elif ext_type == FUNC_VERSION:
            # 解析版本号
            self.version_H = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.version_L = struct.unpack('B', bytearray(ext_data[1:2]))[0]
            if self.debug:
                print(f"[STM32] 版本号: {self.version_H}.{self.version_L}")
        
        elif ext_type == FUNC_LIMIT_SWITCH:
            # 解析限位开关状态
            old_state = self.limit_switch_state
            self.limit_switch_state = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            if self.debug:
                print(f"[STM32] 限位开关状态: {self.limit_switch_state}")
            
            # 检测限位开关状态变化并触发回调
            if old_state != self.limit_switch_state and self.limit_switch_callback:
                self.limit_switch_callback(old_state, self.limit_switch_state)
    
    def send_command(self, func, data):
        """
        发送命令到 STM32
        
        Args:
            func: 功能码
            data: 数据字节
        
        Returns:
            是否成功
        """
        if not self.connected:
            return False
        
        try:
            # 构建帧
            length = len(data)
            check_sum = (HEAD + DEVICE_ID + length + func + sum(data) + 1) & 0xFF
            
            frame = bytes([HEAD, DEVICE_ID, length, func]) + bytes(data) + bytes([check_sum])
            
            # 发送
            self.ser.write(frame)
            self.ser.flush()
            
            # 延迟
            time.sleep(self.delay)
            
            return True
        except Exception as e:
            if self.debug:
                print(f"[STM32] 发送命令错误: {e}")
            return False
    
    def get_motion_data(self):
        """获取运动数据"""
        return self.vx, self.vy, self.vz
    
    def get_imu_attitude(self):
        """获取IMU姿态"""
        return self.roll, self.pitch, self.yaw
    
    def get_encoder_data(self):
        """获取编码器数据"""
        return self.encoder_m1, self.encoder_m2, self.encoder_m3, self.encoder_m4
    
    def get_battery_voltage(self):
        """获取电池电压"""
        return self.battery_voltage
    
    def get_limit_switch_state(self):
        """获取限位开关状态"""
        return self.limit_switch_state
    
    def set_limit_switch_callback(self, callback):
        """
        设置限位开关状态变化回调函数
        
        Args:
            callback: 回调函数，接收参数 (old_state, new_state)
        """
        self.limit_switch_callback = callback
    
    def __del__(self):
        """析构函数，自动断开连接"""
        self.disconnect()