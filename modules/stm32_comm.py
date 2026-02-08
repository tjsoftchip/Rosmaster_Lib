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
        
        # ========== 舵机数据缓存 ==========
        self.uart_servo_id = 0          # 总线舵机ID
        self.uart_servo_value = 0       # 总线舵机脉冲值
        self.arm_offset_id = 0          # 机械臂中位偏差ID
        self.arm_offset_state = 0       # 机械臂中位偏差状态
        self.arm_ctrl_enable = True     # 机械臂控制开关
        self.arm_angles = [-1, -1, -1, -1, -1, -1]  # 机械臂角度数组
        
        # ========== PID数据缓存 ==========
        self.pid_index = 0              # PID索引
        self.kp = 0                     # 比例系数
        self.ki = 0                     # 积分系数
        self.kd = 0                     # 微分系数
        
        # ========== 阿克曼数据缓存 ==========
        self.akm_def_angle = 100        # 阿克曼默认角度
        self.akm_readed_angle = False   # 是否已读取阿克曼角度
        
        # ========== 车型数据缓存 ==========
        self.read_car_type = 0          # 从机器读取的车型
        
        # 接收线程
        self.receive_thread = None
        self.uart_state = 0
        
        # 限位开关状态变化回调
        self.limit_switch_callback = None
        
        # 串口访问锁
        self._lock = threading.Lock()
        
        # 连接串口
        self._connect()
    
    def _connect(self):
        """连接串口（检查是否已被占用）"""
        # 检查串口是否已经被打开
        if self.ser and self.ser.is_open:
            if self.debug:
                print(f"[STM32] 串口 {self.port} 已经打开，复用现有连接")
            self.connected = True
            return

        try:
            # 尝试打开串口
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                exclusive=False  # 允许其他进程访问（同一进程内共享）
            )
            self.connected = True

            if self.debug:
                print(f"[STM32] 串口连接成功: {self.port}")
                print(f"[STM32] 波特率: {self.baudrate}")
        except serial.SerialException as e:
            # 如果串口已被占用，尝试获取现有的文件描述符
            if "Permission denied" in str(e) or "could not open port" in str(e):
                if self.debug:
                    print(f"[STM32] 串口 {self.port} 已被占用，尝试查找现有连接...")
                self.connected = False
            else:
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
            # 清空串口缓冲区，防止残留数据导致帧错位
            if self.connected:
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    if self.debug:
                        print("[STM32] 串口缓冲区已清空")
                except Exception as e:
                    if self.debug:
                        print(f"[STM32] 清空缓冲区失败: {e}")
            
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
        
        consecutive_errors = 0
        max_consecutive_errors = 10
        
        while True:
            try:
                with self._lock:  # 保护串口访问
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
                                consecutive_errors = 0  # 重置错误计数器
                            else:
                                consecutive_errors += 1
                                if self.debug and consecutive_errors <= 5:  # 只打印前5次错误
                                    print(f"[STM32] 校验和错误: type={ext_type}, len={ext_len}, calc={check_sum%256}, recv={rx_check_num}, data={ext_data[:5] if ext_data else []}")
                                
                                # 如果连续错误过多，清空缓冲区
                                if consecutive_errors >= max_consecutive_errors:
                                    if self.debug:
                                        print(f"[STM32] 连续错误过多，清空缓冲区")
                                    self.ser.reset_input_buffer()
                                    consecutive_errors = 0
                        else:
                            # 帧头不匹配，可能是数据错位，清空缓冲区
                            if self.debug:
                                print(f"[STM32] 帧头不匹配: head2={head2}, expected={DEVICE_ID-1}")
                            self.ser.reset_input_buffer()
                            consecutive_errors = 0
                    else:
                        # 帧头不匹配，跳过
                        consecutive_errors += 1
                        if consecutive_errors >= max_consecutive_errors:
                            if self.debug:
                                print(f"[STM32] 连续帧头错误，清空缓冲区")
                            self.ser.reset_input_buffer()
                            consecutive_errors = 0
                    
            except serial.SerialException as e:
                if self.debug:
                    print(f"[STM32] 串口接收错误: {e}")
                consecutive_errors += 1
                time.sleep(0.01)
            except Exception as e:
                if self.debug:
                    print(f"[STM32] 接收数据错误: {e}")
                consecutive_errors += 1
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
            # 解析速度、电池电压和限位开关状态
            # 新协议格式（长度0x0B）：
            # [0-1] X轴速度（±1000）
            # [2-3] Y轴速度（±1000）
            # [4-5] Z轴速度（±5000）
            # [6]   电池电压（10倍）
            # [7]   限位开关状态
            self.vx = int(struct.unpack('h', bytearray(ext_data[0:2]))[0]) / 1000.0
            self.vy = int(struct.unpack('h', bytearray(ext_data[2:4]))[0]) / 1000.0
            self.vz = int(struct.unpack('h', bytearray(ext_data[4:6]))[0]) / 1000.0
            self.battery_voltage = struct.unpack('B', bytearray(ext_data[6:7]))[0]
            
            # 解析限位开关状态（新增）
            if len(ext_data) >= 8:
                old_state = self.limit_switch_state
                self.limit_switch_state = struct.unpack('B', bytearray(ext_data[7:8]))[0]
                
                # 检测限位开关状态变化并触发回调
                if old_state != self.limit_switch_state and self.limit_switch_callback:
                    self.limit_switch_callback(old_state, self.limit_switch_state)
                    
                if self.debug:
                    print(f"[STM32] 速度上报包限位状态: {self.limit_switch_state}")
        
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
            # 响应格式: FF FB 05 33 xx 00 checksum
            # [0] HEAD, [1] DEVICE_ID-1, [2] LENGTH, [3] FUNC, [4] 状态, [5] 保留, [6] 校验和
            old_state = self.limit_switch_state
            self.limit_switch_state = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            # ext_data[1] 是保留字节，暂时忽略
            if self.debug:
                print(f"[STM32] 限位开关状态: {self.limit_switch_state}")

            # 检测限位开关状态变化并触发回调
            if old_state != self.limit_switch_state and self.limit_switch_callback:
                self.limit_switch_callback(old_state, self.limit_switch_state)
        
        elif ext_type == FUNC_UART_SERVO:
            """解析总线舵机响应"""
            self.uart_servo_id = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.uart_servo_value = struct.unpack('h', bytearray(ext_data[1:3]))[0]
            if self.debug:
                print(f"[STM32] UART Servo: id={self.uart_servo_id}, value={self.uart_servo_value}")
        
        elif ext_type == FUNC_ARM_CTRL:
            """解析机械臂控制响应"""
            self.arm_angles[0] = struct.unpack('h', bytearray(ext_data[0:2]))[0]
            self.arm_angles[1] = struct.unpack('h', bytearray(ext_data[2:4]))[0]
            self.arm_angles[2] = struct.unpack('h', bytearray(ext_data[4:6]))[0]
            self.arm_angles[3] = struct.unpack('h', bytearray(ext_data[6:8]))[0]
            self.arm_angles[4] = struct.unpack('h', bytearray(ext_data[8:10]))[0]
            self.arm_angles[5] = struct.unpack('h', bytearray(ext_data[10:12]))[0]
            if self.debug:
                print(f"[STM32] Arm Ctrl: angles={self.arm_angles}")
        
        elif ext_type == FUNC_SET_MOTOR_PID:
            """解析电机PID响应"""
            self.pid_index = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.kp = struct.unpack('h', bytearray(ext_data[1:3]))[0] / 1000.0
            self.ki = struct.unpack('h', bytearray(ext_data[3:5]))[0] / 1000.0
            self.kd = struct.unpack('h', bytearray(ext_data[5:7]))[0] / 1000.0
            if self.debug:
                print(f"[STM32] Motor PID: index={self.pid_index}, kp={self.kp}, ki={self.ki}, kd={self.kd}")
        
        elif ext_type == FUNC_SET_YAW_PID:
            """解析偏航PID响应"""
            self.pid_index = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.kp = struct.unpack('h', bytearray(ext_data[1:3]))[0] / 1000.0
            self.ki = struct.unpack('h', bytearray(ext_data[3:5]))[0] / 1000.0
            self.kd = struct.unpack('h', bytearray(ext_data[5:7]))[0] / 1000.0
            if self.debug:
                print(f"[STM32] Yaw PID: index={self.pid_index}, kp={self.kp}, ki={self.ki}, kd={self.kd}")
        
        elif ext_type == FUNC_ARM_OFFSET:
            """解析机械臂中位偏差响应"""
            self.arm_offset_id = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.arm_offset_state = struct.unpack('B', bytearray(ext_data[1:2]))[0]
            if self.debug:
                print(f"[STM32] Arm Offset: id={self.arm_offset_id}, state={self.arm_offset_state}")
        
        elif ext_type == FUNC_AKM_DEF_ANGLE:
            """解析阿克曼默认角度响应"""
            id = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.akm_def_angle = struct.unpack('B', bytearray(ext_data[1:2]))[0]
            self.akm_readed_angle = True
            if self.debug:
                print(f"[STM32] AKM Default Angle: id={id}, angle={self.akm_def_angle}")
        
        elif ext_type == FUNC_SET_CAR_TYPE:
            """解析车型设置响应"""
            car_type = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.read_car_type = car_type
            if self.debug:
                print(f"[STM32] Car Type: {car_type}")
    
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
            with self._lock:  # 保护串口访问
                # 构建帧（按照原版代码的计算方式）
                # 注意：length 字段是固定的，不是 len(data)
                length = 0x05  # 固定长度
                cmd = [HEAD, DEVICE_ID, length, func] + list(data)
                check_sum = sum(cmd, COMPLEMENT) & 0xFF
                cmd.append(check_sum)

                frame = bytes(cmd)

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
    
    # ========== 舵机控制方法 ==========
    
    def set_pwm_servo(self, servo_id, angle):
        """设置PWM舵机角度
        
        Args:
            servo_id: 舵机ID (1-4)
            angle: 角度 (0-180)
        
        Returns:
            是否成功
        """
        from ..core.constants import FUNC_PWM_SERVO
        
        if servo_id < 1 or servo_id > 4:
            if self.debug:
                print(f"[STM32] PWM Servo ID error: {servo_id}")
            return False
        
        angle = max(0, min(180, int(angle)))
        data = bytes([servo_id, angle, 0, 0])
        return self.send_command(FUNC_PWM_SERVO, data)
    
    def set_pwm_servo_all(self, angle_s1, angle_s2, angle_s3, angle_s4):
        """同时控制四路PWM舵机
        
        Args:
            angle_s1: 舵机1角度 (0-180)
            angle_s2: 舵机2角度 (0-180)
            angle_s3: 舵机3角度 (0-180)
            angle_s4: 舵机4角度 (0-180)
        
        Returns:
            是否成功
        """
        from ..core.constants import FUNC_PWM_SERVO_ALL
        
        # 角度范围检查，超出范围设为255（不控制）
        a1 = 255 if angle_s1 < 0 or angle_s1 > 180 else int(angle_s1)
        a2 = 255 if angle_s2 < 0 or angle_s2 > 180 else int(angle_s2)
        a3 = 255 if angle_s3 < 0 or angle_s3 > 180 else int(angle_s3)
        a4 = 255 if angle_s4 < 0 or angle_s4 > 180 else int(angle_s4)
        
        data = bytes([a1, a2, a3, a4])
        return self.send_command(FUNC_PWM_SERVO_ALL, data)
    
    def set_uart_servo(self, servo_id, pulse_value, run_time=500):
        """控制总线舵机
        
        Args:
            servo_id: 舵机ID (1-255)
            pulse_value: 脉冲值 (96-4000)
            run_time: 运行时间ms (0-2000)
        
        Returns:
            是否成功
        """
        from ..core.constants import FUNC_UART_SERVO
        
        if not self.arm_ctrl_enable:
            if self.debug:
                print("[STM32] Arm control disabled")
            return False
        
        if servo_id < 1 or pulse_value < 96 or pulse_value > 4000 or run_time < 0:
            if self.debug:
                print(f"[STM32] UART Servo input error: id={servo_id}, pulse={pulse_value}, time={run_time}")
            return False
        
        run_time = max(0, min(2000, int(run_time)))
        
        data = bytes([
            servo_id,
            (pulse_value >> 8) & 0xFF,
            pulse_value & 0xFF,
            (run_time >> 8) & 0xFF,
            run_time & 0xFF,
            0, 0
        ])
        
        return self.send_command(FUNC_UART_SERVO, data)
    
    def set_uart_servo_torque(self, enable):
        """设置总线舵机扭矩
        
        Args:
            enable: True=开启扭矩, False=关闭扭矩
        
        Returns:
            是否成功
        """
        from ..core.constants import FUNC_UART_SERVO_TORQUE
        
        on = 1 if enable else 0
        data = bytes([on, 0])
        return self.send_command(FUNC_UART_SERVO_TORQUE, data)
    
    def set_uart_servo_id(self, servo_id):
        """设置总线舵机ID（谨慎使用）
        
        Args:
            servo_id: 舵机ID (1-250)
        
        Returns:
            是否成功
        """
        from ..core.constants import FUNC_UART_SERVO_ID
        
        if servo_id < 1 or servo_id > 250:
            if self.debug:
                print(f"[STM32] Servo ID error: {servo_id}")
            return False
        
        data = bytes([servo_id])
        return self.send_command(FUNC_UART_SERVO_ID, data)
    
    def set_arm_ctrl(self, angle_array, run_time=500):
        """控制机械臂所有关节
        
        Args:
            angle_array: 角度数组 [s1, s2, s3, s4, s5, s6]
                         s1-s4: 0-180度
                         s5: 0-270度
                         s6: 0-180度
            run_time: 运行时间ms (0-2000)
        
        Returns:
            是否成功
        """
        from ..core.constants import FUNC_ARM_CTRL
        
        if not self.arm_ctrl_enable:
            if self.debug:
                print("[STM32] Arm control disabled")
            return False
        
        # 角度范围检查
        if not (0 <= angle_array[0] <= 180 and 0 <= angle_array[1] <= 180 and 
                0 <= angle_array[2] <= 180 and 0 <= angle_array[3] <= 180 and
                0 <= angle_array[4] <= 270 and 0 <= angle_array[5] <= 180):
            if self.debug:
                print(f"[STM32] Arm angles error: {angle_array}")
            return False
        
        run_time = max(0, min(2000, int(run_time)))
        
        # 角度转脉冲值
        pulse_values = []
        for i in range(6):
            pulse_values.append(self._arm_convert_value(i+1, angle_array[i]))
        
        # 构建数据
        data = bytes()
        for pulse in pulse_values:
            data += bytes([(pulse >> 8) & 0xFF, pulse & 0xFF])
        data += bytes([(run_time >> 8) & 0xFF, run_time & 0xFF])
        
        return self.send_command(FUNC_ARM_CTRL, data)
    
    def set_arm_offset(self, servo_id):
        """设置机械臂中位偏差
        
        Args:
            servo_id: 舵机ID (0-6), 0=全部恢复默认
        
        Returns:
            中位偏差状态
        """
        from ..core.constants import FUNC_ARM_OFFSET
        
        self.arm_offset_id = 0xFF
        self.arm_offset_state = 0
        
        s_id = int(servo_id) & 0xFF
        data = bytes([s_id])
        
        if not self.send_command(FUNC_ARM_OFFSET, data):
            return self.arm_offset_state
        
        # 等待响应
        for i in range(200):
            if self.arm_offset_id == servo_id:
                if self.debug:
                    if self.arm_offset_id == 0:
                        print("[STM32] Arm Reset Offset Value")
                    else:
                        print(f"[STM32] Arm Offset State: id={self.arm_offset_id}, state={self.arm_offset_state}, i={i}")
                return self.arm_offset_state
            time.sleep(0.001)
        
        return self.arm_offset_state
    
    # ========== PID控制方法 ==========
    
    def set_motor_pid(self, kp, ki, kd, forever=False):
        """设置电机PID参数
        
        Args:
            kp: 比例系数 (0-10)
            ki: 积分系数 (0-10)
            kd: 微分系数 (0-10)
            forever: 是否永久保存
        
        Returns:
            是否成功
        """
        from ..core.constants import FUNC_SET_MOTOR_PID
        
        if kp > 10 or ki > 10 or kd > 10 or kp < 0 or ki < 0 or kd < 0:
            if self.debug:
                print(f"[STM32] PID value error: kp={kp}, ki={ki}, kd={kd}")
            return False
        
        state = 0x5F if forever else 0
        
        kp_params = int(kp * 1000)
        ki_params = int(ki * 1000)
        kd_params = int(kd * 1000)
        
        data = bytes([
            (kp_params >> 8) & 0xFF, kp_params & 0xFF,
            (ki_params >> 8) & 0xFF, ki_params & 0xFF,
            (kd_params >> 8) & 0xFF, kd_params & 0xFF,
            state
        ])
        
        success = self.send_command(FUNC_SET_MOTOR_PID, data)
        if forever:
            time.sleep(0.1)
        
        return success
    
    def set_yaw_pid(self, kp, ki, kd, forever=False):
        """设置偏航PID参数
        
        Args:
            kp: 比例系数 (0-10)
            ki: 积分系数 (0-10)
            kd: 微分系数 (0-10)
            forever: 是否永久保存
        
        Returns:
            是否成功
        """
        from ..core.constants import FUNC_SET_YAW_PID
        
        if kp > 10 or ki > 10 or kd > 10 or kp < 0 or ki < 0 or kd < 0:
            if self.debug:
                print(f"[STM32] Yaw PID value error: kp={kp}, ki={ki}, kd={kd}")
            return False
        
        state = 0x5F if forever else 0
        
        kp_params = int(kp * 1000)
        ki_params = int(ki * 1000)
        kd_params = int(kd * 1000)
        
        data = bytes([
            (kp_params >> 8) & 0xFF, kp_params & 0xFF,
            (ki_params >> 8) & 0xFF, ki_params & 0xFF,
            (kd_params >> 8) & 0xFF, kd_params & 0xFF,
            state
        ])
        
        success = self.send_command(FUNC_SET_YAW_PID, data)
        if forever:
            time.sleep(0.1)
        
        return success
    
    # ========== RGB灯带控制方法 ==========
    
    def set_colorful_lamps(self, led_id, red, green, blue):
        """设置RGB灯带颜色
        
        Args:
            led_id: LED ID (0-13), 0xFF=全部
            red: 红色值 (0-255)
            green: 绿色值 (0-255)
            blue: 蓝色值 (0-255)
        
        Returns:
            是否成功
        """
        from ..core.constants import FUNC_RGB
        
        data = bytes([
            int(led_id) & 0xFF,
            int(red) & 0xFF,
            int(green) & 0xFF,
            int(blue) & 0xFF
        ])
        return self.send_command(FUNC_RGB, data)
    
    # ========== 系统控制方法 ==========
    
    def set_car_type(self, car_type):
        """设置车型
        
        Args:
            car_type: 车型 (0x01=X3, 0x02=X3_PLUS, 0x04=X1, 0x05=R2)
        
        Returns:
            是否成功
        """
        from ..core.constants import FUNC_SET_CAR_TYPE
        
        data = bytes([car_type & 0xFF, 0x5F])
        return self.send_command(FUNC_SET_CAR_TYPE, data)
    
    def reset_flash_value(self):
        """重置Flash数据
        
        Returns:
            是否成功
        """
        from ..core.constants import FUNC_RESET_FLASH
        
        data = bytes([0x5F])
        success = self.send_command(FUNC_RESET_FLASH, data)
        time.sleep(0.1)
        return success
    
    def reset_car_state(self):
        """重置小车状态
        
        Returns:
            是否成功
        """
        from ..core.constants import FUNC_RESET_STATE
        
        data = bytes([0x5F])
        return self.send_command(FUNC_RESET_STATE, data)
    
    # ========== 阿克曼控制方法 ==========
    
    def set_akm_default_angle(self, angle, forever=False):
        """设置阿克曼默认角度
        
        Args:
            angle: 默认角度 (60-120)
            forever: 是否永久保存
        
        Returns:
            是否成功
        """
        from ..core.constants import FUNC_AKM_DEF_ANGLE
        
        if int(angle) > 120 or int(angle) < 60:
            if self.debug:
                print(f"[STM32] AKM default angle error: {angle}")
            return False
        
        id = self.AKM_SERVO_ID
        state = 0x5F if forever else 0
        
        if forever:
            self.akm_def_angle = angle
        
        data = bytes([id, int(angle), state])
        success = self.send_command(FUNC_AKM_DEF_ANGLE, data)
        if forever:
            time.sleep(0.1)
        
        return success
    
    def set_akm_steering_angle(self, angle, ctrl_car=False):
        """控制阿克曼转向角度

        Args:
            angle: 转向角度 (-45~45)
            ctrl_car: 是否同时控制电机速度

        Returns:
            是否成功
        """
        from ..core.constants import FUNC_AKM_STEER_ANGLE

        if int(angle) > 45 or int(angle) < -45:
            if self.debug:
                print(f"[STM32] AKM steering angle error: {angle}")
            return False

        id = self.AKM_SERVO_ID
        if ctrl_car:
            id = self.AKM_SERVO_ID + 0x80

        data = bytes([id, int(angle) & 0xFF])
        return self.send_command(FUNC_AKM_STEER_ANGLE, data)
    
    # ========== 数据获取方法 ==========
    
    def request_data(self, function, param=0):
        """主动请求数据
        
        Args:
            function: 功能码
            param: 参数
        
        Returns:
            是否成功
        """
        from ..core.constants import FUNC_REQUEST_DATA
        
        data = bytes([function & 0xFF, param & 0xFF])
        return self.send_command(FUNC_REQUEST_DATA, data)
    
    def get_uart_servo_value(self):
        """获取总线舵机脉冲值
        
        Returns:
            (servo_id, pulse_value)
        """
        return self.uart_servo_id, self.uart_servo_value
    
    def get_uart_servo_angle(self, servo_id):
        """获取总线舵机角度
        
        Args:
            servo_id: 舵机ID (1-6)
        
        Returns:
            角度值
        """
        if 1 <= servo_id <= 6:
            return self._arm_convert_angle(servo_id, self.arm_angles[servo_id-1])
        return -1
    
    def get_uart_servo_angle_array(self):
        """获取机械臂角度数组
        
        Returns:
            角度数组 [s1, s2, s3, s4, s5, s6]
        """
        angles = []
        for i in range(6):
            angles.append(self._arm_convert_angle(i+1, self.arm_angles[i]))
        return angles
    
    def get_motion_pid(self, motor_id):
        """获取电机PID参数
        
        Args:
            motor_id: 电机ID (1-4)
        
        Returns:
            (kp, ki, kd)
        """
        from ..core.constants import FUNC_SET_MOTOR_PID
        
        # 发送请求
        self.request_data(FUNC_SET_MOTOR_PID, motor_id)
        time.sleep(0.05)
        return self.kp, self.ki, self.kd
    
    def get_yaw_pid(self):
        """获取偏航PID参数
        
        Returns:
            (kp, ki, kd)
        """
        from ..core.constants import FUNC_SET_YAW_PID
        
        # 发送请求
        self.request_data(FUNC_SET_YAW_PID, 0)
        time.sleep(0.05)
        return self.kp, self.ki, self.kd
    
    def get_akm_default_angle(self):
        """获取阿克曼默认角度
        
        Returns:
            默认角度
        """
        from ..core.constants import FUNC_AKM_DEF_ANGLE
        
        if not self.akm_readed_angle:
            self.request_data(FUNC_AKM_DEF_ANGLE, self.AKM_SERVO_ID)
            time.sleep(0.05)
        return self.akm_def_angle
    
    def get_car_type_from_machine(self):
        """从机器获取车型
        
        Returns:
            车型
        """
        from ..core.constants import FUNC_SET_CAR_TYPE
        
        self.request_data(FUNC_SET_CAR_TYPE, 0)
        time.sleep(0.05)
        return self.read_car_type
    
    # ========== 内部工具方法 ==========
    
    def _arm_convert_value(self, s_id, s_angle):
        """将角度转换为脉冲值（写入角度）
        
        Args:
            s_id: 舵机ID (1-6)
            s_angle: 角度
        
        Returns:
            脉冲值
        """
        value = -1
        if s_id == 1:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 2:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 3:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 4:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 5:
            value = int((3700 - 380) * (s_angle - 0) / (270 - 0) + 380)
        elif s_id == 6:
            value = int((3100 - 900) * (s_angle - 0) / (180 - 0) + 900)
        return value
    
    def _arm_convert_angle(self, s_id, s_value):
        """将脉冲值转换为角度（读取角度）
        
        Args:
            s_id: 舵机ID (1-6)
            s_value: 脉冲值
        
        Returns:
            角度
        """
        s_angle = -1
        if s_id == 1:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 2:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 3:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 4:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 5:
            s_angle = int((270 - 0) * (s_value - 380) / (3700 - 380) + 0 + 0.5)
        elif s_id == 6:
            s_angle = int((180 - 0) * (s_value - 900) / (3100 - 900) + 0 + 0.5)
        return s_angle
    
    def __del__(self):
        """析构函数，自动断开连接"""
        self.disconnect()