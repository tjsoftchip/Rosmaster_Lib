#!/usr/bin/env python3
# coding: utf-8

"""
MSSD 无刷电机驱动器控制模块

支持 MSSD-15LMB, MSSD-20LMA, MSSD-30LMA, MSSD-40LMA 等型号
使用 Modbus RTU 协议通信

作者：nx-ros2
日期：2026-01-16
"""

import serial
import time
import math
import threading
from ..utils.crc16 import calculate_crc16
from ..core.constants import (
    MSSD_DEVICE_ID,
    MSSD_GEAR_RATIO,
    TIRE_DIAMETER_R2,
    WHEELBASE_R2,
    TRACK_WIDTH_R2,
    DEFAULT_BAUDRATE,
)


class MSSDController:
    """
    MSSD 无刷电机驱动器控制器

    功能：
    - 读取电机速度、电流、温度等状态
    - 设置电机速度
    - 读取编码器数据
    - 错误检测和报告
    """

    def __init__(
        self,
        port="/dev/mssd",
        device_id=MSSD_DEVICE_ID,
        baudrate=DEFAULT_BAUDRATE,
        debug=False,
    ):
        """
        初始化 MSSD 控制器

        Args:
            port: 串口设备路径（默认 /dev/mssd）
            device_id: MSSD 设备地址（默认 1）
            baudrate: 波特率（默认 115200）
            debug: 调试模式
        """
        self.port = port
        self.device_id = device_id
        self.baudrate = baudrate
        self.debug = debug
        self.connected = False
        self.ser = None

        # 串口访问锁（与转向角度传感器共享）
        self._lock = threading.Lock()

        # 速度缓存
        self.speed = 0  # 电机速度（RPM）
        self.linear_velocity = 0  # 线速度（m/s）
        self.cached_motor_speed = 0  # 缓存的目标速度（-100~100）
        self.last_update_time = 0  # 上次更新时间

        # 编码器里程相关
        self.encoder_pulses_per_rev = 24  # 编码器每转脉冲数
        self.last_encoder_value = None  # 上次编码器值
        self.encoder_distance = 0.0  # 编码器累计里程（米）
        self.encoder_distance_delta = 0.0  # 编码器本次里程增量（米）

        # 物理参数
        self.wheelbase = WHEELBASE_R2  # 轴距
        self.track_width = TRACK_WIDTH_R2  # 轮距
        self.gear_ratio = MSSD_GEAR_RATIO  # 减速比
        self.tire_diameter = TIRE_DIAMETER_R2  # 轮胎直径

        # 连接串口
        self._connect()

    def _connect(self):
        """连接串口（检查是否已被占用）"""
        # 检查串口是否已经被打开
        if self.ser and self.ser.is_open:
            if self.debug:
                print(f"[MSSD] 串口 {self.port} 已经打开，复用现有连接")
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
                exclusive=False,  # 允许其他进程访问（同一进程内共享）
            )
            self.connected = True
            print(f"[MSSD] ✓ Connected to {self.port} (device_id={self.device_id})")
        except serial.SerialException as e:
            # 如果串口已被占用，尝试获取现有的文件描述符
            if "Permission denied" in str(e) or "could not open port" in str(e):
                if self.debug:
                    print(f"[MSSD] 串口 {self.port} 已被占用，尝试查找现有连接...")
                self.connected = False
            else:
                self.connected = False
                print(f"[MSSD] ✗ Connection failed: {e}")

        # MSSD 相关变量
        self.last_command_time = 0  # 上次命令时间
        self.speed_change_threshold = 0.1  # 速度变化阈值
        self.last_steer_angle = 0  # 上次转向角度

    def disconnect(self):
        """断开串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.connected = False
            if self.debug:
                print("[MSSD] 串口已断开")

    def _read_registers(self, start_addr, count):
        """
        读取寄存器（功能码 0x03）

        Args:
            start_addr: 起始地址
            count: 读取数量

        Returns:
            寄存器值列表，失败返回 None
        """
        if not self.connected:
            return None

        # 使用锁保护串口访问
        with self._lock:
            try:
                # 构建请求帧
                data = bytes(
                    [
                        (start_addr >> 8) & 0xFF,
                        start_addr & 0xFF,
                        (count >> 8) & 0xFF,
                        count & 0xFF,
                    ]
                )

                frame = bytes([self.device_id, 0x03]) + data
                crc = calculate_crc16(frame)
                frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF])

                # 发送请求（不清空缓冲区，避免干扰其他操作）
                self.ser.write(frame)
                self.ser.flush()

                # 读取响应
                time.sleep(0.01)
                response = self.ser.read(5 + 2 * count)

                if len(response) < 5:
                    return None

                # 验证设备 ID
                if response[0] != self.device_id:
                    if self.debug:
                        print(
                            f"[MSSD] 设备 ID 不匹配: 期望 {self.device_id}, 实际 {response[0]}"
                        )
                    return None

                # 验证功能码
                if response[1] != 0x03:
                    if self.debug:
                        print(f"[MSSD] 功能码不匹配: 期望 0x03, 实际 {response[1]}")
                    return None

                # 验证 CRC
                received_crc = (response[-1] << 8) | response[-2]
                calculated_crc = calculate_crc16(response[:-2])

                if received_crc != calculated_crc:
                    if self.debug:
                        print("[MSSD] CRC 校验错误")
                    return None

                # 解析数据
                byte_count = response[2]
                values = []
                for i in range(byte_count // 2):
                    high = response[3 + 2 * i]
                    low = response[4 + 2 * i]
                    values.append((high << 8) | low)

                return values
            except Exception as e:
                if self.debug:
                    print(f"[MSSD] 读取寄存器错误: {e}")
                return None

    def _write_register(self, addr, value):
        """
        写寄存器（功能码 0x06）

        Args:
            addr: 寄存器地址
            value: 写入值

        Returns:
            是否成功
        """
        if not self.connected:
            return False

        # 使用锁保护串口访问
        with self._lock:
            try:
                # 构建请求帧
                data = bytes(
                    [(addr >> 8) & 0xFF, addr & 0xFF, (value >> 8) & 0xFF, value & 0xFF]
                )

                frame = bytes([self.device_id, 0x06]) + data
                crc = calculate_crc16(frame)
                frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF])

                # 发送请求（不清空缓冲区，避免干扰其他操作）
                self.ser.write(frame)
                self.ser.flush()

                # 读取响应
                time.sleep(0.01)
                response = self.ser.read(8)

                if len(response) != 8:
                    return False

                # 验证设备 ID
                if response[0] != self.device_id:
                    if self.debug:
                        print(
                            f"[MSSD] 设备 ID 不匹配: 期望 {self.device_id}, 实际 {response[0]}"
                        )
                    return False

                # 验证功能码
                if response[1] != 0x06:
                    if self.debug:
                        print(f"[MSSD] 功能码不匹配: 期望 0x06, 实际 {response[1]}")
                    return False

                # 验证 CRC
                received_crc = (response[-1] << 8) | response[-2]
                calculated_crc = calculate_crc16(response[:-2])

                return received_crc == calculated_crc
            except Exception as e:
                if self.debug:
                    print(f"[MSSD] 写寄存器错误: {e}")
                return False

    def _write_registers(self, start_addr, values):
        """
        写多个寄存器（功能码 0x10）

        Args:
            start_addr: 起始地址
            values: 写入值列表

        Returns:
            是否成功
        """
        if not self.connected:
            return False

        # 使用锁保护串口访问
        with self._lock:
            try:
                count = len(values)
                byte_count = count * 2

                # 构建数据
                data = bytes(
                    [
                        (start_addr >> 8) & 0xFF,
                        start_addr & 0xFF,
                        (count >> 8) & 0xFF,
                        count & 0xFF,
                        byte_count,
                    ]
                )

                for value in values:
                    data += bytes([(value >> 8) & 0xFF, value & 0xFF])

                # 构建帧
                frame = bytes([self.device_id, 0x10]) + data
                crc = calculate_crc16(frame)
                frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF])

                # 发送请求（不清空缓冲区，避免干扰其他操作）
                self.ser.write(frame)
                self.ser.flush()

                # 读取响应
                time.sleep(0.01)
                response = self.ser.read(8)

                if len(response) != 8:
                    return False

                # 验证设备 ID
                if response[0] != self.device_id:
                    if self.debug:
                        print(
                            f"[MSSD] 设备 ID 不匹配: 期望 {self.device_id}, 实际 {response[0]}"
                        )
                    return False

                # 验证功能码
                if response[1] != 0x10:
                    if self.debug:
                        print(f"[MSSD] 功能码不匹配: 期望 0x10, 实际 {response[1]}")
                    return False

                # 验证 CRC
                received_crc = (response[-1] << 8) | response[-2]
                calculated_crc = calculate_crc16(response[:-2])

                return received_crc == calculated_crc
            except Exception as e:
                if self.debug:
                    print(f"[MSSD] 写多个寄存器错误: {e}")
                return False

    def set_speed(self, speed_rpm):
        """
        设置电机速度

        Args:
            speed_rpm: 目标转速（RPM），支持正负值

        Returns:
            是否成功
        """
        if not self.connected:
            return False

        try:
            # 如果速度不为零，先启动电机
            if abs(speed_rpm) > 0:
                self.start()
            else:
                # 速度为零，停止电机
                self.stop()
                return True

            # 写入速度寄存器（33-34）
            speed_high = (speed_rpm >> 16) & 0xFFFF
            speed_low = speed_rpm & 0xFFFF

            success = self._write_registers(33, [speed_high, speed_low])

            if success:
                self.cached_motor_speed = speed_rpm
                if self.debug:
                    print(f"[MSSD] 设置速度: {speed_rpm} RPM")

            return success
        except Exception as e:
            if self.debug:
                print(f"[MSSD] 设置速度错误: {e}")
            return False

    def stop(self, decelerate=False):
        """
        停止电机

        Args:
            decelerate: 是否减速停止（默认 False，立即停止）

        Returns:
            是否成功
        """
        if not self.connected:
            return False

        try:
            if decelerate:
                # 减速停止：先设置速度为 0，再停止
                success = self._write_registers(33, [0, 0])
                if success:
                    time.sleep(0.2)  # 等待减速
                    success = self._write_register(32, 0)  # 正常停止
            else:
                # 立即停止
                success = self._write_register(32, 0)  # 0 = 正常停止

            if success:
                self.cached_motor_speed = 0
                if self.debug:
                    print(f"[MSSD] 电机已停止 ({'减速' if decelerate else '立即'})")

            return success
        except Exception as e:
            if self.debug:
                print(f"[MSSD] 停止电机错误: {e}")
            return False

    def start(self):
        """
        启动电机

        Returns:
            是否成功
        """
        if not self.connected:
            return False

        try:
            # 写入启动命令（寄存器 32）
            success = self._write_register(32, 1)  # 1 = 启动

            if success:
                if self.debug:
                    print("[MSSD] 电机已启动")

            return success
        except Exception as e:
            if self.debug:
                print(f"[MSSD] 启动电机错误: {e}")
            return False

    def get_speed(self):
        """
        获取电机实际速度

        Returns:
            实际速度（RPM），失败返回 0
        """
        if not self.connected:
            return 0

        try:
            # 读取速度寄存器（9-10）
            values = self._read_registers(9, 2)
            if values and len(values) >= 2:
                speed_high = values[0]
                speed_low = values[1]
                speed = (speed_high << 16) | speed_low

                # 处理负数
                if speed & 0x80000000:
                    speed = speed - 0x100000000

                self.speed = speed
                self.last_update_time = time.time()

                # 转换为线速度
                self.linear_velocity = (
                    (speed / 60.0 / self.gear_ratio) * math.pi * self.tire_diameter
                )

                return speed
            else:
                return 0
        except Exception as e:
            if self.debug:
                print(f"[MSSD] 获取速度错误: {e}")
            return 0

    def get_cached_speed(self):
        """
        获取缓存的目标速度

        Returns:
            缓存的速度（RPM）
        """
        return self.cached_motor_speed

    def get_linear_velocity(self):
        """
        获取线速度

        Returns:
            线速度（m/s）
        """
        return self.linear_velocity

    def get_encoder(self):
        """
        获取编码器值

        Returns:
            编码器值，失败返回 0
        """
        if not self.connected:
            return 0

        try:
            # 读取编码器寄存器（22-23）
            values = self._read_registers(22, 2)
            if values and len(values) >= 2:
                encoder_high = values[0]
                encoder_low = values[1]
                encoder = (encoder_high << 16) | encoder_low

                # 处理负数
                if encoder & 0x80000000:
                    encoder = encoder - 0x100000000

                return encoder
            else:
                return 0
        except Exception as e:
            if self.debug:
                print(f"[MSSD] 获取编码器错误: {e}")
            return 0

    def update_encoder_distance(self):
        """
        更新编码器里程

        根据编码器脉冲增量计算行驶距离：
        - 正编码代表向前，负编码代表向后
        - 轮子每转一圈是24个脉冲
        - 里程 = (encoder_delta / 24) × π × 轮胎直径

        Returns:
            (distance_delta, total_distance) - 本次里程增量(米)和累计里程(米)
        """
        current_encoder = self.get_encoder()

        if self.last_encoder_value is None:
            # 首次读取，只记录不计算
            self.last_encoder_value = current_encoder
            self.encoder_distance_delta = 0.0
            return 0.0, self.encoder_distance

        # 计算编码器增量（处理溢出情况）
        encoder_delta = current_encoder - self.last_encoder_value

        # 处理32位有符号整数溢出
        if encoder_delta > 0x7FFFFFFF:
            encoder_delta -= 0x100000000
        elif encoder_delta < -0x7FFFFFFF:
            encoder_delta += 0x100000000

        # 计算里程增量
        # 里程 = (脉冲数 / 每转脉冲数) × 轮胎周长
        # 轮胎周长 = π × 直径
        tire_circumference = math.pi * self.tire_diameter
        self.encoder_distance_delta = (
            encoder_delta / self.encoder_pulses_per_rev
        ) * tire_circumference

        # 累加总里程
        self.encoder_distance += self.encoder_distance_delta
        self.last_encoder_value = current_encoder

        if self.debug and encoder_delta != 0:
            print(
                f"[MSSD] Encoder: delta={encoder_delta}, distance_delta={self.encoder_distance_delta:.4f}m, total={self.encoder_distance:.4f}m"
            )

        return self.encoder_distance_delta, self.encoder_distance

    def get_encoder_distance(self):
        """
        获取编码器累计里程

        Returns:
            累计里程（米）
        """
        return self.encoder_distance

    def get_encoder_distance_delta(self):
        """
        获取编码器本次里程增量

        Returns:
            里程增量（米），正数向前，负数向后
        """
        return self.encoder_distance_delta

    def reset_encoder_distance(self):
        """
        重置编码器里程为0
        """
        self.encoder_distance = 0.0
        self.encoder_distance_delta = 0.0
        self.last_encoder_value = None
        if self.debug:
            print("[MSSD] 编码器里程已重置")

    def get_status(self):
        """
        获取驱动器状态信息

        Returns:
            状态字典
        """
        if not self.connected:
            return {"connected": False}

        try:
            status = {}

            # 读取实时状态（寄存器 7-23）
            values = self._read_registers(7, 17)
            if values and len(values) >= 17:
                status["bus_current"] = values[0] / 100.0
                status["motor_current"] = values[1] / 100.0

                # 速度（32位有符号整数）
                speed_high = values[2]
                speed_low = values[3]
                speed = (speed_high << 16) | speed_low
                if speed & 0x80000000:
                    speed = speed - 0x100000000
                status["speed"] = speed

                # 错误状态
                error_code = values[4]
                status["error_code"] = error_code
                status["error_message"] = self._get_error_message(error_code)

                # 电压和温度
                status["voltage"] = values[13]
                status["motor_temperature"] = values[12]

                # 编码器
                encoder_high = values[15]
                encoder_low = values[16]
                encoder = (encoder_high << 16) | encoder_low
                if encoder & 0x80000000:
                    encoder = encoder - 0x100000000
                status["encoder"] = encoder

            status["connected"] = True
            status["device_id"] = self.device_id
            status["cached_speed"] = self.cached_motor_speed

            return status
        except Exception as e:
            if self.debug:
                print(f"[MSSD] 获取状态错误: {e}")
            return {"connected": False}

    def _get_error_message(self, error_code):
        """获取错误信息"""
        error_messages = {
            0: "无错误",
            1: "堵转停机",
            2: "过流停机",
            3: "过压停机",
            4: "欠压停机",
            5: "过热停机",
            6: "过载停机",
            7: "缺相停机",
            8: "编码器故障",
            9: "通信故障",
            10: "参数错误",
            11: "急停触发",
            12: "方向错误",
        }
        return error_messages.get(error_code, f"未知错误({error_code})")

    def __del__(self):
        """析构函数，自动断开连接"""
        self.disconnect()
