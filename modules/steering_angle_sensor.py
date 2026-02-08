#!/usr/bin/env python3
# coding: utf-8

"""
转向角度传感器模块

通过 ADC 模块读取转向角度传感器电压值
使用 Modbus RTU 协议通信

设备信息：
- 设备 ID: 8
- 功能码: 0x03 (读保持寄存器)
- 寄存器地址: 0x0000 (第一通道)
- 波特率: 115200
- 串口: /dev/mssd

注意：
- 与 MSSD 电机控制器共用 /dev/mssd 串口
- 电机控制优先级高于转向角度读取
- 使用锁机制避免通信冲突

作者：nx-ros2
日期：2026-01-24
"""

import time
import threading
from collections import deque
from ..utils.crc16 import calculate_crc16
from ..core.constants import DEFAULT_BAUDRATE


class SteeringAngleSensor:
    """
    转向角度传感器控制器
    
    功能：
    - 读取第一通道 ADC 电压值
    - 电压值转换为角度（需要校准数据）
    - 线程安全的串口访问
    - 优先级控制（电机控制优先）
    """
    
    def __init__(self, mssd_controller, device_id=8, baudrate=DEFAULT_BAUDRATE, debug=False):
        """
        初始化转向角度传感器
        
        Args:
            mssd_controller: MSSD 控制器（复用串口）
            device_id: ADC 模块设备地址（默认 8）
            baudrate: 波特率（默认 115200）
            debug: 调试模式
        """
        self.mssd = mssd_controller
        self.device_id = device_id
        self.baudrate = baudrate
        self.debug = debug
        
        # 校准数据（角度 -> 电压）
        self.calibration_data = [
            (0, 0.915),
            (45, 1.696),
            (90, 2.477),
            (135, 3.247),
            (180, 4.037)
        ]

        # 串口访问锁（与 MSSD 控制器共享）
        self._lock = mssd_controller._lock

        # 电压缓存
        self.cached_voltage = 0.0
        self.last_read_time = 0
        self.cache_valid = False
        
        # 角度滤波（滑动平均）
        self.angle_buffer = deque(maxlen=5)  # 缓冲区大小为5
        self.angle_read_interval = 0.1  # 读取间隔 100ms (10Hz)
        self.last_angle_read_time = 0
    
    def _read_holding_register(self, register_addr):
        """
        读保持寄存器（功能码 0x03）

        Args:
            register_addr: 寄存器地址

        Returns:
            寄存器值（16位无符号整数），失败返回 None
        """
        if not self.mssd.connected or not self.mssd.ser:
            if self.debug:
                print("[SteeringAngleSensor] MSSD 未连接")
            return None

        # 使用锁保护串口访问（电机控制优先，非阻塞获取）
        if not self._lock.acquire(blocking=False):
            if self.debug:
                print("[SteeringAngleSensor] 串口被电机控制占用，跳过读取")
            return None

        try:
            # 构建请求帧
            data = bytes([
                (register_addr >> 8) & 0xFF,
                register_addr & 0xFF,
                0x00,
                0x01
            ])

            frame = bytes([self.device_id, 0x03]) + data
            crc = calculate_crc16(frame)
            frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF])

            # 发送请求（不清空缓冲区，避免干扰 MSSD 控制器）
            self.mssd.ser.write(frame)
            self.mssd.ser.flush()

            # 读取响应
            time.sleep(0.02)
            response = self.mssd.ser.read(7)

            if len(response) != 7:
                if self.debug:
                    print(f"[SteeringAngleSensor] 响应长度不正确: 期望 7 字节, 实际 {len(response)} 字节")
                return None

            # 验证设备 ID 和功能码
            if response[0] != self.device_id:
                if self.debug:
                    print(f"[SteeringAngleSensor] 设备 ID 不匹配: 期望 {self.device_id}, 实际 {response[0]}")
                return None

            if response[1] != 0x03:
                if self.debug:
                    print(f"[SteeringAngleSensor] 功能码不匹配: 期望 0x03, 实际 {response[1]}")
                return None

            # 验证 CRC
            received_crc = (response[-1] << 8) | response[-2]
            calculated_crc = calculate_crc16(response[:-2])
            if received_crc != calculated_crc:
                if self.debug:
                    print(f"[SteeringAngleSensor] CRC 校验失败: 接收 {received_crc:#06x}, 计算 {calculated_crc:#06x}")
                return None

            # 提取数据
            value = (response[3] << 8) | response[4]

            if self.debug:
                print(f"[SteeringAngleSensor] 读取寄存器 0x{register_addr:04X}: {value}")

            return value

        except Exception as e:
            if self.debug:
                print(f"[SteeringAngleSensor] 读取失败: {e}")
            return None
        finally:
            # 释放锁
            self._lock.release()
    
    def get_voltage(self, channel=0, use_cache=True, cache_timeout=0.1):
        """
        获取指定通道的电压值
        
        Args:
            channel: 通道号（0 或 1）
            use_cache: 是否使用缓存数据
            cache_timeout: 缓存超时时间（秒）
        
        Returns:
            电压值（V），失败返回 None
        """
        if channel not in [0, 1]:
            print(f"[SteeringAngleSensor] 无效的通道号: {channel}")
            return None
        
        # 检查缓存
        current_time = time.time()
        if use_cache and self.cache_valid and (current_time - self.last_read_time) < cache_timeout:
            if self.debug:
                print(f"[SteeringAngleSensor] 使用缓存电压: {self.cached_voltage:.3f}V")
            return self.cached_voltage
        
        # 读取原始值
        raw_value = self._read_holding_register(channel)
        
        if raw_value is None:
            return None
        
        # 转换为电压值（12位 ADC: 0-4095 对应 0-5V）
        voltage = (raw_value / 4095.0) * 5.0
        
        # 更新缓存
        self.cached_voltage = voltage
        self.last_read_time = current_time
        self.cache_valid = True
        
        if self.debug:
            print(f"[SteeringAngleSensor] 通道 {channel} 电压: {voltage:.3f}V (原始值: {raw_value})")
        
        return voltage
    
    def get_voltage_raw(self, channel=0):
        """
        获取指定通道的原始 ADC 值
        
        Args:
            channel: 通道号（0 或 1）
        
        Returns:
            原始 ADC 值（0-4095），失败返回 None
        """
        if channel not in [0, 1]:
            print(f"[SteeringAngleSensor] 无效的通道号: {channel}")
            return None
        
        raw_value = self._read_holding_register(channel)
        return raw_value
    
    def voltage_to_angle(self, voltage):
        """
        电压值转换为角度（线性插值）
        
        Args:
            voltage: 电压值（V）
        
        Returns:
            角度值（度），失败返回 None
        """
        if voltage is None:
            return None
        
        points = self.calibration_data
        
        # 超出范围处理
        if voltage <= points[0][1]:
            return points[0][0]
        if voltage >= points[-1][1]:
            return points[-1][0]
        
        # 线性插值
        for i in range(len(points) - 1):
            v1, a1 = points[i]
            v2, a2 = points[i + 1]
            if v1 <= voltage <= v2:
                angle = a1 + (a2 - a1) * (voltage - v1) / (v2 - v1)
                if self.debug:
                    print(f"[SteeringAngleSensor] 电压 {voltage:.3f}V -> 角度 {angle:.1f}°")
                return angle
        
        return None
    
    def get_angle(self, channel=0, use_cache=True, cache_timeout=0.1):
        """
        获取转向角度（带滤波）
        
        Args:
            channel: 通道号（0 或 1）
            use_cache: 是否使用缓存数据
            cache_timeout: 缓存超时时间（秒）
        
        Returns:
            角度值（度），失败返回 None
        """
        voltage = self.get_voltage(channel, use_cache, cache_timeout)
        if voltage is None:
            return None
        
        angle = self.voltage_to_angle(voltage)
        if angle is None:
            return None
        
        # 添加到缓冲区
        self.angle_buffer.append(angle)
        
        # 滑动平均滤波
        if len(self.angle_buffer) >= 3:
            # 使用最近3个值的平均值
            filtered_angle = sum(list(self.angle_buffer)[-3:]) / 3
        else:
            # 缓冲区未满，使用当前值
            filtered_angle = angle
        
        if self.debug:
            print(f"[SteeringAngleSensor] 原始角度: {angle:.1f}°, 滤波后角度: {filtered_angle:.1f}°")
        
        return filtered_angle
    
    def set_calibration_data(self, calibration_data):
        """
        设置校准数据
        
        Args:
            calibration_data: 校准数据列表 [(角度, 电压), ...]
        """
        self.calibration_data = sorted(calibration_data, key=lambda x: x[0])
        print(f"[SteeringAngleSensor] 校准数据已更新: {len(calibration_data)} 个点")
    
    def get_calibration_data(self):
        """
        获取校准数据
        
        Returns:
            校准数据列表 [(角度, 电压), ...]
        """
        return self.calibration_data
    
    def invalidate_cache(self):
        """使缓存失效"""
        self.cache_valid = False
        if self.debug:
            print("[SteeringAngleSensor] 缓存已失效")
    
    def get_status(self):
        """
        获取传感器状态
        
        Returns:
            状态字典
        """
        return {
            'connected': self.mssd.connected,
            'device_id': self.device_id,
            'baudrate': self.baudrate,
            'cached_voltage': self.cached_voltage,
            'cache_valid': self.cache_valid,
            'last_read_time': self.last_read_time,
            'calibration_points': len(self.calibration_data)
        }