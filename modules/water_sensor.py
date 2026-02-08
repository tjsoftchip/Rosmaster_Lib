#!/usr/bin/env python3
# coding: utf-8

"""
水位检测模块

通过压力变送器测量水位高度
使用 Modbus RTU 协议通信

作者：nx-ros2
日期：2026-01-16
"""

import time
from ..utils.crc16 import calculate_crc16
from ..core.constants import WATER_SENSOR_DEVICE_ID, WATER_DENSITY, GRAVITY


class WaterSensor:
    """
    水位传感器控制器
    
    功能：
    - 读取压力值
    - 计算水位高度
    - 水位告警
    """
    
    def __init__(self, relay_controller, device_id=WATER_SENSOR_DEVICE_ID, debug=False):
        """
        初始化水位传感器
        
        Args:
            relay_controller: 继电器控制器（复用串口）
            device_id: 水位传感器设备地址（默认 6）
            debug: 调试模式
        """
        self.relay = relay_controller
        self.device_id = device_id
        self.debug = debug
        
        # 物理参数
        self.water_density = WATER_DENSITY  # 水的密度（kg/m³）
        self.gravity = GRAVITY  # 重力加速度（m/s²）
    
    def _read_holding_register(self, register_addr):
        """
        读保持寄存器（功能码 0x03）
        
        Args:
            register_addr: 寄存器地址
        
        Returns:
            寄存器值（16位有符号整数），失败返回 None
        """
        if not self.relay.connected:
            return None
        
        # 使用锁保护串口访问
        with self.relay._lock:
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
                
                # 发送请求
                self.relay.ser.write(frame)
                self.relay.ser.flush()
                
                # 读取响应
                time.sleep(0.02)
                response = self.relay.ser.read(7)
                
                if len(response) != 7:
                    if self.debug:
                        print(f"[WaterSensor] 响应长度不正确: 期望 7 字节, 实际 {len(response)} 字节")
                    return None
                
                # 验证设备 ID 和功能码
                if response[0] != self.device_id:
                    if self.debug:
                        print(f"[WaterSensor] 设备 ID 不匹配: 期望 {self.device_id}, 实际 {response[0]}")
                    return None
                
                if response[1] != 0x03:
                    if self.debug:
                        print(f"[WaterSensor] 功能码不匹配: 期望 0x03, 实际 {response[1]}")
                    return None
                
                # 验证 CRC
                received_crc = (response[-1] << 8) | response[-2]
                calculated_crc = calculate_crc16(response[:-2])
                
                if received_crc != calculated_crc:
                    if self.debug:
                        print("[WaterSensor] CRC 校验错误")
                    return None
                
                # 解析数据（16位有符号整数）
                # Modbus RTU 响应格式: 地址 + 功能码 + 字节数 + 数据高位 + 数据低位 + CRC低位 + CRC高位
                value = (response[3] << 8) | response[4]
                if value & 0x8000:
                    value = value - 0x10000

                return value
            except Exception as e:
                if self.debug:
                    print(f"[WaterSensor] 读取寄存器错误: {e}")
                return None
    
    def _read_holding_register_average(self, register_addr, count=5):
        """
        连续读取保持寄存器多次并取平均值
        
        Args:
            register_addr: 寄存器地址
            count: 读取次数（默认5次）
        
        Returns:
            平均值（16位有符号整数），失败返回 None
        """
        values = []
        
        for i in range(count):
            value = self._read_holding_register(register_addr)
            if value is None:
                continue
            values.append(value)
            time.sleep(0.01)  # 每次读取间隔10ms
        
        if not values:
            return None
        
        # 计算平均值
        average = sum(values) // len(values)
        
        if self.debug:
            print(f"[WaterSensor] 寄存器0x{register_addr:04X}读取{len(values)}次，平均值: {average}")
        
        return average
    
    def get_decimal_point(self):
        """
        获取小数点位置
        
        Returns:
            小数点位置（0-3），失败返回 None
        
        Note:
            寄存器0x0003存储小数点位置
            0-整数，1-1位小数，2-2位小数，3-3位小数
        """
        try:
            decimal_point = self._read_holding_register(0x0003)
            
            if decimal_point is not None:
                if self.debug:
                    print(f"[WaterSensor] 小数点位置: {decimal_point}")
                return decimal_point
            else:
                return None
        except Exception as e:
            if self.debug:
                print(f"[WaterSensor] 获取小数点位置错误: {e}")
            return None
    
    def get_pressure_unit(self):
        """
        获取压力单位
        
        Returns:
            压力单位代码，失败返回 None
        
        Note:
            寄存器0x0002存储压力单位
            0- Mpa/℃, 1- Kpa, 2- Pa, 3- Bar, 4- Mbar, 5- kg/cm2, 6- psi, 7- mh2o, 8- mmh2o
        """
        try:
            unit_code = self._read_holding_register(0x0002)
            
            if unit_code is not None:
                if self.debug:
                    units = ['Mpa/℃', 'Kpa', 'Pa', 'Bar', 'Mbar', 'kg/cm2', 'psi', 'mh2o', 'mmh2o']
                    unit_name = units[unit_code] if 0 <= unit_code < len(units) else f'Unknown({unit_code})'
                    print(f"[WaterSensor] 压力单位: {unit_name}")
                return unit_code
            else:
                return None
        except Exception as e:
            if self.debug:
                print(f"[WaterSensor] 获取压力单位错误: {e}")
            return None
    
    def get_pressure(self):
        """
        获取压力值（连续读取5次取平均值）
        
        Returns:
            压力值（Pa），失败返回 None
        
        Note:
            寄存器0x0004存储压力值（原始值）
            需要根据小数点位置和单位转换为Pa
        """
        try:
            # 读取小数点位置
            decimal_point = self.get_decimal_point()
            if decimal_point is None:
                decimal_point = 0  # 默认无小数点
            
            # 读取压力单位
            unit_code = self.get_pressure_unit()
            if unit_code is None:
                unit_code = 2  # 默认Pa
            
            # 读取压力寄存器（地址0x0004），连续5次取平均值
            pressure_raw = self._read_holding_register_average(0x0004, count=5)
            
            if pressure_raw is None:
                return None
            
            # 根据小数点位置计算实际值
            divisor = 10 ** decimal_point
            pressure_value = pressure_raw / divisor
            
            # 根据单位转换为Pa
            # 0-Mpa, 1-Kpa, 2-Pa, 3-Bar, 4-Mbar, 5-kg/cm2, 6-psi, 7-mh2o, 8-mmh2o
            unit_conversions = {
                0: 1e6,      # Mpa -> Pa
                1: 1e3,      # Kpa -> Pa
                2: 1.0,      # Pa -> Pa
                3: 1e5,      # Bar -> Pa
                4: 1e2,      # Mbar -> Pa
                5: 98066.5,  # kg/cm2 -> Pa
                6: 6894.76,  # psi -> Pa
                7: 9806.65,  # mh2o -> Pa
                8: 9.80665   # mmh2o -> Pa
            }
            
            conversion_factor = unit_conversions.get(unit_code, 1.0)
            pressure_pa = pressure_value * conversion_factor
            
            if self.debug:
                print(f"[WaterSensor] 压力原始值: {pressure_raw}, 小数点: {decimal_point}, 单位代码: {unit_code}")
                print(f"[WaterSensor] 压力值: {pressure_pa:.2f} Pa")
            
            return pressure_pa
        except Exception as e:
            if self.debug:
                print(f"[WaterSensor] 获取压力错误: {e}")
            return None
    
    def get_water_level(self):
        """
        获取水位高度（对外API）

        Returns:
            水位高度（米），失败返回 None

        Note:
            水位计算公式：h = P / (ρ * g)
            其中：
            - h: 水位高度（m）
            - P: 压力（Pa）
            - ρ: 水的密度（kg/m³）
            - g: 重力加速度（m/s²）

            此方法会连续读取5次压力数据取平均值后计算水位
        """
        try:
            # 读取压力值（已包含连续5次读取取平均）
            pressure_pa = self.get_pressure()

            if pressure_pa is None:
                return None

            # 计算水位高度：h = P / (ρ * g)
            water_level = pressure_pa / (self.water_density * self.gravity)

            if self.debug:
                print(f"[WaterSensor] 水位: {water_level:.3f} m")

            return water_level
        except Exception as e:
            if self.debug:
                print(f"[WaterSensor] 获取水位错误: {e}")
            return None

    def get_water_level_cm(self):
        """
        获取水位高度（厘米单位）

        Returns:
            水位高度（厘米），失败返回 None

        Note:
            水位计算公式：h = P / (ρ * g)
            其中：
            - h: 水位高度（m）
            - P: 压力（Pa）
            - ρ: 水的密度（kg/m³）
            - g: 重力加速度（m/s²）

            此方法会连续读取5次压力数据取平均值后计算水位
        """
        try:
            # 读取压力值（已包含连续5次读取取平均）
            pressure_pa = self.get_pressure()

            if pressure_pa is None:
                return None

            # 计算水位高度：h = P / (ρ * g) (米)
            water_level_m = pressure_pa / (self.water_density * self.gravity)

            # 转换为厘米
            water_level_cm = water_level_m * 100

            if self.debug:
                print(f"[WaterSensor] 水位: {water_level_cm:.2f} cm")

            return water_level_cm
        except Exception as e:
            if self.debug:
                print(f"[WaterSensor] 获取水位错误: {e}")
            return None
    
    def get_water_level_raw(self):
        """
        获取水位原始数据（包含压力和计算过程）

        Returns:
            字典包含：
            - pressure_pa: 压力值（Pa）
            - water_level: 水位高度（m）
            - water_level_cm: 水位高度（cm）
            - unit_code: 压力单位代码
            - decimal_point: 小数点位置
            失败返回 None
        """
        try:
            # 读取小数点位置
            decimal_point = self.get_decimal_point()
            if decimal_point is None:
                decimal_point = 0

            # 读取压力单位
            unit_code = self.get_pressure_unit()
            if unit_code is None:
                unit_code = 2

            # 读取压力值
            pressure_pa = self.get_pressure()

            if pressure_pa is None:
                return None

            # 计算水位（米）
            water_level = pressure_pa / (self.water_density * self.gravity)

            # 计算水位（厘米）
            water_level_cm = water_level * 100

            return {
                'pressure_pa': pressure_pa,
                'water_level': water_level,
                'water_level_cm': water_level_cm,
                'unit_code': unit_code,
                'decimal_point': decimal_point
            }
        except Exception as e:
            if self.debug:
                print(f"[WaterSensor] 获取水位原始数据错误: {e}")
            return None
    
    def get_water_level_percentage(self, max_level=1.0):
        """
        获取水位百分比
        
        Args:
            max_level: 最大水位高度（米），默认1.0米
        
        Returns:
            水位百分比（0-100），失败返回 None
        """
        water_level = self.get_water_level()
        
        if water_level is None:
            return None
        
        percentage = (water_level / max_level) * 100.0
        percentage = max(0.0, min(100.0, percentage))  # 限制在0-100范围内
        
        if self.debug:
            print(f"[WaterSensor] 水位百分比: {percentage:.1f}%")
        
        return percentage
    
    def check_water_level(self, min_level=0.2, max_level=0.9):
        """
        检查水位状态
        
        Args:
            min_level: 最低水位（米）
            max_level: 最高水位（米）
        
        Returns:
            状态字典：
            - level: 水位高度（米）
            - percentage: 水位百分比
            - status: 状态（'low', 'normal', 'high'）
            - message: 状态消息
        """
        water_level = self.get_water_level()
        
        if water_level is None:
            return {
                'level': None,
                'percentage': None,
                'status': 'error',
                'message': '无法读取水位'
            }
        
        percentage = self.get_water_level_percentage()
        
        # 判断水位状态
        if water_level < min_level:
            status = 'low'
            message = f'水位过低：{water_level:.3f}m ({percentage:.1f}%)'
        elif water_level > max_level:
            status = 'high'
            message = f'水位过高：{water_level:.3f}m ({percentage:.1f}%)'
        else:
            status = 'normal'
            message = f'水位正常：{water_level:.3f}m ({percentage:.1f}%)'
        
        if self.debug:
            print(f"[WaterSensor] {message}")
        
        return {
            'level': water_level,
            'percentage': percentage,
            'status': status,
            'message': message
        }
    
    def is_water_sufficient(self, min_level=0.3):
        """
        检查水位是否充足
        
        Args:
            min_level: 最低水位（米）
        
        Returns:
            True=水位充足，False=水位不足
        """
        water_level = self.get_water_level()
        
        if water_level is None:
            return False
        
        sufficient = water_level >= min_level
        
        if self.debug:
            status = '充足' if sufficient else '不足'
            print(f"[WaterSensor] 水位{status}：{water_level:.3f}m")
        
        return sufficient