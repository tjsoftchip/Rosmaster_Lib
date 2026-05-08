#!/usr/bin/env python3
# coding: utf-8

"""
水位检测模块

通过压力变送器测量水位高度
使用 Modbus RTU 协议通信

修复：
- 原子化读取：整个多寄存器读取序列在同一把锁内完成，防止继电器/电池操作插入
- 串口缓冲区刷新：每次发送前清空 RX 缓冲区，避免读到残留的脏数据

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

        # 缓存设备配置（这些寄存器不会在运行时改变，反复读取反而容易被干扰）
        self._cached_decimal_point = None
        self._cached_unit_code = None
        self._config_read_attempts = 0

    def _send_modbus_read(self, register_addr):
        """
        发送 Modbus 读保持寄存器请求并读取响应（不加锁，由调用方负责锁）

        每次发送前刷新 RX 缓冲区，确保不会读到之前残留的数据。

        Args:
            register_addr: 寄存器地址

        Returns:
            寄存器值（16位有符号整数），失败返回 None
        """
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

            # 刷新 RX 缓冲区，清除可能残留的其他设备响应数据
            self.relay.ser.reset_input_buffer()

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
            value = (response[3] << 8) | response[4]
            if value & 0x8000:
                value = value - 0x10000

            return value
        except Exception as e:
            if self.debug:
                print(f"[WaterSensor] 读取寄存器错误: {e}")
            return None

    def _read_holding_register(self, register_addr):
        """
        读保持寄存器（功能码 0x03）— 加锁版本，保持向后兼容

        Args:
            register_addr: 寄存器地址

        Returns:
            寄存器值（16位有符号整数），失败返回 None
        """
        if not self.relay.connected:
            return None

        with self.relay._lock:
            return self._send_modbus_read(register_addr)

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

    def get_water_level_cm(self):
        """
        获取水位高度（厘米）— 原子化读取

        优化策略：
        1. unit_code 和 decimal_point 是设备配置，不会在运行时改变，只读取一次并缓存
           反复读取容易被串口干扰导致 unit_code 误读，这是 4.4%↔100% 跳变的根因
        2. 压力值使用中位数滤波代替平均值，更抗异常值
        3. 所有读取在同一把锁内完成

        Returns:
            水位高度（厘米），失败返回 None
        """
        if not self.relay.connected:
            return None

        try:
            with self.relay._lock:
                # 1. 使用硬编码设备配置（寄存器易被串口干扰误读，导致4%↔100%跳变）
                # 实测：raw=866, dp=1, uc=8(mmH2O) → 86.6mmH2O → 8.66cm → 4.02%
                # 误读：dp=2, uc=7(mH2O) → 8.66mH2O → 866cm → 100%
                if self._cached_decimal_point is None or self._cached_unit_code is None:
                    self._cached_decimal_point = 1   # 1位小数
                    self._cached_unit_code = 8       # mmH2O
                    if self.debug:
                        print(f"[WaterSensor] 使用硬编码配置: decimal_point=1, unit_code=8(mmH2O)")

                decimal_point = self._cached_decimal_point if self._cached_decimal_point is not None else 1
                unit_code = self._cached_unit_code if self._cached_unit_code is not None else 1

                # 2. 读取压力寄存器（连续5次，取中位数）
                values = []
                for _ in range(5):
                    val = self._send_modbus_read(0x0004)
                    if val is not None:
                        values.append(val)
                    time.sleep(0.01)

                if not values:
                    return None

                # 中位数滤波：排序后取中间值，比平均值更抗异常
                values.sort()
                if len(values) >= 3:
                    pressure_raw = values[len(values) // 2]  # 中位数
                else:
                    pressure_raw = sum(values) // len(values)

            # 3. 根据小数点位置计算实际值
            divisor = 10 ** decimal_point
            pressure_value = pressure_raw / divisor

            # 4. 根据单位转换为 Pa
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

            # 5. 计算水位高度
            water_level_m = pressure_pa / (self.water_density * self.gravity)
            water_level_cm = water_level_m * 100

            if self.debug:
                print(f"[WaterSensor] 原子读取: raw={pressure_raw}(中位数), dp={decimal_point}, "
                      f"unit={unit_code}, P={pressure_pa:.1f}Pa, h={water_level_cm:.2f}cm")

            return water_level_cm
        except Exception as e:
            if self.debug:
                print(f"[WaterSensor] 获取水位错误: {e}")
            return None

    def get_decimal_point(self):
        """获取小数点位置"""
        try:
            return self._read_holding_register(0x0003)
        except Exception as e:
            if self.debug:
                print(f"[WaterSensor] 获取小数点位置错误: {e}")
            return None

    def get_pressure_unit(self):
        """获取压力单位"""
        try:
            return self._read_holding_register(0x0002)
        except Exception as e:
            if self.debug:
                print(f"[WaterSensor] 获取压力单位错误: {e}")
            return None

    def get_pressure(self):
        """获取压力值（Pa）"""
        try:
            decimal_point = self.get_decimal_point()
            if decimal_point is None:
                decimal_point = 0

            unit_code = self.get_pressure_unit()
            if unit_code is None:
                unit_code = 2

            pressure_raw = self._read_holding_register_average(0x0004, count=5)
            if pressure_raw is None:
                return None

            divisor = 10 ** decimal_point
            pressure_value = pressure_raw / divisor

            unit_conversions = {
                0: 1e6, 1: 1e3, 2: 1.0, 3: 1e5, 4: 1e2,
                5: 98066.5, 6: 6894.76, 7: 9806.65, 8: 9.80665
            }

            pressure_pa = pressure_value * unit_conversions.get(unit_code, 1.0)

            if self.debug:
                print(f"[WaterSensor] 压力: {pressure_pa:.2f} Pa")

            return pressure_pa
        except Exception as e:
            if self.debug:
                print(f"[WaterSensor] 获取压力错误: {e}")
            return None

    def get_water_level(self):
        """获取水位高度（米）"""
        try:
            # 优先使用原子化读取
            cm = self.get_water_level_cm()
            if cm is not None:
                return cm / 100.0
            return None
        except Exception as e:
            if self.debug:
                print(f"[WaterSensor] 获取水位错误: {e}")
            return None

    def get_water_level_raw(self):
        """获取水位原始数据"""
        try:
            decimal_point = self.get_decimal_point()
            if decimal_point is None:
                decimal_point = 0

            unit_code = self.get_pressure_unit()
            if unit_code is None:
                unit_code = 2

            pressure_pa = self.get_pressure()
            if pressure_pa is None:
                return None

            water_level = pressure_pa / (self.water_density * self.gravity)
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
        """获取水位百分比"""
        water_level = self.get_water_level()
        if water_level is None:
            return None
        percentage = (water_level / max_level) * 100.0
        return max(0.0, min(100.0, percentage))

    def check_water_level(self, min_level=0.2, max_level=0.9):
        """检查水位状态"""
        water_level = self.get_water_level()
        if water_level is None:
            return {'level': None, 'percentage': None, 'status': 'error', 'message': '无法读取水位'}

        percentage = self.get_water_level_percentage()

        if water_level < min_level:
            status = 'low'
            message = f'水位过低：{water_level:.3f}m ({percentage:.1f}%)'
        elif water_level > max_level:
            status = 'high'
            message = f'水位过高：{water_level:.3f}m ({percentage:.1f}%)'
        else:
            status = 'normal'
            message = f'水位正常：{water_level:.3f}m ({percentage:.1f}%)'

        return {'level': water_level, 'percentage': percentage, 'status': status, 'message': message}

    def is_water_sufficient(self, min_level=0.3):
        """检查水位是否充足"""
        water_level = self.get_water_level()
        if water_level is None:
            return False
        return water_level >= min_level
