#!/usr/bin/env python3
# coding: utf-8

"""
电池库仑计监控模块

通过库仑计模块监控电池电量、电压、电流、功率等参数
使用 Modbus RTU 协议通信

作者：nx-ros2
日期：2026-01-17
"""

import time
from ..utils.crc16 import calculate_crc16


class BatteryMonitor:
    """
    电池库仑计控制器
    
    功能：
    - 读取电池容量百分比
    - 读取电压、电流、功率
    - 读取温度
    - 读取累计容量和电能
    - 读取报警状态和充放电状态
    """
    
    def __init__(self, relay_controller, device_id=7, debug=False):
        """
        初始化电池库仑计
        
        Args:
            relay_controller: 继电器控制器（复用串口）
            device_id: 库仑计设备地址（默认 7）
            debug: 调试模式
        """
        self.relay = relay_controller
        self.device_id = device_id
        self.debug = debug
    
    def _read_input_register(self, register_addr, count=1):
        """
        读输入寄存器（功能码 0x04）
        
        Args:
            register_addr: 寄存器地址
            count: 读取数量
        
        Returns:
            寄存器值列表，失败返回 None
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
                    (count >> 8) & 0xFF,
                    count & 0xFF
                ])
                
                frame = bytes([self.device_id, 0x04]) + data
                crc = calculate_crc16(frame)
                frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
                
                # 发送请求
                self.relay.ser.write(frame)
                self.relay.ser.flush()
                
                # 读取响应
                time.sleep(0.02)
                response = self.relay.ser.read(5 + count * 2)
                
                if len(response) < 5:
                    if self.debug:
                        print(f"[BatteryMonitor] 响应长度不足: 期望 {5 + count * 2} 字节, 实际 {len(response)} 字节")
                    return None
                
                # 验证设备 ID 和功能码
                if response[0] != self.device_id:
                    if self.debug:
                        print(f"[BatteryMonitor] 设备 ID 不匹配: 期望 {self.device_id}, 实际 {response[0]}")
                    return None
                
                if response[1] != 0x04:
                    if self.debug:
                        print(f"[BatteryMonitor] 功能码不匹配: 期望 0x04, 实际 {response[1]}")
                    return None
                
                # 验证 CRC
                received_crc = (response[-1] << 8) | response[-2]
                calculated_crc = calculate_crc16(response[:-2])
                
                if received_crc != calculated_crc:
                    if self.debug:
                        print("[BatteryMonitor] CRC 校验错误")
                    return None
                
                # 解析数据（高字节在前）
                byte_count = response[2]
                values = []
                for i in range(byte_count // 2):
                    value = (response[3 + i * 2] << 8) | response[3 + i * 2 + 1]
                    values.append(value)

                return values
            except Exception as e:
                if self.debug:
                    print(f"[BatteryMonitor] 读取输入寄存器错误: {e}")
                return None
    
    def _read_input_register_average(self, register_addr, count=1, read_count=5):
        """
        连续读取输入寄存器多次并取平均值
        
        Args:
            register_addr: 寄存器地址
            count: 每次读取的寄存器数量
            read_count: 读取次数（默认5次）
        
        Returns:
            平均值列表，失败返回 None
        """
        all_values = []
        
        for i in range(read_count):
            values = self._read_input_register(register_addr, count)
            if values is None:
                continue
            all_values.append(values)
            time.sleep(0.01)  # 每次读取间隔10ms
        
        if not all_values:
            return None
        
        # 计算平均值
        averages = []
        for j in range(count):
            sum_val = sum(values[j] for values in all_values)
            avg_val = sum_val // len(all_values)
            averages.append(avg_val)
        
        if self.debug:
            print(f"[BatteryMonitor] 寄存器0x{register_addr:04X}读取{len(all_values)}次，平均值: {averages}")
        
        return averages
    
    def get_battery_percentage(self):
        """
        获取电池容量百分比（对外API）
        
        Returns:
            容量百分比（0-100），失败返回 None
            
        Note:
            寄存器地址：0x0000
            此方法会连续读取5次数据取平均值
        """
        try:
            values = self._read_input_register_average(0x0000, count=1, read_count=5)
            
            if values is None or len(values) == 0:
                return None
            
            percentage = values[0]
            percentage = max(0, min(100, percentage))  # 限制在0-100范围内
            
            if self.debug:
                print(f"[BatteryMonitor] 电池容量百分比: {percentage}%")
            
            return percentage
        except Exception as e:
            if self.debug:
                print(f"[BatteryMonitor] 获取电池容量百分比错误: {e}")
            return None
    
    def get_battery_voltage(self):
        """
        获取电池电压
        
        Returns:
            电压值（V），失败返回 None
            
        Note:
            寄存器地址：0x0002
            读取值需要除以100得到实际电压值
        """
        try:
            values = self._read_input_register(0x0002, count=1)
            
            if values is None or len(values) == 0:
                return None
            
            voltage = values[0] / 100.0
            
            if self.debug:
                print(f"[BatteryMonitor] 电池电压: {voltage:.2f}V")
            
            return voltage
        except Exception as e:
            if self.debug:
                print(f"[BatteryMonitor] 获取电池电压错误: {e}")
            return None
    
    def get_battery_current(self):
        """
        获取电池电流
        
        Returns:
            电流值（A），失败返回 None
            
        Note:
            寄存器地址：0x0003
            读取值需要除以100得到实际电流值
            正值表示充电，负值表示放电
        """
        try:
            values = self._read_input_register(0x0003, count=1)
            
            if values is None or len(values) == 0:
                return None
            
            # 处理有符号整数
            current_raw = values[0]
            if current_raw & 0x8000:
                current_raw = current_raw - 0x10000
            
            current = current_raw / 100.0
            
            if self.debug:
                print(f"[BatteryMonitor] 电池电流: {current:.2f}A")
            
            return current
        except Exception as e:
            if self.debug:
                print(f"[BatteryMonitor] 获取电池电流错误: {e}")
            return None
    
    def get_battery_power(self):
        """
        获取电池功率
        
        Returns:
            功率值（W），失败返回 None
            
        Note:
            寄存器地址：0x0004-0x0005（32位无符号整数）
            读取值需要除以10得到实际功率值
        """
        try:
            values = self._read_input_register(0x0004, count=2)
            
            if values is None or len(values) < 2:
                return None
            
            # 组合32位无符号整数
            power_raw = (values[0] << 16) | values[1]
            power = power_raw / 10.0
            
            if self.debug:
                print(f"[BatteryMonitor] 电池功率: {power:.2f}W")
            
            return power
        except Exception as e:
            if self.debug:
                print(f"[BatteryMonitor] 获取电池功率错误: {e}")
            return None
    
    def get_battery_temperature(self):
        """
        获取电池温度
        
        Returns:
            温度值（℃），失败返回 None
            
        Note:
            寄存器地址：0x0001
            直接使用读取值
        """
        try:
            values = self._read_input_register(0x0001, count=1)
            
            if values is None or len(values) == 0:
                return None
            
            # 处理有符号整数
            temp_raw = values[0]
            if temp_raw & 0x8000:
                temp_raw = temp_raw - 0x10000
            
            temperature = temp_raw
            
            if self.debug:
                print(f"[BatteryMonitor] 电池温度: {temperature}℃")
            
            return temperature
        except Exception as e:
            if self.debug:
                print(f"[BatteryMonitor] 获取电池温度错误: {e}")
            return None
    
    def get_battery_capacity_remaining(self):
        """
        获取剩余容量
        
        Returns:
            剩余容量（mAh），失败返回 None
            
        Note:
            寄存器地址：0x0008-0x0009（32位无符号整数）
            单位为mAh
        """
        try:
            values = self._read_input_register(0x0008, count=2)
            
            if values is None or len(values) < 2:
                return None
            
            # 组合32位无符号整数
            capacity = (values[0] << 16) | values[1]
            
            if self.debug:
                capacity_ah = capacity / 1000.0
                print(f"[BatteryMonitor] 剩余容量: {capacity}mAh ({capacity_ah:.2f}Ah)")
            
            return capacity
        except Exception as e:
            if self.debug:
                print(f"[BatteryMonitor] 获取剩余容量错误: {e}")
            return None
    
    def get_battery_energy_accumulated(self):
        """
        获取累计电能
        
        Returns:
            累计电能（Wh），失败返回 None
            
        Note:
            寄存器地址：0x0006-0x0007（32位无符号整数）
            读取值需要除以100得到实际电能值
        """
        try:
            values = self._read_input_register(0x0006, count=2)
            
            if values is None or len(values) < 2:
                return None
            
            # 组合32位无符号整数
            energy_raw = (values[0] << 16) | values[1]
            energy = energy_raw / 100.0
            
            if self.debug:
                energy_kwh = energy / 1000.0
                print(f"[BatteryMonitor] 累计电能: {energy:.2f}Wh ({energy_kwh:.4f}kWh)")
            
            return energy
        except Exception as e:
            if self.debug:
                print(f"[BatteryMonitor] 获取累计电能错误: {e}")
            return None
    
    def get_alarm_status(self):
        """
        获取报警状态
        
        Returns:
            报警状态字典，失败返回 None
            
        Note:
            寄存器地址：0x000A
            位标志：
            - bit0: 过压报警
            - bit1: 过流报警
            - bit2: 高温报警
            - bit3: 低压报警
            - bit4: 低电量报警
        """
        try:
            values = self._read_input_register(0x000A, count=1)
            
            if values is None or len(values) == 0:
                return None
            
            status = values[0]
            
            alarm_status = {
                'over_voltage': bool(status & 0x01),
                'over_current': bool(status & 0x02),
                'high_temperature': bool(status & 0x04),
                'low_voltage': bool(status & 0x08),
                'low_battery': bool(status & 0x10),
                'raw_status': status
            }
            
            if self.debug:
                alarms = []
                if alarm_status['over_voltage']:
                    alarms.append('过压')
                if alarm_status['over_current']:
                    alarms.append('过流')
                if alarm_status['high_temperature']:
                    alarms.append('高温')
                if alarm_status['low_voltage']:
                    alarms.append('低压')
                if alarm_status['low_battery']:
                    alarms.append('低电量')
                
                if alarms:
                    print(f"[BatteryMonitor] 报警状态: {', '.join(alarms)}")
                else:
                    print("[BatteryMonitor] 无报警")
            
            return alarm_status
        except Exception as e:
            if self.debug:
                print(f"[BatteryMonitor] 获取报警状态错误: {e}")
            return None
    
    def get_charge_discharge_status(self):
        """
        获取充放电状态
        
        Returns:
            状态字符串，失败返回 None
            
        Note:
            寄存器地址：0x000B
            - 0: 静置状态
            - 1: 放电状态
            - 2: 充电状态
        """
        try:
            values = self._read_input_register(0x000B, count=1)
            
            if values is None or len(values) == 0:
                return None
            
            status_code = values[0]
            
            status_map = {
                0: 'idle',      # 静置状态
                1: 'discharge', # 放电状态
                2: 'charge'     # 充电状态
            }
            
            status = status_map.get(status_code, 'unknown')
            
            if self.debug:
                status_names = {
                    'idle': '静置',
                    'discharge': '放电',
                    'charge': '充电',
                    'unknown': '未知'
                }
                print(f"[BatteryMonitor] 充放电状态: {status_names.get(status, '未知')}")
            
            return status
        except Exception as e:
            if self.debug:
                print(f"[BatteryMonitor] 获取充放电状态错误: {e}")
            return None
    
    def get_battery_status(self):
        """
        获取电池完整状态（对外API）
        
        Returns:
            状态字典，包含所有电池参数，失败返回 None
        """
        try:
            status = {
                'percentage': self.get_battery_percentage(),
                'voltage': self.get_battery_voltage(),
                'current': self.get_battery_current(),
                'power': self.get_battery_power(),
                'temperature': self.get_battery_temperature(),
                'capacity_remaining': self.get_battery_capacity_remaining(),
                'energy_accumulated': self.get_battery_energy_accumulated(),
                'alarm_status': self.get_alarm_status(),
                'charge_discharge_status': self.get_charge_discharge_status()
            }
            
            if self.debug:
                print(f"[BatteryMonitor] 电池状态: 电量={status['percentage']}%, 电压={status['voltage']}V, 电流={status['current']}A")
            
            return status
        except Exception as e:
            if self.debug:
                print(f"[BatteryMonitor] 获取电池状态错误: {e}")
            return None
    
    def is_battery_low(self, threshold=20):
        """
        检查电池电量是否过低
        
        Args:
            threshold: 电量阈值（%），默认20%
        
        Returns:
            True=电量过低，False=电量正常，None=无法读取
        """
        percentage = self.get_battery_percentage()
        
        if percentage is None:
            return None
        
        is_low = percentage < threshold
        
        if self.debug:
            status = '过低' if is_low else '正常'
            print(f"[BatteryMonitor] 电池电量{status}: {percentage}%")
        
        return is_low
    
    def is_battery_sufficient(self, threshold=30):
        """
        检查电池电量是否充足
        
        Args:
            threshold: 电量阈值（%），默认30%
        
        Returns:
            True=电量充足，False=电量不足，None=无法读取
        """
        percentage = self.get_battery_percentage()
        
        if percentage is None:
            return None
        
        sufficient = percentage >= threshold
        
        if self.debug:
            status = '充足' if sufficient else '不足'
            print(f"[BatteryMonitor] 电池电量{status}: {percentage}%")
        
        return sufficient