#!/usr/bin/env python3
# coding: utf-8

"""
继电器控制模块

用于控制喷水系统的继电器模块，支持8路继电器控制
使用 Modbus RTU 协议通信

通道定义：
- CH0: 左侧展臂
- CH1: 左侧水阀
- CH2: 右侧展臂
- CH3: 右侧水阀
- CH4: 报警器开关
- CH5: 水泵喷水开关（同时负责水泵的开关）
- CH6: 升起喷水支架
- CH7: 降下喷水支架

作者：nx-ros2
日期：2026-01-16
"""

import serial
import time
import threading
from ..utils.crc16 import calculate_crc16
from ..core.constants import RELAY_DEVICE_ID, DEFAULT_BAUDRATE


class RelayController:
    """
    继电器控制器
    
    功能：
    - 控制8路继电器通道
    - 喷水控制（水泵+水阀）
    - 支架控制（升降）
    - 展臂控制（左右）
    """
    
    def __init__(self, port="/dev/modbus", device_id=RELAY_DEVICE_ID,
                 baudrate=9600, debug=False):
        """
        初始化继电器控制器
        
        Args:
            port: 串口设备路径（默认 /dev/modbus）
            device_id: 继电器设备地址（默认 5）
            baudrate: 波特率（默认 9600）
            debug: 调试模式
        """
        self.port = port
        self.device_id = device_id
        self.baudrate = baudrate
        self.debug = debug
        self.connected = False
        self.ser = None
        
        # 串口访问锁（确保线程安全）
        self._lock = threading.Lock()
        
        # 继电器状态
        self.ch1_state = False  # 左侧展臂
        self.ch2_state = False  # 左侧水阀
        self.ch3_state = False  # 右侧展臂
        self.ch4_state = False  # 右侧水阀
        self.ch5_state = False  # 报警器开关
        self.ch6_state = False  # 水泵喷水开关（同时负责水泵的开关）
        self.ch7_state = False  # 升起喷水支架
        self.ch8_state = False  # 降下喷水支架
        
        # 连接串口
        self._connect()
    
    def _connect(self):
        """连接串口（检查是否已被占用）"""
        # 检查串口是否已经被打开
        if self.ser and self.ser.is_open:
            if self.debug:
                print(f"[Relay] 串口 {self.port} 已经打开，复用现有连接")
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
                print(f"[Relay] 串口连接成功: {self.port}")
                print(f"[Relay] 设备地址: {self.device_id}, 波特率: {self.baudrate}")
        except serial.SerialException as e:
            # 如果串口已被占用，尝试获取现有的文件描述符
            if "Permission denied" in str(e) or "could not open port" in str(e):
                if self.debug:
                    print(f"[Relay] 串口 {self.port} 已被占用，尝试查找现有连接...")
                # 这里可以添加逻辑来查找已打开的串口连接
                # 但由于Python的serial模块不支持直接获取已打开的连接，
                # 我们需要在单例级别管理串口连接
                self.connected = False
            else:
                self.connected = False
                if self.debug:
                    print(f"[Relay] 串口连接失败: {e}")
    
    def disconnect(self):
        """断开串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.connected = False
            if self.debug:
                print("[Relay] 串口已断开")
    
    def _write_coil(self, coil_addr, value, wait_response=True):
        """
        写线圈（功能码 0x05）

        Args:
            coil_addr: 线圈地址（0-7 对应通道1-8）
            value: 线圈值（0=关闭，1=开启）
            wait_response: 是否等待响应（默认 True，用于调试）

        Returns:
            是否成功
        """
        if not self.connected:
            if self.debug:
                print("[Relay] 未连接")
            return False

        with self._lock:  # 使用锁保护串口访问
            try:
                # 构建请求帧
                data = bytes([
                    (coil_addr >> 8) & 0xFF,
                    coil_addr & 0xFF,
                    0xFF if value else 0x00,
                    0x00
                ])

                frame = bytes([self.device_id, 0x05]) + data
                crc = calculate_crc16(frame)
                frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF])

                if self.debug:
                    print(f"[Relay] 发送: {frame.hex()} (coil_addr={coil_addr}, value={value})")

                # 刷新 RX 缓冲区，清除可能残留的其他设备响应数据
                self.ser.reset_input_buffer()

                # 发送请求
                self.ser.write(frame)
                self.ser.flush()

                # 如果不需要等待响应，直接返回
                if not wait_response:
                    return True

                # 等待响应（仅当需要验证时）
                time.sleep(0.005)
                response = self.ser.read(8)

                if self.debug:
                    print(f"[Relay] 接收: {response.hex()} (长度={len(response)})")

                if len(response) != 8:
                    if self.debug:
                        print(f"[Relay] 响应长度不正确: 期望 8 字节, 实际 {len(response)} 字节")
                    return False

                # 验证 CRC
                received_crc = (response[-1] << 8) | response[-2]
                calculated_crc = calculate_crc16(response[:-2])

                return received_crc == calculated_crc
            except Exception as e:
                if self.debug:
                    print(f"[Relay] 写线圈错误: {e}")
                return False
    
    def _read_coils(self, start_addr, count):
        """
        读线圈状态（功能码 0x01）

        Args:
            start_addr: 起始线圈地址
            count: 读取数量

        Returns:
            线圈状态列表，失败返回 None
        """
        if not self.connected:
            return None

        with self._lock:  # 使用锁保护串口访问
            try:
                # 构建请求帧
                data = bytes([
                    (start_addr >> 8) & 0xFF,
                    start_addr & 0xFF,
                    (count >> 8) & 0xFF,
                    count & 0xFF
                ])

                frame = bytes([self.device_id, 0x01]) + data
                crc = calculate_crc16(frame)
                frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF])

                # 刷新 RX 缓冲区，清除可能残留的其他设备响应数据
                self.ser.reset_input_buffer()

                # 发送请求
                self.ser.write(frame)
                self.ser.flush()

                # 减少延迟，提高响应速度
                time.sleep(0.005)

                # 读取响应
                response = self.ser.read(5 + count)

                if len(response) < 5:
                    if self.debug:
                        print(f"[Relay] 响应长度不足: 期望至少 5 字节, 实际 {len(response)} 字节")
                    return None

                # 验证 CRC
                received_crc = (response[-1] << 8) | response[-2]
                calculated_crc = calculate_crc16(response[:-2])

                if received_crc != calculated_crc:
                    return None

                # 解析数据
                byte_count = response[2]
                states = []
                for i in range(byte_count):
                    byte_val = response[3 + i]
                    for j in range(8):
                        if len(states) < count:
                            states.append(bool(byte_val & (1 << j)))

                return states[:count]
            except Exception as e:
                if self.debug:
                    print(f"[Relay] 读线圈错误: {e}")
                return None

    def read_all_coils(self):
        """
        读取所有8个继电器通道的状态

        Returns:
            8个通道的状态列表，失败返回 None
        """
        return self._read_coils(0, 8)

    def set_left_arm(self, open_flag):
        """
        控制左侧展臂开合（通道1）
        
        Args:
            open_flag: True=展开，False=闭合
        
        Returns:
            是否成功
        """
        success = self._write_coil(0, 1 if open_flag else 0)
        if success:
            self.ch1_state = open_flag
            if self.debug:
                print(f"[Relay] 左侧展臂: {'展开' if open_flag else '闭合'}")
        return success
    
    def set_left_valve(self, open_flag):
        """
        控制左侧水阀开关（通道2）
        
        Args:
            open_flag: True=开启，False=关闭
        
        Returns:
            是否成功
        """
        success = self._write_coil(1, 1 if open_flag else 0)
        if success:
            self.ch2_state = open_flag
            if self.debug:
                print(f"[Relay] 左侧水阀: {'开启' if open_flag else '关闭'}")
        return success
    
    def set_right_arm(self, open_flag):
        """
        控制右侧展臂开合（通道3）
        
        Args:
            open_flag: True=展开，False=闭合
        
        Returns:
            是否成功
        """
        success = self._write_coil(2, 1 if open_flag else 0)
        if success:
            self.ch3_state = open_flag
            if self.debug:
                print(f"[Relay] 右侧展臂: {'展开' if open_flag else '闭合'}")
        return success
    
    def set_right_valve(self, open_flag):
        """
        控制右侧水阀开关（通道4）
        
        Args:
            open_flag: True=开启，False=关闭
        
        Returns:
            是否成功
        """
        success = self._write_coil(3, 1 if open_flag else 0)
        if success:
            self.ch4_state = open_flag
            if self.debug:
                print(f"[Relay] 右侧水阀: {'开启' if open_flag else '关闭'}")
        return success
    
    def set_alarm(self, on_flag):
        """
        控制报警器开关（通道5）

        Args:
            on_flag: True=开启，False=关闭

        Returns:
            是否成功
        """
        success = self._write_coil(4, 1 if on_flag else 0)
        if success:
            self.ch5_state = on_flag
            if self.debug:
                print(f"[Relay] 报警器: {'开启' if on_flag else '关闭'}")
        return success
    
    def set_pump(self, on_flag):
        """
        控制水泵喷水开关（通道6，同时负责水泵的开关）

        Args:
            on_flag: True=开启，False=关闭

        Returns:
            是否成功
        """
        success = self._write_coil(5, 1 if on_flag else 0)
        if success:
            self.ch6_state = on_flag
            if self.debug:
                print(f"[Relay] 水泵喷水: {'开启' if on_flag else '关闭'}")
        return success

    def raise_spray_mount(self):
        """
        升起喷水支架（通道7）

        Returns:
            是否成功
        """
        # 先关闭降下通道（互斥，无论当前状态）
        if self.debug:
            print("[Relay] 升起前先关闭降下通道（互斥）")
        success_close = self._write_coil(7, 0)
        self.ch8_state = False  # 无论成功与否都更新状态
        if not success_close and self.debug:
            print("[Relay] 关闭降下通道失败，继续尝试升起")

        # 打开升起通道
        success = self._write_coil(6, 1)
        if success:
            self.ch7_state = True
            if self.debug:
                print("[Relay] 升起喷水支架")
        return success

    def lower_spray_mount(self):
        """
        降下喷水支架（通道8）

        Returns:
            是否成功
        """
        # 先关闭升起通道（互斥，无论当前状态）
        if self.debug:
            print("[Relay] 下降前先关闭升起通道（互斥）")
        success_close = self._write_coil(6, 0)
        self.ch7_state = False  # 无论成功与否都更新状态
        if not success_close and self.debug:
            print("[Relay] 关闭升起通道失败，继续尝试下降")

        # 打开降下通道
        success = self._write_coil(7, 1)
        if success:
            self.ch8_state = True
            if self.debug:
                print("[Relay] 降下喷水支架")
        return success

    def stop_raise_mount(self):
        """
        停止升起喷水支架（关闭通道7）
        
        注意：限位保护场景下必须确保继电器关闭，不依赖本地状态变量判断

        Returns:
            是否成功
        """
        # 修复：限位保护场景下必须发送关闭命令，不依赖本地状态
        # 因为本地状态可能与实际继电器状态不同步
        
        success = self._write_coil(6, 0)
        if success:
            self.ch7_state = False
            if self.debug:
                print("[Relay] 停止升起喷水支架")
        else:
            # 即使失败也更新状态，强制重新发送命令
            if self.debug:
                print("[Relay] 停止升起喷水支架失败，强制重置状态")
            self.ch7_state = True  # 保持True，下次会重试
        return success

    def stop_lower_mount(self):
        """
        停止降下喷水支架（关闭通道8）
        
        注意：限位保护场景下必须确保继电器关闭，不依赖本地状态变量判断

        Returns:
            是否成功
        """
        # 修复：限位保护场景下必须发送关闭命令，不依赖本地状态
        # 因为本地状态可能与实际继电器状态不同步
        
        success = self._write_coil(7, 0)
        if success:
            self.ch8_state = False
            if self.debug:
                print("[Relay] 停止降下喷水支架")
        else:
            # 即使失败也更新状态，强制重新发送命令
            if self.debug:
                print("[Relay] 停止降下喷水支架失败，强制重置状态")
            self.ch8_state = True  # 保持True，下次会重试
        return success

    def start_spraying(self):
        """
        开始喷水（打开通道6：水泵喷水开关，同时负责水泵的开关）

        Returns:
            是否成功
        """
        if not self.connected:
            if self.debug:
                print("[Relay] 未连接")
            return False

        try:
            # 打开水泵喷水开关（通道6）
            success = self._write_coil(5, 1)
            if not success:
                if self.debug:
                    print("[Relay] 打开水泵喷水开关失败")
                return False

            self.ch6_state = True
            if self.debug:
                print("[Relay] 水泵喷水: ON，开始喷水")

            return True
        except Exception as e:
            if self.debug:
                print(f"[Relay] 开始喷水错误: {e}")
            return False
    
    def stop_spraying(self):
        """
        停止喷水（关闭通道6：水泵喷水开关，同时负责水泵的开关）

        Returns:
            是否成功
        """
        if not self.connected:
            if self.debug:
                print("[Relay] 未连接")
            return False

        try:
            # 关闭水泵喷水开关（通道6）
            success = self._write_coil(5, 0)
            if not success:
                if self.debug:
                    print("[Relay] 关闭水泵喷水开关失败")
                return False

            self.ch6_state = False
            if self.debug:
                print("[Relay] 水泵喷水: OFF，停止喷水")

            return True
        except Exception as e:
            if self.debug:
                print(f"[Relay] 停止喷水错误: {e}")
            return False
    
    def set_all_off(self):
        """
        关闭所有继电器通道
        
        Returns:
            是否成功
        """
        success = True
        for i in range(8):
            if not self._write_coil(i, 0):
                success = False
        
        if success:
            self.ch1_state = False
            self.ch2_state = False
            self.ch3_state = False
            self.ch4_state = False
            self.ch5_state = False
            self.ch6_state = False
            self.ch7_state = False
            self.ch8_state = False
            
            if self.debug:
                print("[Relay] 所有继电器: OFF")
        
        return success
    
    def emergency_stop(self):
        """
        紧急停止

        关闭：
        - 报警器（通道5）
        - 水泵喷水开关（通道6）
        - 升起喷水支架（通道7）
        - 降下喷水支架（通道8）

        其他开关保持当前状态不变

        Returns:
            是否成功
        """
        if not self.connected:
            if self.debug:
                print("[Relay] 未连接")
            return False

        try:
            success = True

            # 关闭报警器（通道5）
            if self.ch5_state:
                if not self._write_coil(4, 0):
                    success = False
                self.ch5_state = False

            # 关闭水泵喷水开关（通道6）
            if self.ch6_state:
                if not self._write_coil(5, 0):
                    success = False
                self.ch6_state = False

            # 关闭升起喷水支架（通道7）
            if self.ch7_state:
                if not self._write_coil(6, 0):
                    success = False
                self.ch7_state = False

            # 关闭降下喷水支架（通道8）
            if self.ch8_state:
                if not self._write_coil(7, 0):
                    success = False
                self.ch8_state = False

            if self.debug:
                print("[Relay] 紧急停止：已关闭报警器、水泵和支架控制")

            return success
        except Exception as e:
            if self.debug:
                print(f"[Relay] 紧急停止错误: {e}")
            return False
    
    def get_states(self):
        """
        获取所有继电器状态

        Returns:
            状态字典
        """
        return {
            'ch1': self.ch1_state,  # 左侧展臂
            'ch2': self.ch2_state,  # 左侧水阀
            'ch3': self.ch3_state,  # 右侧展臂
            'ch4': self.ch4_state,  # 右侧水阀
            'ch5': self.ch5_state,  # 报警器开关
            'ch6': self.ch6_state,  # 水泵喷水开关（同时负责水泵的开关）
            'ch7': self.ch7_state,  # 升起喷水支架
            'ch8': self.ch8_state   # 降下喷水支架
        }
    
    def get_status(self):
        """
        获取控制器状态
        
        Returns:
            状态字典
        """
        return {
            'connected': self.connected,
            'device_id': self.device_id,
            'baudrate': self.baudrate,
            'states': self.get_states()
        }
    
    def __del__(self):
        """析构函数，自动断开连接"""
        self.disconnect()
