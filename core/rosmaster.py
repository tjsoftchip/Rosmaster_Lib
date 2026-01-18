#!/usr/bin/env python3
# coding: utf-8

"""
Rosmaster 主类（重构版）

集成所有功能模块，提供统一的API接口
保持与原版 Rosmaster_Lib.py 的完全兼容性

作者：nx-ros2
日期：2026-01-16
"""

import time
import math
from .constants import *
from ..modules.mssd_controller import MSSDController
from ..modules.relay_controller import RelayController
from ..modules.water_sensor import WaterSensor
from ..modules.battery_monitor import BatteryMonitor
from ..modules.stm32_comm import STM32Communicator


class Rosmaster:
    """
    Rosmaster 机器人控制类（重构版）
    
    功能：
    - 运动控制
    - IMU 数据读取
    - 舵机控制
    - 机械臂控制
    - MSSD 电机控制（R2_MSSD 模式）
    - 继电器控制
    - 水位检测
    
    支持车型：
    - X3, X3 Plus, X1, R2, R2_MSSD
    """
    
    def __init__(self, car_type=5, com="/dev/myserial", delay=0.002, 
                 debug=False, mssd_port="/dev/mssd", relay_port="/dev/modbus",
                 enable_mssd=False):
        """
        初始化 Rosmaster
        
        Args:
            car_type: 车型类型（0x01=X3, 0x02=X3_PLUS, 0x04=X1, 0x05=R2, 0x06=R2_MSSD）
            com: STM32 串口路径（默认 /dev/myserial）
            delay: 指令延迟时间（秒）
            debug: 调试模式
            mssd_port: MSSD 串口路径（默认 /dev/mssd）
            relay_port: 继电器串口路径（默认 /dev/modbus）
            enable_mssd: 是否启用 MSSD 模式
        """
        self.debug = debug
        self.delay = delay
        
        # 兼容性处理：R2 车型默认启用 MSSD（根据原版代码）
        if car_type == 0x05:  # CARTYPE_R2
            self.CAR_TYPE = 0x05
            self.is_r2_mssd = True  # 默认启用 MSSD（原版逻辑）
            if debug:
                print("[Rosmaster] R2 mode with MSSD enabled (default)")
        elif car_type == 0x06:  # CARTYPE_R2_MSSD
            self.CAR_TYPE = 0x05  # STM32 兼容类型
            self.is_r2_mssd = True
            if debug:
                print("[Rosmaster] R2_MSSD mode enabled (explicit)")
        else:
            self.CAR_TYPE = car_type
            self.is_r2_mssd = enable_mssd
        
        # 初始化 STM32 通信模块
        self.stm32 = STM32Communicator(com, DEFAULT_BAUDRATE, delay, debug)
        
        # 初始化 MSSD 控制器（使用 /dev/mssd）
        self.mssd = None
        if self.is_r2_mssd:
            self.mssd = MSSDController("/dev/mssd", MSSD_DEVICE_ID, DEFAULT_BAUDRATE, debug)
        
        # 初始化继电器控制器（R2_MSSD 模式）
        self.relay = None
        self.water_sensor = None
        self.battery_monitor = None
        if self.is_r2_mssd:
            self.relay = RelayController(relay_port, RELAY_DEVICE_ID, 9600, debug)
            self.water_sensor = WaterSensor(self.relay, WATER_SENSOR_DEVICE_ID, debug)
            self.battery_monitor = BatteryMonitor(self.relay, BATTERY_MONITOR_DEVICE_ID, debug)
        
        # 功能码常量（兼容原版）
        self.FUNC_AUTO_REPORT = FUNC_AUTO_REPORT
        self.FUNC_BEEP = FUNC_BEEP
        self.FUNC_PWM_SERVO = FUNC_PWM_SERVO
        self.FUNC_PWM_SERVO_ALL = FUNC_PWM_SERVO_ALL
        self.FUNC_RGB = FUNC_RGB
        self.FUNC_RGB_EFFECT = FUNC_RGB_EFFECT
        self.FUNC_REPORT_SPEED = FUNC_REPORT_SPEED
        self.FUNC_REPORT_MPU_RAW = FUNC_REPORT_MPU_RAW
        self.FUNC_REPORT_IMU_ATT = FUNC_REPORT_IMU_ATT
        self.FUNC_REPORT_ENCODER = FUNC_REPORT_ENCODER
        self.FUNC_REPORT_ICM_RAW = FUNC_REPORT_ICM_RAW
        self.FUNC_RESET_STATE = FUNC_RESET_STATE
        self.FUNC_MOTOR = FUNC_MOTOR
        self.FUNC_CAR_RUN = FUNC_CAR_RUN
        self.FUNC_MOTION = FUNC_MOTION
        self.FUNC_SET_MOTOR_PID = FUNC_SET_MOTOR_PID
        self.FUNC_SET_YAW_PID = FUNC_SET_YAW_PID
        self.FUNC_SET_CAR_TYPE = FUNC_SET_CAR_TYPE
        self.FUNC_UART_SERVO = FUNC_UART_SERVO
        self.FUNC_UART_SERVO_ID = FUNC_UART_SERVO_ID
        self.FUNC_UART_SERVO_TORQUE = FUNC_UART_SERVO_TORQUE
        self.FUNC_ARM_CTRL = FUNC_ARM_CTRL
        self.FUNC_ARM_OFFSET = FUNC_ARM_OFFSET
        self.FUNC_AKM_DEF_ANGLE = FUNC_AKM_DEF_ANGLE
        self.FUNC_AKM_STEER_ANGLE = FUNC_AKM_STEER_ANGLE
        self.FUNC_LIMIT_SWITCH = FUNC_LIMIT_SWITCH
        self.FUNC_REQUEST_DATA = FUNC_REQUEST_DATA
        self.FUNC_VERSION = FUNC_VERSION
        self.FUNC_RESET_FLASH = FUNC_RESET_FLASH
        
        # 车型常量（兼容原版）
        self.CARTYPE_X3 = CARTYPE_X3
        self.CARTYPE_X3_PLUS = CARTYPE_X3_PLUS
        self.CARTYPE_X1 = CARTYPE_X1
        self.CARTYPE_R2 = CARTYPE_R2
        self.CARTYPE_R2_MSSD = CARTYPE_R2_MSSD
        
        # 物理参数
        self.__wheelbase = WHEELBASE_R2
        self.__track_width = TRACK_WIDTH_R2
        
        # 限位保护相关变量
        self.__limit_protection_enabled = False  # 限位保护开关
        self.__limit_protection_mode = None  # 保护模式: 'raise' 或 'lower'
        self.__limit_protection_callback = None  # 限位触发回调函数
        
        # 注册限位开关状态变化回调到 STM32
        if self.stm32:
            self.stm32.set_limit_switch_callback(self._handle_limit_switch_change)
        
        if debug:
            print(f"[Rosmaster] 初始化完成: 车型={CAR_TYPE_NAMES.get(self.CAR_TYPE, 'UNKNOWN')} ({self.CAR_TYPE})")
    
    def create_receive_threading(self):
        """创建接收线程"""
        self.stm32.start_receive_thread()
    
    def set_car_run(self, state, speed):
        """
        设置小车运动状态
        
        Args:
            state: 运动状态（0=停止, 1=前进, 2=后退, 3=左转, 4=右转, 5=左自旋, 6=右自旋）
            speed: 速度（0-100）
        """
        # R2_MSSD 模式：通过 MSSD 控制速度
        if self.is_r2_mssd and self.mssd:
            if state == 0:  # 停止
                self.mssd.stop()
            elif state == 1:  # 前进
                target_rpm = int(speed * 36)  # 转换为 RPM
                self.mssd.set_speed(target_rpm)
            elif state == 2:  # 后退
                target_rpm = -int(speed * 36)
                self.mssd.set_speed(target_rpm)
            # 其他状态暂不支持
        else:
            # 原有模式：通过 STM32 控制
            data = bytes([state, speed, 0, 0])
            self.stm32.send_command(self.FUNC_CAR_RUN, data)
    
    def set_motor(self, speed_1, speed_2, speed_3, speed_4):
        """
        控制电机速度
        
        R2_MSSD 模式：
        - 使用变化检测，仅在速度变化时发送 MSSD 指令
        - MSSD 自动保持转速，无需持续发送
        - 完全兼容原有 API
        
        原有模式：
        - 保持原有逻辑，持续发送 PWM 指令
        """
        if self.is_r2_mssd:
            # R2_MSSD 模式
            # 计算目标速度
            motor_speed = (speed_1 + speed_2) / 2.0
            motor_speed = max(-100, min(100, motor_speed))
            
            # 变化检测
            if abs(motor_speed - self.mssd.cached_motor_speed) > self.mssd.speed_change_threshold:
                # 速度变化，发送 MSSD 指令
                target_rpm = int(motor_speed * 36)
                
                if self.mssd.set_speed(target_rpm):
                    self.mssd.cached_motor_speed = motor_speed
                    self.mssd.last_command_time = time.time()
                    
                    if self.debug:
                        print(f"[Rosmaster] MSSD speed: {motor_speed} -> {target_rpm} RPM")
                else:
                    if self.debug:
                        print(f"[Rosmaster] MSSD set speed failed")
            else:
                # 速度未变化，MSSD 自动保持
                if self.debug:
                    print(f"[Rosmaster] MSSD speed unchanged: {motor_speed}")
        else:
            # 原有 STM32 逻辑（保持不变）
            import struct
            try:
                t_speed_a = bytearray(struct.pack('b', self._limit_motor_value(speed_1)))
                t_speed_b = bytearray(struct.pack('b', self._limit_motor_value(speed_2)))
                t_speed_c = bytearray(struct.pack('b', self._limit_motor_value(speed_3)))
                t_speed_d = bytearray(struct.pack('b', self._limit_motor_value(speed_4)))
                
                cmd = [HEAD, DEVICE_ID, 0x00, FUNC_MOTOR,
                       t_speed_a[0], t_speed_b[0], t_speed_c[0], t_speed_d[0]]
                cmd[2] = len(cmd) - 1
                checksum = sum(cmd, COMPLEMENT) & 0xff
                cmd.append(checksum)
                
                self.stm32.ser.write(cmd)
                
                if self.debug:
                    print(f"[Rosmaster] motor: {cmd}")
                
                time.sleep(self.delay)
            except Exception as e:
                if self.debug:
                    print(f'[Rosmaster] set_motor error: {e}')
    
    def _limit_motor_value(self, value):
        """限制电机速度值在 -100 到 100 之间"""
        return max(-100, min(100, int(value)))
    
    def set_akm_steering_angle(self, angle, ctrl_car=False):
        """
        控制阿克曼转向角度
        
        R2_MSSD 模式：
        - angle: 目标转向角度（-45~45 度）
        - 自动应用 3:1 减速器补偿
        
        原有模式：
        - 保持原有逻辑不变
        """
        if self.is_r2_mssd:
            # 应用 3:1 减速器补偿
            compensated_angle = self._compensate_steer_angle(angle)
            self.mssd.last_steer_angle = angle
            
            if self.debug:
                print(f"[Rosmaster] Steer angle: {angle}° -> {compensated_angle}° (3:1 ratio)")
        
        # 调用原有逻辑（STM32 控制舵机）
        try:
            if int(angle) > 45 or int(angle) < -45:
                return
            
            id = self.stm32.AKM_SERVO_ID
            if ctrl_car:
                id = self.stm32.AKM_SERVO_ID + 0x80
            
            cmd = [HEAD, DEVICE_ID, 0x00, FUNC_AKM_STEER_ANGLE, id, int(angle)&0xFF]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, COMPLEMENT) & 0xff
            cmd.append(checksum)
            
            self.stm32.ser.write(cmd)
            
            if self.debug:
                print(f"[Rosmaster] akm_steering_angle: {cmd}")
            
            time.sleep(self.delay)
        except Exception as e:
            if self.debug:
                print(f'[Rosmaster] set_akm_steering_angle error: {e}')
    
    def _compensate_steer_angle(self, angle):
        """应用 3:1 减速器补偿"""
        return int(angle * 3.0)
    
    def set_car_motion(self, v_x, v_y, v_z):
        """
        小车运动控制（三自由度）
        
        输入范围 input range: 
        X3: v_x=[-1.0, 1.0], v_y=[-1.0, 1.0], v_z=[-5, 5]
        X3PLUS: v_x=[-0.7, 0.7], v_y=[-0.7, 0.7], v_z=[-3.2, 3.2]
        R2/R2L: v_x=[-1.8, 1.8], v_y=[-0.045, 0.045], v_z=[-3, 3]
        R2_MSSD: v_x=[-1.8, 1.8], v_y=[-0.045, 0.045], v_z=[-3, 3] (大车底盘)
        
        Args:
            v_x: 线速度（m/s）
            v_y: 侧向速度（m/s，R2 车型忽略）
            v_z: 角速度（rad/s）
        """
        if self.is_r2_mssd:
            # R2_MSSD 模式（大车底盘）
            # v_x 控制电机速度
            motor_speed = int(v_x * 100)
            self.set_motor(motor_speed, motor_speed, 0, 0)
            
            # v_z 控制转向角度
            # 大车底盘：最大旋转速度 3 rad/s = 45 度转向
            steer_angle = int(v_z * 15)
            self.set_akm_steering_angle(steer_angle)
            
            if self.debug:
                print(f"[Rosmaster] set_car_motion: vx={v_x:.3f}, vz={v_z:.3f} -> motor={motor_speed}, steer={steer_angle}°")
        else:
            # 原有模式：发送 FUNC_MOTION 指令到 STM32
            import struct
            try:
                vx_parms = bytearray(struct.pack('h', int(v_x * 1000)))
                vy_parms = bytearray(struct.pack('h', int(v_y * 1000)))
                vz_parms = bytearray(struct.pack('h', int(v_z * 1000)))
                
                cmd = [HEAD, DEVICE_ID, 0x00, FUNC_MOTION, self.CAR_TYPE,
                       vx_parms[0], vx_parms[1], vy_parms[0], vy_parms[1], vz_parms[0], vz_parms[1]]
                cmd[2] = len(cmd) - 1
                
                # 计算校验和
                checksum = sum(cmd, COMPLEMENT) & 0xff
                cmd.append(checksum)
                
                self.stm32.ser.write(cmd)
                
                if self.debug:
                    print(f"[Rosmaster] set_car_motion: vx={v_x:.3f}, vy={v_y:.3f}, vz={v_z:.3f}")
                
                time.sleep(self.delay)
            except Exception as e:
                if self.debug:
                    print(f'[Rosmaster] set_car_motion error: {e}')
    
    def get_motion_data(self):
        """
        获取运动数据
        
        Returns:
            (vx, vy, vz) - 线速度、转向角速度、角速度
        """
        if self.is_r2_mssd and self.mssd and self.mssd.connected:
            # R2_MSSD 模式：从 MSSD 读取速度数据
            import math
            import time
            
            # 定期更新 MSSD 速度数据（每 100ms 更新一次）
            current_time = time.time()
            if current_time - self.mssd.last_update_time > 0.1:
                self.mssd.speed = self.mssd.get_speed()
                self.mssd.last_update_time = current_time
                
                # 转换 RPM 到线速度（m/s）
                # 减速比：41:1，轮胎直径：650mm
                # 线速度 = (电机转速 / 60 / 减速比) × π × 轮胎直径
                self.mssd.linear_velocity = (self.mssd.speed / 60.0 / 41.0) * math.pi * 0.65
            
            # vx: 线速度（来自 MSSD）
            val_vx = self.mssd.linear_velocity
            
            # vy: 转向角度（来自 STM32，单位：度）
            val_vy = self.stm32.vy
            
            # vz: 角速度（根据转向角度和线速度计算）
            # 角速度 = 线速度 / 转弯半径
            # 转弯半径 = 轴距 / tan(转向角度)
            if abs(val_vy) > 0.5:  # 转向角度大于 0.5 度
                theta = math.radians(val_vy)
                R = self.__wheelbase / math.tan(theta)
                val_vz = val_vx / R
            else:
                val_vz = 0  # 直线行驶，角速度为 0
            
            if self.debug:
                print(f"[Rosmaster] MSSD Motion: speed={self.mssd.speed} RPM, vx={val_vx:.3f} m/s, vy={val_vy:.1f}°, vz={val_vz:.3f} rad/s")
            
            return val_vx, val_vy, val_vz
        else:
            # 原有模式：使用 STM32 数据
            val_vx = self.stm32.vx
            val_vy = self.stm32.vy
            val_vz = self.stm32.vz
            
            if self.debug:
                print(f"[Rosmaster] STM32 Motion: vx={val_vx:.3f}, vy={val_vy:.3f}, vz={val_vz:.3f}")
            
            return val_vx, val_vy, val_vz
    
    def get_imu_attitude(self):
        """
        获取 IMU 姿态数据
        
        Returns:
            (roll, pitch, yaw) - 横滚角、俯仰角、偏航角
        """
        return self.stm32.get_imu_attitude()
    
    def get_accelerometer_data(self):
        """
        获取加速度计三轴数据
        
        Returns:
            (ax, ay, az) - 三轴加速度（m/s²）
        """
        return self.stm32.ax, self.stm32.ay, self.stm32.az
    
    def get_gyroscope_data(self):
        """
        获取陀螺仪三轴数据
        
        Returns:
            (gx, gy, gz) - 三轴角速度（rad/s）
        """
        return self.stm32.gx, self.stm32.gy, self.stm32.gz
    
    def get_magnetometer_data(self):
        """
        获取磁力计三轴数据
        
        Returns:
            (mx, my, mz) - 三轴磁场强度
        """
        return self.stm32.mx, self.stm32.my, self.stm32.mz
    
    def get_motor_encoder(self):
        """
        获取四路电机编码器数据
        
        R2_MSSD 模式：
        - m1: 后左轮（通过差速器计算）
        - m2: 后右轮（通过差速器计算）
        - m3: 0（前轮无编码器）
        - m4: 0（前轮无编码器）
        
        原有模式：
        - 保持原有逻辑不变
        """
        if self.is_r2_mssd:
            if self.mssd and self.mssd.connected:
                # 从 MSSD 读取编码器（寄存器 22-23）
                values = self.mssd.read_registers(22, 2)
                if values and len(values) >= 2:
                    encoder_high = values[0]
                    encoder_low = values[1]
                    motor_encoder = (encoder_high << 16) | encoder_low
                    
                    # 处理负数
                    if motor_encoder & 0x80000000:
                        motor_encoder = motor_encoder - 0x100000000
                    
                    # 根据阿克曼几何计算左右轮编码器
                    m1, m2 = self.mssd.calculate_differential_encoder(
                        motor_encoder, 
                        self.mssd.last_steer_angle
                    )
                    
                    m3 = 0
                    m4 = 0
                    
                    if self.debug:
                        print(f"[Rosmaster] Encoder: motor={motor_encoder}, left={m1}, right={m2}")
                    
                    return m1, m2, m3, m4
                else:
                    return 0, 0, 0, 0
            else:
                return 0, 0, 0, 0
        else:
            # 原有逻辑（保持不变）
            m1, m2, m3, m4 = self.stm32.encoder_m1, self.stm32.encoder_m2, self.stm32.encoder_m3, self.stm32.encoder_m4
            return m1, m2, m3, m4
    
    def get_battery_voltage(self):
        """
        获取电池电压
        
        Returns:
            电池电压（V）
        """
        return self.stm32.get_battery_voltage()
    
    def get_limit_switch_state(self):
        """
        获取限位开关状态
        
        Returns:
            限位开关状态（0=都未触发, 1=上限触发, 2=下限触发, 3=都触发）
        """
        # 主动查询限位开关状态
        self.stm32.send_command(self.FUNC_REQUEST_DATA, bytes([self.FUNC_LIMIT_SWITCH, 0x00]))
        time.sleep(0.05)  # 等待响应
        return self.stm32.get_limit_switch_state()
    
    # ========== MSSD 相关方法 ==========
    
    def get_mssd_speed(self):
        """
        获取 MSSD 电机速度
        
        Returns:
            电机速度（RPM）
        """
        if self.mssd:
            return self.mssd.get_speed()
        return 0
    
    def get_mssd_cached_speed(self):
        """
        获取 MSSD 缓存速度
        
        Returns:
            缓存的速度（RPM）
        """
        if self.mssd:
            return self.mssd.get_cached_speed()
        return 0
    
    def get_mssd_status(self):
        """
        获取 MSSD 状态
        
        Returns:
            MSSD 状态字典
        """
        if self.mssd:
            return self.mssd.get_status()
        return {'connected': False}
    
    # ========== 继电器相关方法 ==========
    
    def set_relay_left_arm(self, open_flag):
        """控制左侧展臂"""
        if self.relay:
            return self.relay.set_left_arm(open_flag)
        return False
    
    def set_relay_left_valve(self, open_flag):
        """控制左侧水阀"""
        if self.relay:
            return self.relay.set_left_valve(open_flag)
        return False
    
    def set_relay_right_arm(self, open_flag):
        """控制右侧展臂"""
        if self.relay:
            return self.relay.set_right_arm(open_flag)
        return False
    
    def set_relay_right_valve(self, open_flag):
        """控制右侧水阀"""
        if self.relay:
            return self.relay.set_right_valve(open_flag)
        return False
    
    def set_relay_pump(self, on_flag):
        """控制水泵"""
        if self.relay:
            return self.relay.set_pump(on_flag)
        return False
    
    def start_spray(self):
        """开始喷水"""
        if self.relay:
            return self.relay.start_spraying()
        return False
    
    def stop_spray(self):
        """停止喷水"""
        if self.relay:
            return self.relay.stop_spraying()
        return False
    
    def _handle_limit_switch_change(self, old_state, new_state):
        """
        处理限位开关状态变化（内部方法）

        Args:
            old_state: 旧的状态
            new_state: 新的状态
        """
        if not self.__limit_protection_enabled:
            return

        if self.debug:
            print(f"[Rosmaster] 限位开关状态变化: {old_state} -> {new_state}, 模式={self.__limit_protection_mode}")

        # 根据保护模式处理
        if self.__limit_protection_mode == 'raise':
            # 监控上限位：检测到 0x01 或 0x03 时停止
            if new_state == 0x01 or new_state == 0x03:
                if self.debug:
                    print("[Rosmaster] 上限位触发，停止提升支架")
                # 只关闭提升继电器（通道7）
                if self.relay:
                    self.relay.stop_raise_mount()
                # 关闭保护
                self._disable_limit_protection()
                # 调用回调函数
                if self.__limit_protection_callback:
                    self.__limit_protection_callback('raise', new_state)

        elif self.__limit_protection_mode == 'lower':
            # 监控下限位：检测到 0x02 或 0x03 时停止
            if new_state == 0x02 or new_state == 0x03:
                if self.debug:
                    print("[Rosmaster] 下限位触发，停止下降支架")
                # 只关闭下降继电器（通道8）
                if self.relay:
                    self.relay.stop_lower_mount()
                # 关闭保护
                self._disable_limit_protection()
                # 调用回调函数
                if self.__limit_protection_callback:
                    self.__limit_protection_callback('lower', new_state)

    def _enable_limit_protection(self, mode, callback=None):
        """
        启用限位保护（内部方法）

        Args:
            mode: 保护模式 ('raise' 或 'lower')
            callback: 限位触发回调函数
        """
        self.__limit_protection_enabled = True
        self.__limit_protection_mode = mode
        self.__limit_protection_callback = callback
        if self.debug:
            print(f"[Rosmaster] 限位保护已启用: 模式={mode}")

    def _disable_limit_protection(self):
        """禁用限位保护（内部方法）"""
        self.__limit_protection_enabled = False
        self.__limit_protection_mode = None
        self.__limit_protection_callback = None
        if self.debug:
            print("[Rosmaster] 限位保护已禁用")
    
    def raise_spray_mount(self):
        """
        升起喷水支架（带限位开关保护）

        Returns:
            是否成功
        """
        if not self.relay:
            return False

        try:
            # 步骤1: 查询限位开关状态
            limit_state = self.get_limit_switch_state()

            # 步骤2: 检查上限位是否已触发（主动保护）
            if limit_state == 0x01 or limit_state == 0x03:
                if self.debug:
                    print(f"[Rosmaster] 无法提升支架: 上限位已触发 (状态={limit_state})")
                return False

            # 步骤3: 调用继电器控制器升起支架
            success = self.relay.raise_spray_mount()
            
            if success:
                # 步骤4: 启用限位保护（被动保护，事件驱动）
                self._enable_limit_protection('raise')
            
            return success
        except Exception as e:
            if self.debug:
                print(f"[Rosmaster] 升起喷水支架错误: {e}")
            return False
    
    def lower_spray_mount(self):
        """
        降下喷水支架（带限位开关保护）

        Returns:
            是否成功
        """
        if not self.relay:
            return False

        try:
            # 步骤1: 查询限位开关状态
            limit_state = self.get_limit_switch_state()

            # 步骤2: 检查下限位是否已触发（主动保护）
            if limit_state == 0x02 or limit_state == 0x03:
                if self.debug:
                    print(f"[Rosmaster] 无法降下支架: 下限位已触发 (状态={limit_state})")
                return False

            # 步骤3: 调用继电器控制器降下支架
            success = self.relay.lower_spray_mount()
            
            if success:
                # 步骤4: 启用限位保护（被动保护，事件驱动）
                self._enable_limit_protection('lower')
            
            return success
        except Exception as e:
            if self.debug:
                print(f"[Rosmaster] 降下喷水支架错误: {e}")
            return False
    
    def set_relay_all_off(self):
        """关闭所有继电器"""
        if self.relay:
            return self.relay.set_all_off()
        return False
    
    def emergency_stop(self):
        """
        紧急停止
        
        关闭：
        - 水泵电源
        - 水泵喷水开关
        - 升起喷水支架
        - 降下喷水支架
        
        其他开关保持当前状态不变
        
        Returns:
            是否成功
        """
        if self.relay:
            return self.relay.emergency_stop()
        return False
    
    def get_relay_states(self):
        """获取所有继电器状态"""
        if self.relay:
            return self.relay.get_states()
        return {}
    
    def get_relay_status(self):
        """获取继电器状态"""
        if self.relay:
            return self.relay.get_status()
        return {'connected': False}
    
    # ========== 水位检测相关方法 ==========
    
    def get_water_level(self):
        """
        获取水位高度
        
        Returns:
            水位高度（米），失败返回 None
        """
        if self.water_sensor:
            return self.water_sensor.get_water_level()
        return None
    
    def check_water_level(self, min_level=0.2, max_level=0.9):
        """
        检查水位状态
        
        Returns:
            状态字典
        """
        if self.water_sensor:
            return self.water_sensor.check_water_level(min_level, max_level)
        return {'level': None, 'percentage': None, 'status': 'error', 'message': '未初始化'}
    
    def is_water_sufficient(self, min_level=0.3):
        """
        检查水位是否充足
        
        Returns:
            True=充足，False=不足
        """
        if self.water_sensor:
            return self.water_sensor.is_water_sufficient(min_level)
        return False
    
    # ========== 电池监控相关方法 ==========
    
    def get_battery_percentage(self):
        """
        获取电池容量百分比（对外API）
        
        Returns:
            容量百分比（0-100），失败返回 None
            
        Note:
            此方法会连续读取5次数据取平均值
        """
        if self.battery_monitor:
            return self.battery_monitor.get_battery_percentage()
        return None
    
    def get_battery_voltage(self):
        """
        获取电池电压
        
        Returns:
            电压值（V），失败返回 None
            
        Note:
            - R2_MSSD 模式：从电池库仑计读取
            - 其他模式：从 STM32 读取
        """
        if self.battery_monitor:
            return self.battery_monitor.get_battery_voltage()
        else:
            # 回退到 STM32 电池电压读取
            return self.stm32.get_battery_voltage()
    
    def get_battery_current(self):
        """
        获取电池电流
        
        Returns:
            电流值（A），失败返回 None
        """
        if self.battery_monitor:
            return self.battery_monitor.get_battery_current()
        return None
    
    def get_battery_power(self):
        """
        获取电池功率
        
        Returns:
            功率值（W），失败返回 None
        """
        if self.battery_monitor:
            return self.battery_monitor.get_battery_power()
        return None
    
    def get_battery_temperature(self):
        """
        获取电池温度
        
        Returns:
            温度值（℃），失败返回 None
        """
        if self.battery_monitor:
            return self.battery_monitor.get_battery_temperature()
        return None
    
    def get_battery_capacity_remaining(self):
        """
        获取剩余容量
        
        Returns:
            剩余容量（mAh），失败返回 None
        """
        if self.battery_monitor:
            return self.battery_monitor.get_battery_capacity_remaining()
        return None
    
    def get_battery_energy_accumulated(self):
        """
        获取累计电能
        
        Returns:
            累计电能（Wh），失败返回 None
        """
        if self.battery_monitor:
            return self.battery_monitor.get_battery_energy_accumulated()
        return None
    
    def get_battery_alarm_status(self):
        """
        获取报警状态
        
        Returns:
            报警状态字典，失败返回 None
        """
        if self.battery_monitor:
            return self.battery_monitor.get_alarm_status()
        return None
    
    def get_charge_discharge_status(self):
        """
        获取充放电状态
        
        Returns:
            状态字符串（'idle', 'discharge', 'charge'），失败返回 None
        """
        if self.battery_monitor:
            return self.battery_monitor.get_charge_discharge_status()
        return None
    
    def get_battery_status(self):
        """
        获取电池完整状态（对外API）
        
        Returns:
            状态字典，包含所有电池参数，失败返回 None
        """
        if self.battery_monitor:
            return self.battery_monitor.get_battery_status()
        return None
    
    def is_battery_low(self, threshold=20):
        """
        检查电池电量是否过低
        
        Args:
            threshold: 电量阈值（%），默认20%
        
        Returns:
            True=电量过低，False=电量正常，None=无法读取
        """
        if self.battery_monitor:
            return self.battery_monitor.is_battery_low(threshold)
        return None
    
    def is_battery_sufficient(self, threshold=30):
        """
        检查电池电量是否充足
        
        Args:
            threshold: 电量阈值（%），默认30%
        
        Returns:
            True=电量充足，False=电量不足，None=无法读取
        """
        if self.battery_monitor:
            return self.battery_monitor.is_battery_sufficient(threshold)
        return None
    
    # ========== 其他方法（兼容原版）==========
    
    def set_beep(self, on_time):
        """设置蜂鸣器"""
        data = bytes([on_time, 0])
        return self.stm32.send_command(self.FUNC_BEEP, data)
    
    def set_pwm_servo(self, servo_id, angle):
        """设置 PWM 舵机"""
        data = bytes([servo_id, angle, 0, 0])
        return self.stm32.send_command(self.FUNC_PWM_SERVO, data)
    
    def set_pwm_servo_all(self, angle_s1, angle_s2, angle_s3, angle_s4):
        """设置所有 PWM 舵机"""
        data = bytes([angle_s1, angle_s2, angle_s3, angle_s4])
        return self.stm32.send_command(self.FUNC_PWM_SERVO_ALL, data)
    
    def set_uart_servo(self, servo_id, pulse_value, run_time=500):
        """设置总线舵机"""
        data = bytes([
            servo_id,
            (pulse_value >> 8) & 0xFF,
            pulse_value & 0xFF,
            (run_time >> 8) & 0xFF,
            run_time & 0xFF,
            0, 0
        ])
        return self.stm32.send_command(self.FUNC_UART_SERVO, data)
    
    def set_uart_servo_torque(self, enable):
        """设置舵机扭矩"""
        data = bytes([1 if enable else 0, 0])
        return self.stm32.send_command(self.FUNC_UART_SERVO_TORQUE, data)
    
    def set_car_type(self, car_type):
        """设置车型"""
        data = bytes([car_type, 0])
        return self.stm32.send_command(self.FUNC_SET_CAR_TYPE, data)
    
    def set_auto_report_state(self, enable, forever=False):
        """设置自动上报状态"""
        data = bytes([1 if enable else 0, 1 if forever else 0])
        return self.stm32.send_command(self.FUNC_AUTO_REPORT, data)
    
    def get_version(self):
        """
        获取底层单片机版本号
        
        Returns:
            版本号（如 1.1），失败返回 -1
        """
        if self.stm32.version_H == 0:
            # 请求数据
            self.stm32.send_command(self.FUNC_REQUEST_DATA, 
                                  bytes([self.FUNC_VERSION, 0]))
            
            # 等待响应
            for i in range(20):
                if self.stm32.version_H != 0:
                    version = self.stm32.version_H * 1.0 + self.stm32.version_L / 10.0
                    if self.debug:
                        print(f"[Rosmaster] get_version: V{version}, i={i}")
                    return version
                time.sleep(0.001)
        else:
            return self.stm32.version_H * 1.0 + self.stm32.version_L / 10.0
        
        return -1
    
    def set_colorful_effect(self, effect, speed=255, parm=255):
        """
        设置 RGB 灯带特效
        
        Args:
            effect: 特效类型（0=停止, 1=流水灯, 2=跑马灯, 3=呼吸灯, 4=渐变灯, 5=星光点点, 6=电量显示）
            speed: 速度（1-10，数值越小速度越快）
            parm: 附加参数（可选）
        """
        data = bytes([effect & 0xFF, speed & 0xFF, parm & 0xFF])
        return self.stm32.send_command(self.FUNC_RGB_EFFECT, data)
    
    def __del__(self):
        """析构函数"""
        if self.mssd:
            self.mssd.disconnect()
        if self.relay:
            self.relay.disconnect()
        if self.stm32:
            self.stm32.disconnect()
