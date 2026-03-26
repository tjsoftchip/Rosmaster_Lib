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
import threading
from .constants import *
from ..modules.mssd_controller import MSSDController
from ..modules.relay_controller import RelayController
from ..modules.water_sensor import WaterSensor
from ..modules.battery_monitor import BatteryMonitor
from ..modules.stm32_comm import STM32Communicator
from ..modules.data_synchronizer import DataSynchronizer


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
    - X3, X3 Plus, X1, R2 (仅 STM32，无继电器), R2_MSSD (STM32 + MSSD + 继电器)
    """

    def __init__(
        self,
        car_type=5,
        com="/dev/myserial",
        delay=0.002,
        debug=False,
        mssd_port="/dev/mssd",
        relay_port="/dev/modbus",
        enable_mssd=False,
    ):
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

        # 兼容性处理：R2 和 R2_MSSD 模式都启用完整功能（MSSD + 继电器 + 水位 + 电量）
        # 通过单例模式确保所有节点共享同一个实例，避免串口冲突
        if car_type == 0x05:  # CARTYPE_R2
            self.CAR_TYPE = 0x05
            self.is_r2_mssd = True  # R2 模式也启用完整功能
            if debug:
                print(
                    "[Rosmaster] R2 mode with full features (MSSD + Relay + Water + Battery)"
                )
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
            self.mssd = MSSDController(
                "/dev/mssd", MSSD_DEVICE_ID, DEFAULT_BAUDRATE, debug
            )

        # 初始化继电器控制器（R2_MSSD 模式）
        self.relay = None
        self.water_sensor = None
        self.battery_monitor = None
        if self.is_r2_mssd:
            self.relay = RelayController(relay_port, RELAY_DEVICE_ID, 9600, debug)
            self.water_sensor = WaterSensor(self.relay, WATER_SENSOR_DEVICE_ID, debug)
            self.battery_monitor = BatteryMonitor(
                self.relay, BATTERY_MONITOR_DEVICE_ID, debug
            )

        # 初始化数据同步器（统一数据更新频率和时间戳对齐）
        self.data_sync = DataSynchronizer(
            update_interval=0.04, expire_time=0.1, debug=debug
        )

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
        self.__limit_protection_mode = (
            None  # 保护模式: 'raise', 'lower', 'raised', 'lowered', 'idle'
        )
        self.__limit_protection_callback = None  # 限位触发回调函数

        # 支架状态：'idle', 'raising', 'raised', 'lowering', 'lowered'
        self.__mount_state = "idle"

        # 状态保持检查相关变量
        self.__last_state_check_time = time.time()  # 上次状态检查时间
        self.__state_check_interval = 30.0  # 状态检查间隔（秒）

        # 注册限位开关状态变化回调到 STM32
        if self.stm32:
            self.stm32.set_limit_switch_callback(self._handle_limit_switch_change)
            if debug:
                print("[Rosmaster] 限位开关回调已注册")

        if debug:
            print(
                f"[Rosmaster] 初始化完成: 车型={CAR_TYPE_NAMES.get(self.CAR_TYPE, 'UNKNOWN')} ({self.CAR_TYPE})"
            )

    def create_receive_threading(self):
        """创建接收线程"""
        self.stm32.start_receive_thread()

        # 启动数据同步器
        if self.is_r2_mssd:
            self.data_sync.start()

            # 启动数据更新定时器（每10ms更新一次数据到同步器）
            self._data_update_timer = threading.Timer(0.01, self._data_update_loop)
            self._data_update_timer.daemon = True
            self._data_update_timer.start()

            if self.debug:
                print("[Rosmaster] 数据同步器已启动")

    def _data_update_loop(self):
        """
        数据更新循环（定时器回调）

        定期从传感器读取数据并更新到DataSynchronizer
        同时检查支架状态保持（每30秒）
        """
        if self.is_r2_mssd:
            self._update_data_sync()

            # 检查支架状态保持（内部有30秒间隔判断）
            self._check_state_maintenance()

            # 重新调度定时器
            self._data_update_timer = threading.Timer(0.01, self._data_update_loop)
            self._data_update_timer.daemon = True
            self._data_update_timer.start()

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
            if (
                abs(motor_speed - self.mssd.cached_motor_speed)
                > self.mssd.speed_change_threshold
            ):
                # 速度变化，发送 MSSD 指令
                # 修复：系数从36改为14，使实际速度与设置速度匹配
                # motor_speed * 14 = v_x * 100 * 14 = v_x * 1400 RPM
                target_rpm = int(motor_speed * 14)

                if self.mssd.set_speed(target_rpm):
                    self.mssd.cached_motor_speed = motor_speed
                    self.mssd.last_command_time = time.time()

                    if self.debug:
                        print(
                            f"[Rosmaster] MSSD speed: {motor_speed} -> {target_rpm} RPM"
                        )
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
                t_speed_a = bytearray(
                    struct.pack("b", self._limit_motor_value(speed_1))
                )
                t_speed_b = bytearray(
                    struct.pack("b", self._limit_motor_value(speed_2))
                )
                t_speed_c = bytearray(
                    struct.pack("b", self._limit_motor_value(speed_3))
                )
                t_speed_d = bytearray(
                    struct.pack("b", self._limit_motor_value(speed_4))
                )

                cmd = [
                    HEAD,
                    DEVICE_ID,
                    0x00,
                    FUNC_MOTOR,
                    t_speed_a[0],
                    t_speed_b[0],
                    t_speed_c[0],
                    t_speed_d[0],
                ]
                cmd[2] = len(cmd) - 1
                checksum = sum(cmd, COMPLEMENT) & 0xFF
                cmd.append(checksum)

                self.stm32.ser.write(cmd)

                if self.debug:
                    print(f"[Rosmaster] motor: {cmd}")

                time.sleep(self.delay)
            except Exception as e:
                if self.debug:
                    print(f"[Rosmaster] set_motor error: {e}")

    def _limit_motor_value(self, value):
        """限制电机速度值在 -100 到 100 之间"""
        return max(-100, min(100, int(value)))

    def set_akm_steering_angle(self, angle, ctrl_car=False):
        """
        控制阿克曼转向角度

        Args:
            angle: 转向角度（-34~34 度）
            ctrl_car: 是否同时控制电机速度

        Returns:
            是否成功
        """
        # 调用原有逻辑（STM32 控制舵机）
        try:
            if int(angle) > 34 or int(angle) < -34:
                return

            id = self.stm32.AKM_SERVO_ID
            if ctrl_car:
                id = self.stm32.AKM_SERVO_ID + 0x80

            cmd = [HEAD, DEVICE_ID, 0x00, FUNC_AKM_STEER_ANGLE, id, int(angle) & 0xFF]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, COMPLEMENT) & 0xFF
            cmd.append(checksum)

            self.stm32.ser.write(cmd)

            if self.debug:
                print(f"[Rosmaster] akm_steering_angle: {cmd}")

            time.sleep(self.delay)
        except Exception as e:
            if self.debug:
                print(f"[Rosmaster] set_akm_steering_angle error: {e}")

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
            v_z: 转向控制值（-1~1 映射到 -34°~34°）
        """
        if self.is_r2_mssd:
            # R2_MSSD 模式（大车底盘）
            #
            # ============================================================
            # 1. 电机速度计算（修复速度放大问题）
            # ============================================================
            # 物理参数：
            # - 轮径 = 0.56m
            # - 减速比 = 41:1
            # - 线速度 v_x (m/s) → 轮子转速 = v_x / (π * 0.56) * 60 RPM
            # - 电机转速 = 轮子转速 * 41 = v_x * 1398 RPM
            #
            # 正确的转换：
            # target_rpm = v_x * 1400 (约)
            # 由于 motor_speed = v_x * 100，所以：
            # target_rpm = motor_speed * 14
            # ============================================================

            motor_speed = int(v_x * 100)
            target_rpm = motor_speed * 14  # 修复：原为36，现改为14

            # 检查 MSSD 连接状态
            if self.mssd and self.mssd.connected:
                if self.debug:
                    print(
                        f"[Rosmaster] MSSD: v_x={v_x:.3f} m/s → motor_speed={motor_speed} → target_rpm={target_rpm}"
                    )
                self.set_motor(motor_speed, motor_speed, 0, 0)
            else:
                if self.debug:
                    print(f"[Rosmaster] MSSD NOT connected!")

            # ============================================================
            # 2. 转向角度计算（删除分段限制，直接线性映射）
            # ============================================================
            # v_z 范围 -1 ~ 1，直接映射到转向角度 -34° ~ 34°
            # 机械限制：最大转向角度 ±34°
            # ============================================================

            MAX_STEER_ANGLE = 34  # 最大转向角度（度）

            # 线性映射：v_z → 转向角度
            steer_angle = int(v_z * MAX_STEER_ANGLE)

            # 限制在机械范围内
            steer_angle = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, steer_angle))

            if self.debug:
                print(f"[Rosmaster] Steering: v_z={v_z:.3f} → angle={steer_angle}°")

            self.set_akm_steering_angle(steer_angle)

            if self.debug:
                print(
                    f"[Rosmaster] set_car_motion: vx={v_x:.3f} m/s, vz={v_z:.3f} → motor={motor_speed}, steer={steer_angle}°"
                )
        else:
            # 原有模式：发送 FUNC_MOTION 指令到 STM32
            import struct

            try:
                vx_parms = bytearray(struct.pack("h", int(v_x * 1000)))
                vy_parms = bytearray(struct.pack("h", int(v_y * 1000)))
                vz_parms = bytearray(struct.pack("h", int(v_z * 1000)))

                cmd = [
                    HEAD,
                    DEVICE_ID,
                    0x00,
                    FUNC_MOTION,
                    self.CAR_TYPE,
                    vx_parms[0],
                    vx_parms[1],
                    vy_parms[0],
                    vy_parms[1],
                    vz_parms[0],
                    vz_parms[1],
                ]
                cmd[2] = len(cmd) - 1

                # 计算校验和
                checksum = sum(cmd, COMPLEMENT) & 0xFF
                cmd.append(checksum)

                self.stm32.ser.write(cmd)

                if self.debug:
                    print(
                        f"[Rosmaster] set_car_motion: vx={v_x:.3f}, vy={v_y:.3f}, vz={v_z:.3f}"
                    )

                time.sleep(self.delay)
            except Exception as e:
                if self.debug:
                    print(f"[Rosmaster] set_car_motion error: {e}")

    def get_motion_data(self):
        """
        获取运动数据

        Returns:
            (vx, vy, vz) - 线速度、转向角速度、角速度
        """
        if self.is_r2_mssd:
            # R2_MSSD 模式：从数据同步器获取同步数据
            sync_data = self.data_sync.get_sync_data()

            val_vx = sync_data["vx"]
            val_vy = sync_data["vy"]
            val_vz = sync_data["vz"]

            if self.debug and self.stm32.uart_state == 1:
                print(
                    f"[Rosmaster] Sync Motion: vx={val_vx:.3f} m/s, vy={val_vy:.1f}°, vz={val_vz:.3f} rad/s"
                )

            return val_vx, val_vy, val_vz
        else:
            # 原有模式：使用 STM32 数据
            val_vx = self.stm32.vx
            val_vy = self.stm32.vy
            val_vz = self.stm32.vz

            if self.debug:
                print(
                    f"[Rosmaster] STM32 Motion: vx={val_vx:.3f}, vy={val_vy:.3f}, vz={val_vz:.3f}"
                )

            return val_vx, val_vy, val_vz

    def _update_data_sync(self):
        """
        更新数据同步器的数据（内部方法）

        从STM32和MSSD读取最新数据，更新到DataSynchronizer
        """
        if not self.is_r2_mssd:
            return

        # 更新MSSD速度
        if self.mssd and self.mssd.connected:
            speed_rpm = self.mssd.get_speed()
            self.data_sync.update_mssd_speed(speed_rpm)

            # 更新MSSD编码器
            encoder = self.mssd.get_encoder()
            self.data_sync.update_mssd_encoder(encoder)

        # 更新转向角度（从ADC传感器）
        if (
            self.mssd
            and hasattr(self.mssd, "steering_sensor")
            and self.mssd.steering_sensor
        ):
            angle = self.mssd.steering_sensor.get_angle(
                channel=0, use_cache=True, cache_timeout=0.1
            )
            if angle is not None:
                self.data_sync.update_steering_angle(angle)
        else:
            # 使用STM32的转向角度作为备用
            self.data_sync.update_steering_angle(self.stm32.vy)

        # 更新IMU数据
        roll, pitch, yaw = self.stm32.get_imu_attitude()
        self.data_sync.update_imu_data(yaw, roll, pitch)

        # 更新电池电压
        voltage = self.stm32.battery_voltage / 10.0  # 转换为V
        self.data_sync.update_battery_voltage(voltage)

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
                        motor_encoder, self.mssd.last_steer_angle
                    )

                    m3 = 0
                    m4 = 0

                    if self.debug:
                        print(
                            f"[Rosmaster] Encoder: motor={motor_encoder}, left={m1}, right={m2}"
                        )

                    return m1, m2, m3, m4
                else:
                    return 0, 0, 0, 0
            else:
                return 0, 0, 0, 0
        else:
            # 原有逻辑（保持不变）
            m1, m2, m3, m4 = (
                self.stm32.encoder_m1,
                self.stm32.encoder_m2,
                self.stm32.encoder_m3,
                self.stm32.encoder_m4,
            )
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

        Note:
            直接读取 STM32 缓存的限位开关状态，不进行主动查询
            限位开关状态会通过自动上报机制实时更新
        """
        # 直接读取缓存的限位开关状态
        return self.stm32.get_limit_switch_state()

    def _wait_for_latest_limit_state(self, timeout=0.1):
        """
        等待最新的限位开关状态上报

        Args:
            timeout: 最长等待时间（秒），默认 0.1 秒

        Returns:
            限位开关状态，如果超时则返回当前状态
        """
        import time

        # 获取当前时间戳
        _, old_timestamp = self.stm32.get_limit_switch_state_with_timestamp()
        start_time = time.time()

        # 等待新的状态上报
        while time.time() - start_time < timeout:
            _, new_timestamp = self.stm32.get_limit_switch_state_with_timestamp()
            if new_timestamp > old_timestamp:
                # 收到新状态
                break
            time.sleep(0.01)  # 等待 10ms

        # 返回最新状态
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
        return {"connected": False}

    def update_encoder_distance(self):
        """
        更新编码器里程

        Returns:
            (distance_delta, total_distance) - 本次里程增量(米)和累计里程(米)
        """
        if self.mssd:
            return self.mssd.update_encoder_distance()
        return 0.0, 0.0

    def get_encoder_distance(self):
        """
        获取编码器累计里程

        Returns:
            累计里程（米）
        """
        if self.mssd:
            return self.mssd.get_encoder_distance()
        return 0.0

    def get_encoder_distance_delta(self):
        """
        获取编码器本次里程增量

        Returns:
            里程增量（米）
        """
        if self.mssd:
            return self.mssd.get_encoder_distance_delta()
        return 0.0

    def reset_encoder_distance(self):
        """
        重置编码器里程
        """
        if self.mssd:
            self.mssd.reset_encoder_distance()

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

    def set_relay_alarm(self, on_flag):
        """控制报警器开关（通道5）"""
        if self.relay:
            return self.relay.set_alarm(on_flag)
        return False

    def set_relay_pump(self, on_flag):
        """控制水泵喷水开关（通道6，同时负责水泵的开关）"""
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

        状态保持机制（定时检查，每30秒）：
        - raised状态：必须保持上限位触发，如果颠簸导致下滑，定时检查后自动启动上升
        - lowered状态：必须保持下限位触发，如果颠簸导致上滑，定时检查后自动启动下降

        限位状态定义：
        - 0x00: 都未触发
        - 0x01: 上限位触发
        - 0x02: 下限位触发
        - 0x03: 都触发（异常状态）

        Args:
            old_state: 旧的状态
            new_state: 新的状态
        """
        import time

        if self.debug:
            print(f"[Rosmaster] 限位开关状态变化: {old_state} -> {new_state}")
            print(f"[Rosmaster] 支架状态: {self.__mount_state}")
            print(f"[Rosmaster] 限位保护启用: {self.__limit_protection_enabled}")

        if not self.relay:
            if self.debug:
                print("[Rosmaster] 继电器控制器未初始化")
            return

        # ==================== 限位触发立即关闭继电器 ====================
        # raising/lowering 状态下，限位触发时立即关闭继电器

        if self.__mount_state == "raising":
            # 正在升起中，检测上限位触发
            if new_state == 0x01 or new_state == 0x03:
                if self.debug:
                    print("[Rosmaster] 上限位触发，停止升起，进入raised状态")
                # 关闭上升继电器
                self._stop_mount_relay("raise")
                self.__mount_state = "raised"
                # 重置状态检查时间
                self.__last_state_check_time = time.time()
                # 调用回调
                if self.__limit_protection_callback:
                    self.__limit_protection_callback("raised", new_state)

        elif self.__mount_state == "lowering":
            # 正在下降中，检测下限位触发
            if new_state == 0x02 or new_state == 0x03:
                if self.debug:
                    print("[Rosmaster] 下限位触发，停止下降，进入lowered状态")
                # 关闭下降继电器
                self._stop_mount_relay("lower")
                self.__mount_state = "lowered"
                # 重置状态检查时间
                self.__last_state_check_time = time.time()
                # 调用回调
                if self.__limit_protection_callback:
                    self.__limit_protection_callback("lowered", new_state)

        # ==================== raised/lowered 状态：不立即响应 ====================
        # 状态保持检查改为定时检查（每30秒），在 _check_state_maintenance 中处理
        # 这里只记录日志，不做任何动作

        elif self.__mount_state == "raised":
            # 上限位触发，确保上升继电器关闭
            if new_state == 0x01 or new_state == 0x03:
                self._stop_mount_relay("raise")
            # 状态保持检查在 _check_state_maintenance 中进行
            elif self.debug and (new_state == 0x00 or new_state == 0x02):
                print("[Rosmaster] 上限位不再触发，等待定时检查（30秒）")

        elif self.__mount_state == "lowered":
            # 下限位触发，确保下降继电器关闭
            if new_state == 0x02 or new_state == 0x03:
                self._stop_mount_relay("lower")
            # 状态保持检查在 _check_state_maintenance 中进行
            elif self.debug and (new_state == 0x00 or new_state == 0x01):
                print("[Rosmaster] 下限位不再触发，等待定时检查（30秒）")

        # 兼容旧的限位保护模式
        if self.__limit_protection_enabled:
            if self.__limit_protection_mode == "raise":
                if new_state == 0x01 or new_state == 0x03:
                    self._stop_mount_relay("raise")
                    self._stop_mount_relay("lower")
                    self._disable_limit_protection()
                    if self.__limit_protection_callback:
                        self.__limit_protection_callback("raise", new_state)
            elif self.__limit_protection_mode == "lower":
                if new_state == 0x02 or new_state == 0x03:
                    self._stop_mount_relay("lower")
                    self._stop_mount_relay("raise")
                    self._disable_limit_protection()
                    if self.__limit_protection_callback:
                        self.__limit_protection_callback("lower", new_state)

    def _check_state_maintenance(self):
        """
        检查状态保持（定时调用，每30秒检查一次）

        如果 raised 状态下限位不再触发，自动启动上升
        如果 lowered 状态下限位不再触发，自动启动下降
        """
        import time

        current_time = time.time()

        # 检查是否到达检查时间
        if current_time - self.__last_state_check_time < self.__state_check_interval:
            return

        # 更新检查时间
        self.__last_state_check_time = current_time

        # 获取当前限位状态
        limit_state = self.get_limit_switch_state()
        if limit_state is None:
            return

        if self.debug:
            print(
                f"[Rosmaster] 状态保持检查: mount_state={self.__mount_state}, limit_state={limit_state}"
            )

        if not self.relay:
            return

        # ==================== 状态保持逻辑 ====================

        if self.__mount_state == "raised":
            # 已升起状态，检查上限位是否仍触发
            if limit_state == 0x00 or limit_state == 0x02:
                # 上限位不再触发（颠簸下滑），自动启动上升
                if self.debug:
                    print("[Rosmaster] 状态保持检查: 上限位不再触发，自动启动上升")
                # 先确保下降继电器关闭（互斥）
                self._stop_mount_relay("lower")
                # 启动上升
                self._start_mount_relay("raise")
                self.__mount_state = "raising"

        elif self.__mount_state == "lowered":
            # 已下降状态，检查下限位是否仍触发
            if limit_state == 0x00 or limit_state == 0x01:
                # 下限位不再触发（颠簸上滑），自动启动下降
                if self.debug:
                    print("[Rosmaster] 状态保持检查: 下限位不再触发，自动启动下降")
                # 先确保上升继电器关闭（互斥）
                self._stop_mount_relay("raise")
                # 启动下降
                self._start_mount_relay("lower")
                self.__mount_state = "lowering"

    def _stop_mount_relay(self, direction):
        """
        停止支架继电器（确保关闭，多次重试）

        Args:
            direction: 'raise' 或 'lower'
        """
        if not self.relay:
            return

        for attempt in range(3):
            if direction == "raise":
                success = self.relay.stop_raise_mount()
            else:
                success = self.relay.stop_lower_mount()

            if success:
                break
            time.sleep(0.01)

        if self.debug:
            print(f"[Rosmaster] 停止{direction}继电器: {success}")

    def _start_mount_relay(self, direction):
        """
        启动支架继电器

        Args:
            direction: 'raise' 或 'lower'
        """
        if not self.relay:
            return False

        if direction == "raise":
            return self.relay.raise_spray_mount()
        else:
            return self.relay.lower_spray_mount()

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
            print(f"[Rosmaster] 限位保护将通过自动上报（25Hz）驱动")

    def _disable_limit_protection(self):
        """禁用限位保护（内部方法）"""
        self.__limit_protection_enabled = False
        self.__limit_protection_mode = None
        self.__limit_protection_callback = None
        if self.debug:
            print("[Rosmaster] 限位保护已禁用")

    def raise_spray_mount(self):
        """
        升起喷水支架（带限位开关保护和状态保持）

        流程：
        1. 先关闭下降继电器（互斥，无论当前状态）
        2. 等待最新的限位开关状态上报（最多 100ms）
        3. 检查上限位是否已触发，如果已触发则不执行
        4. 打开上升继电器
        5. 进入raising状态，等待上限位触发
        6. 上限位触发后进入raised状态，保持上限位

        Returns:
            是否成功
        """
        if not self.relay:
            return False

        try:
            # 1. 互斥操作：先关闭下降继电器（无论当前状态）
            if self.debug:
                print("[Rosmaster] 升起前先关闭下降继电器（互斥）")
            self._stop_mount_relay("lower")

            # 2. 等待最新的限位开关状态上报（STM32 25Hz 上报，约 40ms）
            limit_state = self._wait_for_latest_limit_state(timeout=0.1)

            # 3. 检查上限位是否已触发
            if limit_state == 0x01 or limit_state == 0x03:
                if self.debug:
                    print(
                        f"[Rosmaster] 上限位已触发（状态={limit_state}），不执行上升，进入raised状态"
                    )
                self.__mount_state = "raised"
                return True  # 已在顶部，视为成功

            # 4. 打开上升继电器
            success = self.relay.raise_spray_mount()

            if success:
                # 5. 进入raising状态
                self.__mount_state = "raising"
                # 启用限位保护（兼容旧逻辑）
                self._enable_limit_protection("raise")
                if self.debug:
                    print("[Rosmaster] 支架状态: raising")

            return success
        except Exception as e:
            if self.debug:
                print(f"[Rosmaster] 升起喷水支架错误: {e}")
            return False

    def lower_spray_mount(self):
        """
        降下喷水支架（带限位开关保护和状态保持）

        流程：
        1. 先关闭上升继电器（互斥，无论当前状态）
        2. 等待最新的限位开关状态上报（最多 100ms）
        3. 检查下限位是否已触发，如果已触发则不执行
        4. 打开下降继电器
        5. 进入lowering状态，等待下限位触发
        6. 下限位触发后进入lowered状态，保持下限位

        Returns:
            是否成功
        """
        if not self.relay:
            return False

        try:
            # 1. 互斥操作：先关闭上升继电器（无论当前状态）
            if self.debug:
                print("[Rosmaster] 下降前先关闭上升继电器（互斥）")
            self._stop_mount_relay("raise")

            # 2. 等待最新的限位开关状态上报（STM32 25Hz 上报，约 40ms）
            limit_state = self._wait_for_latest_limit_state(timeout=0.1)

            # 3. 检查下限位是否已触发
            if limit_state == 0x02 or limit_state == 0x03:
                if self.debug:
                    print(
                        f"[Rosmaster] 下限位已触发（状态={limit_state}），不执行下降，进入lowered状态"
                    )
                self.__mount_state = "lowered"
                return True  # 已在底部，视为成功

            # 4. 打开下降继电器
            success = self.relay.lower_spray_mount()

            if success:
                # 5. 进入lowering状态
                self.__mount_state = "lowering"
                # 启用限位保护（兼容旧逻辑）
                self._enable_limit_protection("lower")
                if self.debug:
                    print("[Rosmaster] 支架状态: lowering")

            return success
        except Exception as e:
            if self.debug:
                print(f"[Rosmaster] 降下喷水支架错误: {e}")
            return False

    def get_mount_state(self):
        """
        获取支架状态

        Returns:
            'idle', 'raising', 'raised', 'lowering', 'lowered'
        """
        return self.__mount_state

    def stop_raise_mount(self):
        """
        停止升起喷水支架（关闭通道7）

        Returns:
            是否成功
        """
        if self.relay:
            success = self.relay.stop_raise_mount()
            if success:
                # 不改变状态，让限位保持机制决定是否需要重新启动
                pass
            return success
        return False

    def stop_lower_mount(self):
        """
        停止降下喷水支架（关闭通道8）

        Returns:
            是否成功
        """
        if self.relay:
            return self.relay.stop_lower_mount()
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
        return {"connected": False}

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
        return {
            "level": None,
            "percentage": None,
            "status": "error",
            "message": "未初始化",
        }

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
        data = bytes(
            [
                servo_id,
                (pulse_value >> 8) & 0xFF,
                pulse_value & 0xFF,
                (run_time >> 8) & 0xFF,
                run_time & 0xFF,
                0,
                0,
            ]
        )
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
            self.stm32.send_command(
                self.FUNC_REQUEST_DATA, bytes([self.FUNC_VERSION, 0])
            )

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

    # ========== 舵机控制方法 ==========

    def set_uart_servo(self, servo_id, pulse_value, run_time=500):
        """控制总线舵机"""
        return self.stm32.set_uart_servo(servo_id, pulse_value, run_time)

    def set_uart_servo_angle(self, s_id, s_angle, run_time=500):
        """控制总线舵机角度"""
        if s_id == 1:
            if 0 <= s_angle <= 180:
                value = self._arm_convert_value(s_id, s_angle)
                return self.stm32.set_uart_servo(s_id, value, run_time)
        elif s_id == 2:
            if 0 <= s_angle <= 180:
                value = self._arm_convert_value(s_id, s_angle)
                return self.stm32.set_uart_servo(s_id, value, run_time)
        elif s_id == 3:
            if 0 <= s_angle <= 180:
                value = self._arm_convert_value(s_id, s_angle)
                return self.stm32.set_uart_servo(s_id, value, run_time)
        elif s_id == 4:
            if 0 <= s_angle <= 180:
                value = self._arm_convert_value(s_id, s_angle)
                return self.stm32.set_uart_servo(s_id, value, run_time)
        elif s_id == 5:
            if 0 <= s_angle <= 270:
                value = self._arm_convert_value(s_id, s_angle)
                return self.stm32.set_uart_servo(s_id, value, run_time)
        elif s_id == 6:
            if 0 <= s_angle <= 180:
                value = self._arm_convert_value(s_id, s_angle)
                return self.stm32.set_uart_servo(s_id, value, run_time)
        return False

    def set_uart_servo_angle_array(
        self, angle_s=[90, 90, 90, 90, 90, 180], run_time=500
    ):
        """控制机械臂所有关节"""
        return self.stm32.set_arm_ctrl(angle_s, run_time)

    def set_uart_servo_id(self, servo_id):
        """设置总线舵机ID"""
        return self.stm32.set_uart_servo_id(servo_id)

    def set_uart_servo_torque(self, enable):
        """设置总线舵机扭矩"""
        return self.stm32.set_uart_servo_torque(enable)

    def set_uart_servo_offset(self, servo_id):
        """设置机械臂中位偏差"""
        return self.stm32.set_arm_offset(servo_id)

    def set_uart_servo_ctrl_enable(self, enable):
        """设置机械臂控制开关"""
        self.stm32.arm_ctrl_enable = enable

    # ========== PID控制方法 ==========

    def set_pid_param(self, kp, ki, kd, forever=False):
        """设置电机PID参数"""
        return self.stm32.set_motor_pid(kp, ki, kd, forever)

    # ========== RGB灯带控制方法 ==========

    def set_colorful_lamps(self, led_id, red, green, blue):
        """设置RGB灯带颜色"""
        return self.stm32.set_colorful_lamps(led_id, red, green, blue)

    # ========== 系统控制方法 ==========

    def reset_flash_value(self):
        """重置Flash数据"""
        return self.stm32.reset_flash_value()

    def reset_car_state(self):
        """重置小车状态"""
        return self.stm32.reset_car_state()

    def clear_auto_report_data(self):
        """清除自动上报缓存数据"""
        self.stm32.vx = 0
        self.stm32.vy = 0
        self.stm32.vz = 0
        self.stm32.ax = 0
        self.stm32.ay = 0
        self.stm32.az = 0
        self.stm32.gx = 0
        self.stm32.gy = 0
        self.stm32.gz = 0
        self.stm32.mx = 0
        self.stm32.my = 0
        self.stm32.mz = 0
        self.stm32.yaw = 0
        self.stm32.roll = 0
        self.stm32.pitch = 0

    # ========== 数据获取方法 ==========

    def get_uart_servo_value(self):
        """获取总线舵机脉冲值"""
        return self.stm32.get_uart_servo_value()

    def get_uart_servo_angle(self, s_id):
        """获取总线舵机角度"""
        return self.stm32.get_uart_servo_angle(s_id)

    def get_uart_servo_angle_array(self):
        """获取机械臂角度数组"""
        return self.stm32.get_uart_servo_angle_array()

    def get_mpu_raw_data(self):
        """获取MPU9250原始数据"""
        return (
            self.stm32.ax,
            self.stm32.ay,
            self.stm32.az,
            self.stm32.gx,
            self.stm32.gy,
            self.stm32.gz,
            self.stm32.mx,
            self.stm32.my,
            self.stm32.mz,
        )

    def get_icm_raw_data(self):
        """获取ICM20948原始数据"""
        return (
            self.stm32.ax,
            self.stm32.ay,
            self.stm32.az,
            self.stm32.gx,
            self.stm32.gy,
            self.stm32.gz,
            self.stm32.mx,
            self.stm32.my,
            self.stm32.mz,
        )

    def get_motion_pid(self, motor_id):
        """获取电机PID参数"""
        return self.stm32.get_motion_pid(motor_id)

    def get_yaw_pid(self):
        """获取偏航PID参数"""
        return self.stm32.get_yaw_pid()

    def get_akm_default_angle(self):
        """获取阿克曼默认角度"""
        return self.stm32.get_akm_default_angle()

    def get_car_type_from_machine(self):
        """从机器获取车型"""
        return self.stm32.get_car_type_from_machine()

    # ========== 阿克曼控制方法 ==========

    def set_akm_default_angle(self, angle, forever=False):
        """设置阿克曼默认角度"""
        return self.stm32.set_akm_default_angle(angle, forever)

    # ========== 内部工具方法 ==========

    def _arm_convert_value(self, s_id, s_angle):
        """将角度转换为脉冲值"""
        return self.stm32._arm_convert_value(s_id, s_angle)

    def _arm_convert_angle(self, s_id, s_value):
        """将脉冲值转换为角度"""
        return self.stm32._arm_convert_angle(s_id, s_value)

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
