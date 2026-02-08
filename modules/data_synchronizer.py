#!/usr/bin/env python3
# coding: utf-8

"""
数据同步器模块

负责统一数据更新频率、时间戳对齐、数据缓存和过期检查

功能：
- 统一数据更新频率（25Hz，40ms）
- 数据缓存机制
- 时间戳对齐
- 数据过期检查
- 独立线程处理数据更新

作者：nx-ros2
日期：2026-01-26
"""

import time
import threading
import math
from collections import deque


class DataSynchronizer:
    """
    数据同步器
    
    负责协调所有传感器数据的时间同步，确保ROS2消息时间线对准
    """
    
    def __init__(self, update_interval=0.04, expire_time=0.1, debug=False):
        """
        初始化数据同步器
        
        Args:
            update_interval: 数据更新间隔（秒），默认0.04s（25Hz）
            expire_time: 数据过期时间（秒），默认0.1s
            debug: 调试模式
        """
        self.update_interval = update_interval
        self.expire_time = expire_time
        self.debug = debug
        
        # 数据缓存
        self._data_cache = {
            'mssd_speed': {'value': 0.0, 'timestamp': 0.0, 'valid': False},
            'mssd_encoder': {'value': 0, 'timestamp': 0.0, 'valid': False},
            'steering_angle': {'value': 0.0, 'timestamp': 0.0, 'valid': False},
            'imu_yaw': {'value': 0.0, 'timestamp': 0.0, 'valid': False},
            'imu_roll': {'value': 0.0, 'timestamp': 0.0, 'valid': False},
            'imu_pitch': {'value': 0.0, 'timestamp': 0.0, 'valid': False},
            'battery_voltage': {'value': 0.0, 'timestamp': 0.0, 'valid': False},
        }
        
        # 同步数据（用于ROS2发布）
        self._sync_data = {
            'vx': 0.0,           # 线速度 (m/s)
            'vy': 0.0,           # 转向角度 (度)
            'vz': 0.0,           # 角速度 (rad/s)
            'encoder': 0,        # 编码器值
            'imu_yaw': 0.0,      # IMU偏航角 (rad)
            'imu_roll': 0.0,     # IMU横滚角 (rad)
            'imu_pitch': 0.0,    # IMU俯仰角 (rad)
            'voltage': 0.0,      # 电池电压 (V)
            'timestamp': 0.0,    # 统一时间戳
        }
        
        # 数据更新锁
        self._lock = threading.Lock()
        
        # 更新线程
        self._update_thread = None
        self._running = False
        
        # 统计信息
        self._stats = {
            'update_count': 0,
            'missed_updates': 0,
            'last_update_time': 0.0,
            'avg_update_time': 0.0,
        }
        
        if self.debug:
            print(f"[DataSynchronizer] 初始化完成")
            print(f"[DataSynchronizer] 更新间隔: {self.update_interval*1000:.1f}ms ({1/self.update_interval:.1f}Hz)")
            print(f"[DataSynchronizer] 过期时间: {self.expire_time*1000:.1f}ms")
    
    def start(self):
        """启动数据同步器"""
        if self._running:
            if self.debug:
                print("[DataSynchronizer] 已经在运行")
            return False
        
        self._running = True
        self._update_thread = threading.Thread(
            target=self._update_loop,
            name="data_synchronizer_thread"
        )
        self._update_thread.setDaemon(True)
        self._update_thread.start()
        
        if self.debug:
            print("[DataSynchronizer] 数据同步器已启动")
        
        return True
    
    def stop(self):
        """停止数据同步器"""
        if not self._running:
            return False
        
        self._running = False
        
        # 等待线程结束
        if self._update_thread:
            self._update_thread.join(timeout=1.0)
        
        if self.debug:
            print("[DataSynchronizer] 数据同步器已停止")
        
        return True
    
    def _update_loop(self):
        """数据更新循环（独立线程）"""
        while self._running:
            start_time = time.time()
            
            try:
                # 更新同步数据
                self._update_sync_data()
                
                # 更新统计信息
                self._stats['update_count'] += 1
                self._stats['last_update_time'] = time.time()
                
                # 计算平均更新时间
                if self._stats['update_count'] > 0:
                    elapsed = time.time() - start_time
                    self._stats['avg_update_time'] = (
                        self._stats['avg_update_time'] * (self._stats['update_count'] - 1) + elapsed
                    ) / self._stats['update_count']
                
            except Exception as e:
                if self.debug:
                    print(f"[DataSynchronizer] 更新错误: {e}")
                self._stats['missed_updates'] += 1
            
            # 等待下一个更新周期
            elapsed = time.time() - start_time
            sleep_time = max(0, self.update_interval - elapsed)
            time.sleep(sleep_time)
    
    def _update_sync_data(self):
        """更新同步数据"""
        current_time = time.time()
        
        with self._lock:
            # 获取MSSD速度
            mssd_speed = self._data_cache['mssd_speed']
            if mssd_speed['valid'] and self._is_data_valid(mssd_speed):
                # 计算线速度
                vx = mssd_speed['value'] / 60.0 / 41.0 * math.pi * 0.56
                self._sync_data['vx'] = vx
            else:
                self._sync_data['vx'] = 0.0
            
            # 获取转向角度
            steering_angle = self._data_cache['steering_angle']
            if steering_angle['valid'] and self._is_data_valid(steering_angle):
                self._sync_data['vy'] = steering_angle['value']
            else:
                self._sync_data['vy'] = 0.0
            
            # 计算角速度（基于阿克曼模型）
            if abs(self._sync_data['vy']) > 0.5:
                theta = math.radians(self._sync_data['vy'])
                R = 1.355 / math.tan(theta)
                if abs(R) > 0.001:
                    vz = self._sync_data['vx'] / R
                else:
                    vz = 0.0
            else:
                vz = 0.0
            self._sync_data['vz'] = vz
            
            # 获取编码器
            mssd_encoder = self._data_cache['mssd_encoder']
            if mssd_encoder['valid'] and self._is_data_valid(mssd_encoder):
                self._sync_data['encoder'] = mssd_encoder['value']
            else:
                self._sync_data['encoder'] = 0
            
            # 获取IMU数据
            imu_yaw = self._data_cache['imu_yaw']
            if imu_yaw['valid'] and self._is_data_valid(imu_yaw):
                self._sync_data['imu_yaw'] = math.radians(imu_yaw['value'])
            else:
                self._sync_data['imu_yaw'] = 0.0
            
            imu_roll = self._data_cache['imu_roll']
            if imu_roll['valid'] and self._is_data_valid(imu_roll):
                self._sync_data['imu_roll'] = math.radians(imu_roll['value'])
            else:
                self._sync_data['imu_roll'] = 0.0
            
            imu_pitch = self._data_cache['imu_pitch']
            if imu_pitch['valid'] and self._is_data_valid(imu_pitch):
                self._sync_data['imu_pitch'] = math.radians(imu_pitch['value'])
            else:
                self._sync_data['imu_pitch'] = 0.0
            
            # 获取电池电压
            battery_voltage = self._data_cache['battery_voltage']
            if battery_voltage['valid'] and self._is_data_valid(battery_voltage):
                self._sync_data['voltage'] = battery_voltage['value']
            else:
                self._sync_data['voltage'] = 0.0
            
            # 更新统一时间戳
            self._sync_data['timestamp'] = current_time
            
            if self.debug and self._stats['update_count'] % 25 == 0:
                print(f"[DataSynchronizer] 更新 #{self._stats['update_count']}: "
                      f"vx={self._sync_data['vx']:.3f}m/s, "
                      f"vy={self._sync_data['vy']:.1f}°, "
                      f"vz={self._sync_data['vz']:.3f}rad/s")
    
    def _is_data_valid(self, data):
        """检查数据是否有效（未过期）"""
        current_time = time.time()
        age = current_time - data['timestamp']
        return age < self.expire_time
    
    def update_mssd_speed(self, speed_rpm):
        """更新MSSD速度缓存"""
        with self._lock:
            self._data_cache['mssd_speed']['value'] = speed_rpm
            self._data_cache['mssd_speed']['timestamp'] = time.time()
            self._data_cache['mssd_speed']['valid'] = True
    
    def update_mssd_encoder(self, encoder):
        """更新MSSD编码器缓存"""
        with self._lock:
            self._data_cache['mssd_encoder']['value'] = encoder
            self._data_cache['mssd_encoder']['timestamp'] = time.time()
            self._data_cache['mssd_encoder']['valid'] = True
    
    def update_steering_angle(self, angle):
        """更新转向角度缓存"""
        with self._lock:
            self._data_cache['steering_angle']['value'] = angle
            self._data_cache['steering_angle']['timestamp'] = time.time()
            self._data_cache['steering_angle']['valid'] = True
    
    def update_imu_data(self, yaw, roll, pitch):
        """更新IMU数据缓存"""
        with self._lock:
            self._data_cache['imu_yaw']['value'] = yaw
            self._data_cache['imu_yaw']['timestamp'] = time.time()
            self._data_cache['imu_yaw']['valid'] = True
            
            self._data_cache['imu_roll']['value'] = roll
            self._data_cache['imu_roll']['timestamp'] = time.time()
            self._data_cache['imu_roll']['valid'] = True
            
            self._data_cache['imu_pitch']['value'] = pitch
            self._data_cache['imu_pitch']['timestamp'] = time.time()
            self._data_cache['imu_pitch']['valid'] = True
    
    def update_battery_voltage(self, voltage):
        """更新电池电压缓存"""
        with self._lock:
            self._data_cache['battery_voltage']['value'] = voltage
            self._data_cache['battery_voltage']['timestamp'] = time.time()
            self._data_cache['battery_voltage']['valid'] = True
    
    def get_sync_data(self):
        """获取同步数据（线程安全）"""
        with self._lock:
            # 返回数据的副本，避免外部修改
            return self._sync_data.copy()
    
    def get_stats(self):
        """获取统计信息"""
        with self._lock:
            return self._stats.copy()
    
    def invalidate_all_cache(self):
        """使所有缓存失效"""
        with self._lock:
            for key in self._data_cache:
                self._data_cache[key]['valid'] = False
        
        if self.debug:
            print("[DataSynchronizer] 所有缓存已失效")
    
    def __del__(self):
        """析构函数，自动停止"""
        self.stop()