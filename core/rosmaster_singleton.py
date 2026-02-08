#!/usr/bin/env python3
# coding: utf-8

"""
Rosmaster 单例管理器

确保所有 ROS2 节点共享同一个 Rosmaster 实例，避免串口冲突

作者：nx-ros2
日期：2026-01-18
"""

import threading
from .rosmaster import Rosmaster
from .constants import *


class RosmasterSingleton:
    """
    Rosmaster 单例管理器

    功能：
    - 确保全局只有一个 Rosmaster 实例
    - 线程安全的初始化
    - 支持延迟初始化
    - 自动管理资源清理
    - 确保串口只在第一次初始化时连接
    """

    _instance = None
    _lock = threading.Lock()
    _initialized = False
    _serial_ports_connected = False  # 跟踪串口是否已连接
    
    @classmethod
    def get_instance(cls, car_type=5, com="/dev/myserial", delay=0.002,
                    debug=False, mssd_port="/dev/mssd", relay_port="/dev/modbus",
                    enable_mssd=False):
        """
        获取 Rosmaster 单例实例

        Args:
            car_type: 车型类型
            com: STM32 串口路径
            delay: 指令延迟时间
            debug: 调试模式
            mssd_port: MSSD 串口路径
            relay_port: 继电器串口路径
            enable_mssd: 是否启用 MSSD

        Returns:
            Rosmaster 实例
        """
        with cls._lock:
            if cls._instance is None:
                # 第一次创建实例，连接所有串口
                cls._instance = Rosmaster(
                    car_type=car_type,
                    com=com,
                    delay=delay,
                    debug=debug,
                    mssd_port=mssd_port,
                    relay_port=relay_port,
                    enable_mssd=enable_mssd
                )
                cls._initialized = True
                cls._serial_ports_connected = True

                if debug:
                    print(f"[Singleton] Rosmaster 实例已创建: 车型={CAR_TYPE_NAMES.get(car_type, 'UNKNOWN')} ({car_type})")
                    print(f"[Singleton] 串口已连接: STM32={com}, MSSD={mssd_port}, RELAY={relay_port}")
            else:
                # 后续调用，复用已存在的实例
                if debug:
                    print(f"[Singleton] 返回已存在的 Rosmaster 实例（串口已连接）")

            return cls._instance
    
    @classmethod
    def is_initialized(cls):
        """检查是否已初始化"""
        return cls._initialized
    
    @classmethod
    def reset(cls):
        """重置单例（用于测试）"""
        with cls._lock:
            if cls._instance is not None:
                # 清理资源
                if hasattr(cls._instance, 'relay') and cls._instance.relay:
                    cls._instance.relay.disconnect()
                if hasattr(cls._instance, 'mssd') and cls._instance.mssd:
                    cls._instance.mssd.disconnect()
                if hasattr(cls._instance, 'stm32') and cls._instance.stm32:
                    cls._instance.stm32.disconnect()

                cls._instance = None
                cls._initialized = False
                cls._serial_ports_connected = False

                print("[Singleton] Rosmaster 实例已重置，串口连接状态已清除")


# 便捷函数
def get_rosmaster(car_type=5, com="/dev/myserial", delay=0.002, 
                  debug=False, mssd_port="/dev/mssd", relay_port="/dev/modbus",
                  enable_mssd=False):
    """
    获取 Rosmaster 单例实例（便捷函数）
    
    Args:
        car_type: 车型类型
        com: STM32 串口路径
        delay: 指令延迟时间
        debug: 调试模式
        mssd_port: MSSD 串口路径
        relay_port: 继电器串口路径
        enable_mssd: 是否启用 MSSD
    
    Returns:
        Rosmaster 实例
    """
    return RosmasterSingleton.get_instance(
        car_type=car_type,
        com=com,
        delay=delay,
        debug=debug,
        mssd_port=mssd_port,
        relay_port=relay_port,
        enable_mssd=enable_mssd
    )


def is_rosmaster_initialized():
    """检查 Rosmaster 是否已初始化"""
    return RosmasterSingleton.is_initialized()


def reset_rosmaster():
    """重置 Rosmaster 单例（用于测试）"""
    RosmasterSingleton.reset()