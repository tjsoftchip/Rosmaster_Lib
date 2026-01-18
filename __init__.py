#!/usr/bin/env python3
# coding: utf-8

"""
Rosmaster_Lib - Yahboom 机器人控制库
支持多种车型：X3, X3 Plus, X1, R2, R2_MSSD

版本：V3.4.0（模块化重构版）
日期：2026-01-16
"""

import os

# 导入新的模块化 Rosmaster 类
from .core.rosmaster import Rosmaster
from .core.constants import *

# 车型名称映射（从 constants 导入）
CAR_TYPE_NAMES = {
    0x01: "X3",
    0x02: "X3_PLUS",
    0x04: "X1",
    0x05: "R2",
    0x06: "R2_MSSD"
}


def get_car_type_from_env():
    """
    从环境变量获取车型类型
    
    支持的环境变量（按优先级）：
    1. CAR_TYPE - 车型编号（0x01, 0x02, 0x04, 0x05, 0x06）
    2. ROBOT_TYPE - 车型名称（X3, X3_PLUS, X1, R2, R2_MSSD）
    
    Returns:
        车型编号，如果未设置则返回 None
    """
    # 方法 1：从 CAR_TYPE 环境变量读取车型编号
    car_type_str = os.getenv('CAR_TYPE')
    if car_type_str:
        try:
            # 支持十六进制格式（0x05）和十进制格式（5）
            if car_type_str.startswith('0x') or car_type_str.startswith('0X'):
                car_type = int(car_type_str, 16)
            else:
                car_type = int(car_type_str)
            
            # 验证车型编号是否有效
            if car_type in CAR_TYPE_NAMES:
                print(f"[Rosmaster_Lib] 从环境变量 CAR_TYPE 读取车型: {car_type_str} ({CAR_TYPE_NAMES[car_type]})")
                return car_type
            else:
                print(f"[Rosmaster_Lib] 警告: 无效的车型编号: {car_type_str}")
        except ValueError:
            print(f"[Rosmaster_Lib] 警告: CAR_TYPE 格式错误: {car_type_str}")
    
    # 方法 2：从 ROBOT_TYPE 环境变量读取车型名称
    car_model = os.getenv('ROBOT_TYPE')
    if car_model:
        # 查找匹配的车型
        car_model_upper = car_model.upper().replace('-', '_')
        for car_type, name in CAR_TYPE_NAMES.items():
            if name.upper() == car_model_upper:
                print(f"[Rosmaster_Lib] 从环境变量 CAR_MODEL 读取车型: {car_model} ({name})")
                return car_type
        
        print(f"[Rosmaster_Lib] 警告: 未知的车型名称: {car_model}")
        print(f"[Rosmaster_Lib] 支持的车型: {', '.join(CAR_TYPE_NAMES.values())}")
    
    return None


def create_rosmaster(com="/dev/myserial", delay=0.002, debug=False, 
                    mssd_port="/dev/mssd", relay_port="/dev/modbus", 
                    enable_mssd=False, car_type=None):
    """
    创建 Rosmaster 实例（自动从环境变量读取车型）
    
    这是推荐的初始化方式，可以确保 ROS2 代码无需修改即可使用新库。
    
    Args:
        com: STM32 串口路径（默认 /dev/myserial）
        delay: 指令延迟时间（秒）
        debug: 调试模式
        mssd_port: MSSD 串口路径（默认 /dev/mssd，仅 R2_MSSD 模式使用）
        relay_port: 继电器串口路径（默认 /dev/modbus，仅 R2_MSSD 模式使用）
        enable_mssd: 是否启用 MSSD 模式
        car_type: 车型类型（可选，优先级最高）
    
    Returns:
        Rosmaster 实例
        
    Example:
        # 方式 1：通过环境变量设置（推荐）
        # 在 .bashrc 中添加：export CAR_TYPE=6
        bot = create_rosmaster()
        
        # 方式 2：直接指定车型
        bot = create_rosmaster(car_type=6)
        
        # 方式 3：使用车型名称
        # 在 .bashrc 中添加：export ROBOT_TYPE=R2_MSSD
        bot = create_rosmaster()
        
        # 方式 4：自定义串口
        bot = create_rosmaster(
            car_type=6,
            com="/dev/ttyTHS1",
            mssd_port="/dev/ttyUSB0",
            relay_port="/dev/ttyUSB1"
        )
    """
    # 优先级：参数 > 环境变量 > 默认值
    if car_type is None:
        car_type = get_car_type_from_env()
    
    # 如果环境变量也没有设置，使用默认值
    if car_type is None:
        print("[Rosmaster_Lib] 未设置车型，使用默认车型: R2 (0x05)")
        car_type = CARTYPE_R2
    
    # 创建 Rosmaster 实例
    print(f"[Rosmaster_Lib] 初始化 Rosmaster: 车型={CAR_TYPE_NAMES.get(car_type, 'UNKNOWN')} ({car_type})")
    print(f"[Rosmaster_Lib] 串口配置: STM32={com}, MSSD={mssd_port}, RELAY={relay_port}")
    
    return Rosmaster(
        car_type=car_type,
        com=com,
        delay=delay,
        debug=debug,
        mssd_port=mssd_port,
        relay_port=relay_port,
        enable_mssd=enable_mssd
    )


# 为了保持向后兼容，导出 Rosmaster 类和所有常量
__all__ = [
    'Rosmaster', 'create_rosmaster', 
    'CARTYPE_X3', 'CARTYPE_X3_PLUS', 'CARTYPE_X1', 
    'CARTYPE_R2', 'CARTYPE_R2_MSSD',
    'CAR_TYPE_NAMES', 'get_car_type_from_env'
]
