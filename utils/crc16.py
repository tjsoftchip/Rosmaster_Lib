#!/usr/bin/env python3
# coding: utf-8

"""
CRC16 校验工具

用于 Modbus RTU 协议的 CRC16 校验计算
"""

def calculate_crc16(data):
    """
    计算 CRC16 校验码（Modbus RTU 标准）
    
    Args:
        data: 字节数据（bytes 或 bytearray）
    
    Returns:
        CRC16 校验码（16位无符号整数）
    
    Example:
        >>> data = bytes([0x01, 0x03, 0x00, 0x09, 0x00, 0x02])
        >>> crc = calculate_crc16(data)
        >>> print(f"CRC16: 0x{crc:04X}")
    """
    crc = 0xFFFF
    
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    
    return crc


def verify_crc16(data, expected_crc):
    """
    验证 CRC16 校验码
    
    Args:
        data: 字节数据
        expected_crc: 期望的 CRC16 校验码
    
    Returns:
        是否验证通过
    """
    calculated_crc = calculate_crc16(data)
    return calculated_crc == expected_crc


def append_crc16(data):
    """
    在数据末尾追加 CRC16 校验码
    
    Args:
        data: 原始字节数据
    
    Returns:
        追加 CRC16 后的字节数据
    """
    crc = calculate_crc16(data)
    return data + bytes([crc & 0xFF, (crc >> 8) & 0xFF])