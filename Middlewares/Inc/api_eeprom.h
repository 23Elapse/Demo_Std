/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-18 20:48:07
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 01:24:04
 * @FilePath: \Demo\Middlewares\Inc\api_eeprom.h
 * @Description: EEPROM 驱动头文件
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __API_EEPROM_H
#define __API_EEPROM_H

#include "iic_core.h"
#include "log_system.h"

#define EEPROM_TYPE AT24C02 // 通过宏定义选择具体型号
#define EEPROM_ADDR 0xA0    // 基础设备地址（A2/A1/A0接地时为0xA0）
#define EEPROM_PAGE_SIZE 8  // AT24C02 页大小（字节）

#define AT24C01 127
#define AT24C02 255
#define AT24C04 511
#define AT24C08 1023
#define AT24C16 2047
#define AT24C32 4095
#define AT24C64 8191
#define AT24C128 16383
#define AT24C256 32767

#define EE_TYPE AT24C02 // 开发板使用的是24C02

extern const IIC_Ops_t IIC1_EEPROM;

/**
 * @brief 从指定寄存器读取一个字节
 * @param reg 寄存器地址
 * @param val 读取的数据存储指针
 * @return IIC_Status 操作状态
 */
IIC_Status EEPROMReadByteFromReg(uint8_t reg, uint8_t *val);

/**
 * @brief 向指定寄存器写入一个字节
 * @param reg 寄存器地址
 * @param val 要写入的数据
 * @return IIC_Status 操作状态
 */
IIC_Status EEPROMWriteByteToReg(uint8_t reg, uint8_t val);

/**
 * @brief 从指定寄存器读取多字节
 * @param reg 起始寄存器地址
 * @param buffer 数据存储缓冲区
 * @param length 要读取的字节数
 * @return IIC_Status 操作状态
 */
IIC_Status EEPROMReadBytesFromReg(uint8_t reg, uint8_t *buffer, uint16_t length);

/**
 * @brief 向指定寄存器写入多字节
 * @param reg 起始寄存器地址
 * @param buffer 要写入的数据缓冲区
 * @param length 要写入的字节数
 * @return IIC_Status 操作状态
 */
IIC_Status EEPROMWriteBytesToReg(uint8_t reg, const uint8_t *buffer, uint16_t length);

#endif

