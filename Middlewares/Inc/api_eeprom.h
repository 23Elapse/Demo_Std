/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-18 20:48:07
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-29 09:54:16
 * @FilePath: \Demo\Middlewares\Inc\api_eeprom.h
 * @Description: EEPROM驱动头文件
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef __API_EEPROM_H
#define __API_EEPROM_H

//#include "pch.h"
 #include "iic_core.h"

#define EEPROM_TYPE AT24C02  //通过宏定义选择具体型号
#define EEPROM_ADDR 0xA0     //基础设备地址（A2/A1/A0接地时为0xA0）

#define AT24C01     127
#define AT24C02     255
#define AT24C04     511
#define AT24C08     1023
#define AT24C16     2047
#define AT24C32     4095
#define AT24C64     8191
#define AT24C128    16383
#define AT24C256    32767

/* 开发板使用的是24c02，所以定义EE_TYPE为AT24C02 */
#define EE_TYPE     AT24C02
//extern const IIC_Ops_t IIC1_EEPROM;
IIC_Status EEPROMReadByteFromReg(uint8_t reg, uint8_t *val);
IIC_Status EEPROMWriteByteToReg(uint8_t reg, uint8_t val);
#endif
