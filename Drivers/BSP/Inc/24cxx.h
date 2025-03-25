/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-05 21:24:07
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-03-24 19:50:08
 * @FilePath: \Demo\Drivers\BSP\Inc\24cxx.h
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */

#ifndef __24CXX_H
#define __24CXX_H
#include "pch.h"
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

/******************************************************************************************/

void at24cxx_init(void);                                                                    /* 初始化IIC */
uint8_t AT24CXX_ReadOneByte(IIC_TypeDef *IIC_Struct, uint16_t addr) ;                       /* 读取一个字节 */
void AT24CXX_WriteOneByte(IIC_TypeDef *IIC_Struct, uint16_t addr, uint8_t data);            /* 写入一个字节 */
void AT24CXX_Write(IIC_TypeDef *IIC_Struct, uint16_t addr, uint8_t *buf, uint16_t len);     /* 写入多个字节 */
void AT24CXX_Read(IIC_TypeDef *IIC_Struct, uint16_t addr, uint8_t *buf, uint16_t len);      /* 读取多个字节 */
uint8_t AT24CXX_Check(IIC_TypeDef *IIC_Struct);                                             /* 检查设备 */
#endif













