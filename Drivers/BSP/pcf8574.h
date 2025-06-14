/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-05 21:24:07
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-08 17:20:16
 * @FilePath: \Demo_backup\Drivers\BSP\pcf8574.h
 * @Description: PCF8574 IIC 扩展 IO 驱动头文件
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __PCF8574_H
#define __PCF8574_H

#include "i2c_driver.h"

/* PCF8574 设备地址（7位地址，左移一位后为 0x40） */
#define PCF8574_ADDR 0x40

/* PCF8574 IO 功能定义 */
#define BEEP_IO 0      /* 蜂鸣器控制引脚 */
#define AP_INT_IO 1    /* AP3216C 中断引脚 */
#define DCMI_PWDN_IO 2 /* DCMI 电源控制引脚 */
#define USB_PWR_IO 3   /* USB 电源控制引脚 */
#define EX_IO 4        /* 扩展 IO */
#define MPU_INT_IO 5   /* SH3001 中断引脚 */
#define RS485_RE_IO 6  /* RS485_RE 引脚 */
#define ETH_RESET_IO 7 /* 以太网复位引脚 */

/**
 * @brief  初始化PCF8574设备（逻辑上，主要是测试通信）
 * @param  bus 指向其所连接的I2C总线
 * @param  dev_addr 设备的7位I2C地址
 * @return I2C_Status_t 通信状态
 */
I2C_Status_t PCF8574_Init(I2C_Bus_t* bus, uint8_t dev_addr);

/**
 * @brief  向PCF8574写入一个字节以控制所有8个I/O口
 * @param  bus 指向其所连接的I2C总线
 * @param  dev_addr 设备的7位I2C地址
 * @param  data 要写入的字节
 * @return I2C_Status_t 通信状态
 */
I2C_Status_t PCF8574_WriteByte(I2C_Bus_t* bus, uint8_t dev_addr, uint8_t data);

/**
 * @brief  从PCF8574读取一个字节以获取所有8个I/O口的状态
 * @param  bus 指向其所连接的I2C总线
 * @param  dev_addr 设备的7位I2C地址
 * @param  p_data 指向存储读取数据的变量
 * @return I2C_Status_t 通信状态
 */
I2C_Status_t PCF8574_ReadByte(I2C_Bus_t* bus, uint8_t dev_addr, uint8_t* p_data);

#endif // __PCF8574_H
