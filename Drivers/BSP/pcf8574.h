/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-05 21:24:07
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-14 18:58:18
 * @FilePath: \Demo_backup\Drivers\BSP\pcf8574.h
 * @Description: PCF8574 IIC 扩展 IO 驱动头文件 (Refactored)
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __PCF8574_H
#define __PCF8574_H

#include "i2c_driver.h" // 依赖 I2C 总线驱动

/* PCF8574 设备地址（7位地址，左移一位后为 0x40） */
// 注意：实际地址可能因A0-A2引脚连接而异，这里是默认值
#define PCF8574_DEFAULT_ADDR 0x20 // 7位地址，例如0100 000B

/* PCF8574 IO 功能定义 (示例，可根据实际硬件连接修改) */
#define BEEP_IO 0      /* 蜂鸣器控制引脚 */
#define AP_INT_IO 1    /* AP3216C 中断引脚 */
#define DCMI_PWDN_IO 2 /* DCMI 电源控制引脚 */
#define USB_PWR_IO 3   /* USB 电源控制引脚 */
#define EX_IO 4        /* 扩展 IO (通用目的) */
#define MPU_INT_IO 5   /* SH3001 中断引脚 */
#define RS485_RE_IO 6  /* RS485_RE 引脚 */
#define ETH_RESET_IO 7 /* 以太网复位引脚 */

/**
 * @brief PCF8574 设备结构体
 * @note  封装了PCF8574所连接的I2C总线和其设备地址
 */
typedef struct {
    I2C_Bus_t* i2c_bus; // 指向PCF8574连接的I2C总线实例
    uint8_t    dev_addr; // PCF8574的7位I2C从设备地址
    // 可以添加当前IO口状态缓存，避免频繁读写总线
    uint8_t    current_io_state;
} PCF8574_Device_t;

/**
 * @brief  初始化PCF8574设备（逻辑上，主要是测试通信）
 * @param  dev 指向PCF8574设备实例
 * @return I2C_Status_t 通信状态
 */
I2C_Status_t PCF8574_Init(PCF8574_Device_t* dev);

/**
 * @brief  向PCF8574写入一个字节以控制所有8个I/O口
 * @param  dev 指向PCF8574设备实例
 * @param  data 要写入的字节，每个位对应一个I/O口
 * @return I2C_Status_t 通信状态
 */
I2C_Status_t PCF8574_WriteByte(PCF8574_Device_t* dev, uint8_t data);

/**
 * @brief  从PCF8574读取一个字节以获取所有8个I/O口的状态
 * @param  dev 指向PCF8574设备实例
 * @param  p_data 指向存储读取数据的变量
 * @return I2C_Status_t 通信状态
 */
I2C_Status_t PCF8574_ReadByte(PCF8574_Device_t* dev, uint8_t* p_data);

#endif // __PCF8574_H
