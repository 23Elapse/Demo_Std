/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-06-08 16:13:58
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-08 17:28:21
 * @FilePath: \Demo_backup\Drivers\BSP\i2c_driver.h
 * @Description: IIC 驱动头文件
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
/**
 * =====================================================================================
 * @file        i2c_driver.h
 * @brief       通用的软件I2C总线驱动头文件（支持多总线、带RTOS锁）
 * @author      23Elapse & Gemini
 * @version     2.0 (Refactored)
 * @date        2025-06-08
 * =====================================================================================
 */
#ifndef __I2C_DRIVER_H
#define __I2C_DRIVER_H

#include "stm32f4xx.h"
#include "rtos_abstraction.h"

/**
 * @brief I2C 操作状态枚举
 */
typedef enum {
    I2C_OK = 0,           // 操作成功
    I2C_ERR_PARAM,        // 参数错误
    I2C_ERR_TIMEOUT,      // 等待超时 (例如获取Mutex)
    I2C_ERR_NO_ACK,       // 未收到ACK信号
    I2C_ERR_BUS_BUSY,     // 总线忙
    I2C_ERR_UNKNOWN,      // 未知错误
    I2C_ERR_INIT_FAILED,  // 初始化失败
} I2C_Status_t;

/**
 * @brief I2C 总线配置与句柄结构体
 * @note  描述一条物理I2C总线，包含其GPIO引脚和并发保护锁
 */
typedef struct {
    GPIO_TypeDef* scl_port;
    uint16_t      scl_pin;
    GPIO_TypeDef* sda_port;
    uint16_t      sda_pin;
    void* mutex; // 用于保护总线访问的互斥信号量
} I2C_Bus_t;

/**
 * @brief I2C 总线高级操作接口
 * @note  提供给上层设备驱动使用的、封装了完整I2C时序的API
 */
typedef struct {
    I2C_Status_t (*Init)(I2C_Bus_t* bus);
    I2C_Status_t (*Write)(I2C_Bus_t* bus, uint8_t dev_addr, const uint8_t* data, uint16_t len);
    I2C_Status_t (*Read)(I2C_Bus_t* bus, uint8_t dev_addr, uint8_t* data, uint16_t len);
    I2C_Status_t (*WriteAndRead)(I2C_Bus_t* bus, uint8_t dev_addr, const uint8_t* w_data, uint16_t w_len, uint8_t* r_data, uint16_t r_len);
} I2C_Bus_Ops_t;

// 声明全局唯一的总线操作函数集
extern const I2C_Bus_Ops_t g_i2c_bus_ops;

#endif // __I2C_DRIVER_H
