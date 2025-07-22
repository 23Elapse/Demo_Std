/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-05 21:24:07
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-14 18:58:07
 * @FilePath: \Demo_backup\Drivers\BSP\pcf8574.c
 * @Description: PCF8574 IIC 扩展 IO 驱动实现 (Refactored)
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "pcf8574.h"
#include "log_system.h" // 引入日志系统

/**
 * @brief  初始化PCF8574设备（逻辑上，主要是测试通信）
 * @param  dev 指向PCF8574设备实例
 * @return I2C_Status_t 通信状态
 */
I2C_Status_t PCF8574_Init(PCF8574_Device_t* dev) {
    if (!dev || !dev->i2c_bus) {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Init: Invalid device or I2C bus pointer.");
        return I2C_ERR_PARAM;
    }

    uint8_t temp_data;
    // 尝试读取一次，确保设备在线并初始化内部状态
    I2C_Status_t status = PCF8574_ReadByte(dev, &temp_data);
    if (status == I2C_OK) {
        dev->current_io_state = temp_data; // 读取成功，缓存当前IO状态
        Log_Message(LOG_LEVEL_INFO, "[PCF8574] Device 0x%02X initialized successfully. Initial state: 0x%02X.", dev->dev_addr, temp_data);
    } else {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Init: Failed to read from device 0x%02X. Status: %d.", dev->dev_addr, status);
    }
    return status;
}

/**
 * @brief  向PCF8574写入一个字节以控制所有8个I/O口
 * @param  dev 指向PCF8574设备实例
 * @param  data 要写入的字节
 * @return I2C_Status_t 通信状态
 */
I2C_Status_t PCF8574_WriteByte(PCF8574_Device_t* dev, uint8_t data) {
    if (!dev || !dev->i2c_bus) {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] WriteByte: Invalid device or I2C bus pointer.");
        return I2C_ERR_PARAM;
    }
    // 直接调用高层I2C总线API进行写入
    I2C_Status_t status = g_i2c_bus_ops.Write(dev->i2c_bus, dev->dev_addr, &data, 1);
    if (status == I2C_OK) {
        dev->current_io_state = data; // 写入成功，更新缓存的IO状态
        Log_Message(LOG_LEVEL_DEBUG, "[PCF8574] Write 0x%02X to device 0x%02X OK.", data, dev->dev_addr);
    } else {
        Log_Message(LOG_LEVEL_WARNING, "[PCF8574] Write 0x%02X to device 0x%02X FAILED. Status: %d.", data, dev->dev_addr, status);
    }
    return status;
}

/**
 * @brief  从PCF8574读取一个字节以获取所有8个I/O口的状态
 * @param  dev 指向PCF8574设备实例
 * @param  p_data 指向存储读取数据的变量
 * @return I2C_Status_t 通信状态
 */
I2C_Status_t PCF8574_ReadByte(PCF8574_Device_t* dev, uint8_t* p_data) {
    if (!dev || !dev->i2c_bus || !p_data) {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] ReadByte: Invalid device, I2C bus, or data pointer.");
        return I2C_ERR_PARAM;
    }
    // 直接调用高层I2C总线API进行读取
    I2C_Status_t status = g_i2c_bus_ops.Read(dev->i2c_bus, dev->dev_addr, p_data, 1);
    if (status == I2C_OK) {
        dev->current_io_state = *p_data; // 读取成功，更新缓存的IO状态
        Log_Message(LOG_LEVEL_DEBUG, "[PCF8574] Read 0x%02X from device 0x%02X OK.", *p_data, dev->dev_addr);
    } else {
        Log_Message(LOG_LEVEL_WARNING, "[PCF8574] Read from device 0x%02X FAILED. Status: %d.", dev->dev_addr, status);
    }
    return status;
}
