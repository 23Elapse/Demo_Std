/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-03-29 17:44:10
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-26 21:49:45
 * @FilePath: \Demo\Drivers\BSP\Inc\my_rs485.h
 * @Description: RS485 驱动头文件
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __MY_RS485_H
#define __MY_RS485_H

#include "serial_driver.h"
#include "ring_buffer.h"
#include "log_system.h"

#define BUFFER_SIZE 512

/**
 * @brief RS485 状态枚举
 */
typedef enum
{
    RS485_OK = 0,
    RS485_ERR_INIT,
    RS485_ERR_TIMEOUT,
    RS485_ERR_TRANSMIT,
    RS485_ERR_BUFFER_FULL,
    RS485_ERR_NO_DATA
} RS485_Status;

/**
 * @brief RS485 接收数据结构体
 */
typedef struct
{
    uint8_t data[32];
    uint32_t length;
} RS485_RxData_t;

/**
 * @brief RS485 设备结构体
 */
typedef struct
{
    Serial_Device_t *serial_dev; // 串口设备实例
    RingBuffer_t tx_buffer;      // 发送缓冲区
    RingBuffer_t rx_buffer;      // 接收缓冲区
} RS485_Device_t;

/**
 * @brief 初始化 RS485 设备
 * @param dev RS485 设备实例
 * @return RS485_Status 操作状态
 */
RS485_Status rs485_init(RS485_Device_t *dev);

/**
 * @brief RS485 中断处理函数
 * @param dev RS485 设备实例
 */
void rs485_irq_handler(RS485_Device_t *dev);

/**
 * @brief 发送数据
 * @param dev RS485 设备实例
 * @return RS485_Status 操作状态
 */
RS485_Status rs485_send(RS485_Device_t *dev);

/**
 * @brief 写入发送缓冲区
 * @param dev RS485 设备实例
 * @param data 要写入的数据
 * @return RS485_Status 操作状态
 */
RS485_Status rs485_write_tx(RS485_Device_t *dev, uint8_t data);

/**
 * @brief 从接收缓冲区读取数据
 * @param dev RS485 设备实例
 * @param data 读取的数据
 * @return RS485_Status 操作状态
 */
RS485_Status rs485_read_rx(RS485_Device_t *dev, uint8_t *data);

/**
 * @brief 清空发送缓冲区
 * @param dev RS485 设备实例
 */
void rs485_clear_tx(RS485_Device_t *dev);

/**
 * @brief 清空接收缓冲区
 * @param dev RS485 设备实例
 */
void rs485_clear_rx(RS485_Device_t *dev);

#endif /* __MY_RS485_H */
