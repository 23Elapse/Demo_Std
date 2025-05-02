/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 20:01:43
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 00:54:24
 * @FilePath: \Demo\Drivers\BSP\Inc\can_driver.h
 * @Description: CAN 驱动头文件
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __CAN_DRIVER_H
#define __CAN_DRIVER_H

#include "ring_buffer.h"
#include "log_system.h"
#include "stm32f4xx.h"

/**
 * @brief CAN 状态枚举
 */
typedef enum
{
    CAN_OK = 0,
    CAN_ERROR_INIT,
    CAN_ERROR_TRANSMIT,
    CAN_ERROR_BUFFER_FULL,
    CAN_ERROR_NO_DATA
} CAN_Status;

/**
 * @brief CAN 消息结构体
 */
typedef struct
{
    uint32_t id;     // 消息 ID
    uint8_t data[8]; // 数据内容
    uint8_t length;  // 数据长度
} CAN_Message_t;

/**
 * @brief CAN 设备结构体
 */
typedef struct
{
    CAN_TypeDef *instance;  // CAN 外设实例
    GPIO_TypeDef *tx_port;  // TX GPIO 端口
    uint16_t tx_pin;        // TX GPIO 引脚
    GPIO_TypeDef *rx_port;  // RX GPIO 端口
    uint16_t rx_pin;        // RX GPIO 引脚
    uint32_t baudrate;      // 波特率
    uint8_t af;             // 复用功能
    uint8_t irqn;           // 中断号
    RingBuffer_t rx_buffer; // 接收缓冲区
} CAN_Device_t;

/**
 * @brief CAN 操作接口
 */
typedef struct
{
    CAN_Status (*Init)(CAN_Device_t *);                                      // 初始化
    CAN_Status (*Deinit)(CAN_Device_t *);                                    // 反初始化
    CAN_Status (*SendMessage)(CAN_Device_t *, CAN_Message_t *);              // 发送消息
    CAN_Status (*ReceiveMessage)(CAN_Device_t *, CAN_Message_t *, uint32_t); // 接收消息
} CAN_Ops_t;

extern const CAN_Ops_t CAN_Operations;

#endif /* __CAN_DRIVER_H */