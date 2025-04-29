/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 20:01:43
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-27 20:02:53
 * @FilePath: \Demo\Drivers\BSP\Inc\can_driver.h
 * @Description: 
 * 
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved. 
 */
#ifndef __CAN_DRIVER_H
#define __CAN_DRIVER_H

#include "ring_buffer.h"
/**
 * @brief  CAN状态枚举
 */
typedef enum {
    CAN_OK = 0,           // 操作成功
    CAN_ERR_INIT,         // 初始化失败
    CAN_ERR_TIMEOUT,      // 操作超时
    CAN_ERR_NO_MAILBOX,   // 无可用邮箱
    CAN_ERR_NO_MESSAGE,   // 无待处理消息
    CAN_ERR_TRANSMIT,     // 传输失败
    CAN_ERR_BUFFER_FULL,  // 环形缓冲区满
    CAN_ERR_FIFO_OVERFLOW // FIFO溢出
} CAN_Status;

/**
 * @brief  CAN设备结构体
 */
typedef struct {
    CAN_TypeDef* instance;    // CAN1 或 CAN2
    GPIO_TypeDef* tx_port;    // TX引脚端口
    uint16_t tx_pin;          // TX引脚
    GPIO_TypeDef* rx_port;    // RX引脚端口
    uint16_t rx_pin;          // RX引脚
    uint32_t baudrate;        // 波特率 (bps)
    RingBuffer_t rx_buffer;   // 接收环形缓冲区
    void (*overflow_callback)(void); // FIFO溢出回调函数
} CAN_Device_t;


extern CAN_Device_t CAN1_Device;
extern CAN_Device_t CAN2_Device;
#endif /* __CAN_DRIVER_H */
