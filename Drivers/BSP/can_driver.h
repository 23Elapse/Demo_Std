/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 20:01:43
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-14 18:21:16
 * @FilePath: \Demo_backup\Drivers\BSP\can_driver.h
 * @Description: CAN 驱动头文件 (Refactored)
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __CAN_DRIVER_H
#define __CAN_DRIVER_H

#include "ring_buffer.h"
#include "stm32f4xx.h"
#include "common_driver.h" // 包含公共驱动，用于GPIO初始化等

#define CAN_RX_BUFFER_CAPACITY 16 // CAN接收环形缓冲区的容量

/**
 * @brief CAN 操作状态枚举
 */
typedef enum
{
    CAN_OK = 0,
    CAN_ERROR_INIT,
    CAN_ERROR_TRANSMIT,
    CAN_ERROR_BUFFER_FULL,
    CAN_ERROR_NO_DATA,
    CAN_ERROR_PARAM,
    CAN_ERROR_MUTEX_TIMEOUT
} CAN_Status_t;

/**
 * @brief CAN 消息结构体
 */
typedef struct
{
    uint32_t id;     // 消息 ID (标准或扩展)
    uint8_t data[8]; // 数据内容
    uint8_t length;  // 数据长度 (0-8)
    uint8_t ide;     // 帧类型 (CAN_Id_Standard 或 CAN_Id_Extended)
    uint8_t rtr;     // 帧类型 (CAN_RTR_Data 或 CAN_RTR_Remote)
} CAN_Message_t;

/**
 * @brief CAN 设备结构体 (包含配置和运行时数据)
 */
typedef struct
{
    CAN_TypeDef *instance;  // CAN 外设实例 (如 CAN1, CAN2)
    GPIO_TypeDef *tx_port;  // TX GPIO 端口
    uint16_t tx_pin;        // TX GPIO 引脚
    GPIO_TypeDef *rx_port;  // RX GPIO 端口
    uint16_t rx_pin;        // RX GPIO 引脚
    uint32_t baudrate;      // 波特率 (例如 500000)
    uint8_t af;             // GPIO 复用功能 (例如 GPIO_AF_CAN1)
    IRQn_Type irqn;         // 中断号 (例如 CAN1_RX0_IRQn)

    RingBuffer_t rx_buffer; // 接收环形缓冲区
    void *mutex;            // 用于保护发送操作的互斥锁 (可选，如果无需多任务发送可不使用)
} CAN_Device_t;

/**
 * @brief CAN 操作接口
 * @note  提供统一的CAN操作函数指针集合
 */
typedef struct
{
    CAN_Status_t (*Init)(CAN_Device_t *dev);                                      // 初始化CAN设备
    CAN_Status_t (*Deinit)(CAN_Device_t *dev);                                    // 反初始化CAN设备
    CAN_Status_t (*SendMessage)(CAN_Device_t *dev, const CAN_Message_t *msg);     // 发送CAN消息
    CAN_Status_t (*ReceiveMessage)(CAN_Device_t *dev, CAN_Message_t *msg, uint32_t timeout_ms); // 接收CAN消息 (带超时)
} CAN_Ops_t;

// 声明全局唯一的CAN操作接口实例
extern const CAN_Ops_t g_can_ops;

/**
 * @brief CAN 中断服务总处理函数
 * @note  应在STM32中断向量表中的CANx_RX0_IRQHandler内调用此函数
 * @param dev 发生中断的CAN设备实例
 */
void CAN_IRQHandler(CAN_Device_t *dev);

#endif // __CAN_DRIVER_H
