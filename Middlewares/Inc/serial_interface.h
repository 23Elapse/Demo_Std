/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-05-02 21:30:06
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 01:12:22
 * @FilePath: \Demo\Drivers\BSP\Inc\serial_interface.h
 * @Description:
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __SERIAL_INTERFACE_H
#define __SERIAL_INTERFACE_H

#include "serial_driver.h"
#include "protocol_handler.h"

/**
 * @brief 错误日志类型枚举
 */
typedef enum
{
    ERROR_CRC = 0,
    ERROR_TIMEOUT,
    ERROR_FRAME
} Serial_ErrorType_t;

/**
 * @brief 错误日志结构体
 */
typedef struct
{
    Serial_ErrorType_t type;
    uint32_t timestamp;
    USART_TypeDef *instance;
} Serial_ErrorLog_t;

#define RS485_TX_QUEUE_SIZE 10
typedef struct
{
    RS485_Frame_t frames[RS485_TX_QUEUE_SIZE];
    uint8_t info_buffers[RS485_TX_QUEUE_SIZE][200];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} RS485_TxFrameQueue_t;

typedef struct
{
    Serial_Status (*Init)(Serial_Device_t *);
    Serial_Status (*Deinit)(Serial_Device_t *);
    Serial_Status (*SendData)(Serial_Device_t *, const uint8_t *, uint32_t);
    Serial_Status (*SendFrame)(Serial_Device_t *, RS485_Frame_t *);
    Serial_Status (*ReceiveFromBuffer)(Serial_Device_t *, Protocol_Data_t *, uint32_t);
    Serial_Status (*GetErrorLog)(Serial_ErrorLog_t *, uint32_t);
    Serial_Status (*AddFrameToQueue)(RS485_Frame_t *);
    void (*PollSendRS485)(Serial_Device_t *);
} Serial_Ops_t;

extern const Serial_Ops_t Serial_Operations;

#endif /* __SERIAL_INTERFACE_H */
