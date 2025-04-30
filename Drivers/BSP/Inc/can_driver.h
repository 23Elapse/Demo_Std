/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 20:01:43
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-29 13:57:24
 * @FilePath: \Demo\Drivers\BSP\Inc\can_driver.h
 * @Description: 
 * 
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved. 
 */
#ifndef __CAN_DRIVER_H
#define __CAN_DRIVER_H

#include "ring_buffer.h"
typedef enum {
    CAN_OK = 0,
    CAN_ERROR_INIT,
    CAN_ERROR_TRANSMIT,
    CAN_ERROR_BUFFER_FULL,
    CAN_ERROR_NO_DATA
} CAN_Status;

typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t length;
} CAN_Message_t;

typedef struct {
    CAN_TypeDef* instance;
    GPIO_TypeDef* tx_port;
    uint16_t tx_pin;
    GPIO_TypeDef* rx_port;
    uint16_t rx_pin;
    uint32_t baudrate;
    uint8_t af;
    uint8_t irqn;
    RingBuffer_t rx_buffer;
} CAN_Device_t;

typedef struct {
    CAN_Status (*Init)(CAN_Device_t*);
    CAN_Status (*Deinit)(CAN_Device_t*);
    CAN_Status (*SendMessage)(CAN_Device_t*, CAN_Message_t*);
    CAN_Status (*ReceiveMessage)(CAN_Device_t*, CAN_Message_t*, uint32_t timeout);
} CAN_Ops_t;

extern const CAN_Ops_t CAN_Operations;

#endif /* __CAN_DRIVER_H */
