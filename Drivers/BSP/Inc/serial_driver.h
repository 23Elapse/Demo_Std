/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 21:34:53
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-02 21:28:41
 * @FilePath: \Demo\Drivers\BSP\Inc\serial_driver.h
 * @Description:
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __SERIAL_DRIVER_H
#define __SERIAL_DRIVER_H

#include "stdint.h"
#include "stdbool.h"
#include "stm32f4xx.h"
#include "ring_buffer.h"

/**
 * @brief 串口模式枚举
 */
typedef enum
{
    RS485_MODE = 0,
    UART_MODE
} Serial_Mode_t;

/**
 * @brief 串口状态枚举
 */
typedef enum
{
    SERIAL_OK = 0,
    SERIAL_ERR_INIT,
    SERIAL_ERR_TIMEOUT,
    SERIAL_ERR_TRANSMIT,
    SERIAL_ERR_BUFFER_FULL,
    SERIAL_ERR_FRAME,
    SERIAL_ERR_NO_DATA
} Serial_Status;

/**
 * @brief 串口设备结构体
 */
typedef struct
{
    USART_TypeDef *instance;
    GPIO_TypeDef *tx_port;
    uint16_t tx_pin;
    GPIO_TypeDef *rx_port;
    uint16_t rx_pin;
    GPIO_TypeDef *de_port;
    uint16_t de_pin;
    uint32_t baudrate;
    uint8_t af;
    uint8_t irqn;
    Serial_Mode_t mode;
    uint32_t silent_ticks;
    RingBuffer_t rx_buffer;
} Serial_Device_t;

Serial_Status Serial_Driver_Init(Serial_Device_t *dev);
Serial_Status Serial_Driver_Deinit(Serial_Device_t *dev);
Serial_Status Serial_Driver_SendData(Serial_Device_t *dev, const uint8_t *data, uint32_t length);
Serial_Status Serial_Driver_ReceiveByte(Serial_Device_t *dev, uint8_t *byte, void *xHigherPriorityTaskWoken);

void Serial_Driver_IRQHandler(Serial_Device_t *dev);

#endif /* __SERIAL_DRIVER_H */