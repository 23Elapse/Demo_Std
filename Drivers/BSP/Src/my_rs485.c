/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-03-29 17:44:10
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 00:21:45
 * @FilePath: \Demo\Drivers\BSP\Src\my_rs485.c
 * @Description: RS485 驱动实现
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "my_rs485.h"
#include "pcf8574.h"
#include "serial_driver.h"
#include "ring_buffer.h"
#include "log_system.h"
#include "rtos_abstraction.h"
#include "pch.h"
/**
 * @brief 初始化 RS485 设备
 * @param dev RS485 设备实例
 * @return RS485_Status 操作状态
 */
RS485_Status rs485_init(RS485_Device_t *dev)
{
    if (!dev || !dev->serial_dev)
    {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] Invalid device or serial device");
        return RS485_ERR_INIT;
    }

    if (RingBuffer_Init(&dev->tx_buffer, BUFFER_SIZE, sizeof(uint8_t)) != RB_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] Failed to init tx buffer");
        return RS485_ERR_INIT;
    }

    if (RingBuffer_Init(&dev->rx_buffer, BUFFER_SIZE, sizeof(uint8_t)) != RB_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] Failed to init rx buffer");
        return RS485_ERR_INIT;
    }

    if (pcf8574_init() != 0)
    {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] Failed to init PCF8574");
        return RS485_ERR_INIT;
    }

    if (Serial_Driver_Init(dev->serial_dev) != SERIAL_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] Failed to init serial driver");
        return RS485_ERR_INIT;
    }

    if (rs485_tx_set(0) != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] Failed to set RX mode");
        return RS485_ERR_INIT;
    }

    Log_Message(LOG_LEVEL_INFO, "[RS485] Initialized successfully");
    return RS485_OK;
}

/**
 * @brief RS485 中断处理函数
 * @param dev RS485 设备实例
 */
void rs485_irq_handler(RS485_Device_t *dev)
{
    if (!dev || !dev->serial_dev)
    {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] Invalid device in IRQ");
        return;
    }

    if (USART_GetITStatus(dev->serial_dev->instance, USART_IT_RXNE) != RESET)
    {
        uint8_t data = USART_ReceiveData(dev->serial_dev->instance);
        if (RingBuffer_Write(&dev->rx_buffer, &data) != RB_OK)
        {
            Log_Message(LOG_LEVEL_WARNING, "[RS485] RX buffer full");
        }
        USART_ClearITPendingBit(dev->serial_dev->instance, USART_IT_RXNE);
    }
}

/**
 * @brief 发送数据
 * @param dev RS485 设备实例
 * @return RS485_Status 操作状态
 */
RS485_Status rs485_send(RS485_Device_t *dev)
{
    if (!dev || !dev->serial_dev)
    {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] Invalid device");
        return RS485_ERR_INIT;
    }

    if (RingBuffer_IsEmpty(&dev->tx_buffer))
    {
        return RS485_ERR_NO_DATA;
    }

    if (rs485_tx_set(1) != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] Failed to set TX mode");
        return RS485_ERR_TRANSMIT;
    }

    uint8_t data;
    if (RingBuffer_Read(&dev->tx_buffer, &data) != RB_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] Failed to read TX buffer");
        rs485_tx_set(0);
        return RS485_ERR_TRANSMIT;
    }

    if (Serial_Driver_SendData(dev->serial_dev, &data, 1) != SERIAL_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] Failed to send data");
        rs485_tx_set(0);
        return RS485_ERR_TRANSMIT;
    }

    while (USART_GetFlagStatus(dev->serial_dev->instance, USART_FLAG_TC) == RESET)
        ;
    if (rs485_tx_set(0) != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] Failed to set RX mode");
        return RS485_ERR_TRANSMIT;
    }

    return RS485_OK;
}

/**
 * @brief 写入发送缓冲区
 * @param dev RS485 设备实例
 * @param data 要写入的数据
 * @return RS485_Status 操作状态
 */
RS485_Status rs485_write_tx(RS485_Device_t *dev, uint8_t data)
{
    if (!dev)
    {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] Invalid device");
        return RS485_ERR_INIT;
    }

    if (RingBuffer_Write(&dev->tx_buffer, &data) != RB_OK)
    {
        Log_Message(LOG_LEVEL_WARNING, "[RS485] TX buffer full");
        return RS485_ERR_BUFFER_FULL;
    }

    return RS485_OK;
}

/**
 * @brief 从接收缓冲区读取数据
 * @param dev RS485 设备实例
 * @param data 读取的数据
 * @return RS485_Status 操作状态
 */
RS485_Status rs485_read_rx(RS485_Device_t *dev, uint8_t *data)
{
    if (!dev || !data)
    {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] Invalid device or pointer");
        return RS485_ERR_INIT;
    }

    if (RingBuffer_Read(&dev->rx_buffer, data) != RB_OK)
    {
        return RS485_ERR_NO_DATA;
    }

    return RS485_OK;
}

///**
// * @brief 清空发送缓冲区
// * @param dev RS485 设备实例
// */
//void rs485_clear_tx(RS485_Device_t *dev)
//{
//    if (dev)
//    {
//        RingBuffer_Clear(&dev->tx_buffer);
//    }
//}

///**
// * @brief 清空接收缓冲区
// * @param dev RS485 设备实例
// */
//void rs485_clear_rx(RS485_Device_t *dev)
//{
//    if (dev)
//    {
//        RingBuffer_Clear(&dev->rx_buffer);
//    }
//}

