/**
 * =====================================================================================
 * @file        rs485_driver.h
 * @brief       RS485协议层驱动头文件
 * @author      23Elapse & Gemini
 * @version     2.0 (Refactored)
 * @date        2025-06-08
 * @note        本驱动工作在串口驱动之上，主要负责控制DE/RE引脚时序。
 * =====================================================================================
 */
#ifndef __RS485_DRIVER_H
#define __RS485_DRIVER_H

#include "serial_driver.h" // 依赖底层串口驱动

/**
 * @brief RS485 操作状态枚举
 */
typedef enum {
    RS485_OK = 0,
    RS485_ERR_PARAM,
    RS485_ERR_INIT,
    RS485_ERR_TRANSMIT,
    RS485_ERR_BUFFER_FULL,
} RS485_Status_t;

/**
 * @brief 初始化RS485设备
 * @param serial_dev 指向用于RS485通信的底层串口设备实例
 * @return RS485_Status_t 操作状态
 */
RS485_Status_t RS485_Init(Serial_Device_t* serial_dev);

/**
 * @brief 通过RS485发送一串数据
 * @param serial_dev 指向底层串口设备实例
 * @param data 指向要发送的数据缓冲区
 * @param len 要发送的数据长度
 * @return RS485_Status_t 操作状态
 */
RS485_Status_t RS485_Write(Serial_Device_t* serial_dev, const uint8_t* data, uint16_t len);

// 注意：RS485的读取操作直接通过底层的 serial_dev->rx_buffer 进行，
// 上层应用可以直接调用 RingBuffer_Read() 从该缓冲区读取数据，
// 因此此处不再提供单独的 RS485_Read 函数，以避免不必要的封装和混淆。

#endif // __RS485_DRIVER_H
