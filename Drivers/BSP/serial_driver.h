/**
 * =====================================================================================
 * @file        serial_driver.h
 * @brief       通用串口驱动头文件 (已优化)
 * @author      23Elapse & Gemini
 * @version     2.0 (Refactored)
 * @date        2025-06-08
 * @note        本驱动为所有串口通信提供底层硬件操作。
 * =====================================================================================
 */
#ifndef __SERIAL_DRIVER_H
#define __SERIAL_DRIVER_H

#include "stm32f4xx.h"
#include "ring_buffer.h"

// 串口操作状态码
typedef enum {
    SERIAL_OK = 0,
    SERIAL_ERR_PARAM,
    SERIAL_ERR_INIT,
    SERIAL_ERR_TRANSMIT,
    SERIAL_ERR_BUFFER_FULL,
    SERIAL_ERR_NO_DATA,
    SERIAL_ERR_FRAME
} Serial_Status;

// 串口工作模式
typedef enum {
    UART_MODE,  // 普通UART模式
    RS485_MODE  // RS485模式，需要DE引脚
} Serial_Mode_t;

// 串口设备结构体 (作为配置和句柄)
typedef struct {
    // 硬件配置
    USART_TypeDef* instance;
    GPIO_TypeDef* tx_port;
    uint16_t       tx_pin;
    GPIO_TypeDef* rx_port;
    uint16_t       rx_pin;
    GPIO_TypeDef* de_port;      // RS485 DE/RE 控制引脚, UART模式下为NULL
    uint16_t       de_pin;
    uint32_t       baudrate;
    uint8_t        af;           // GPIO复用功能
    IRQn_Type      irqn;         // 中断号

    // 运行时数据
    RingBuffer_t   rx_buffer;    // 接收环形缓冲区
    Serial_Mode_t  mode;         // 工作模式
    uint32_t       silent_ticks; // RS485发送后的静默时间
} Serial_Device_t;

// --- 公共API ---

/**
 * @brief 初始化一个串口设备
 * @param dev 指向要初始化的设备结构体
 * @return Serial_Status_t 操作状态
 */
Serial_Status Serial_Driver_Init(Serial_Device_t* dev);

/**
 * @brief 反初始化一个串口设备
 * @param dev 指向要反初始化的设备结构体
 * @return Serial_Status_t 操作状态
 */
Serial_Status Serial_Driver_Deinit(Serial_Device_t* dev);

/**
 * @brief 发送数据 (阻塞或DMA方式)
 * @param dev 指向设备结构体
 * @param data 要发送的数据
 * @param length 数据长度
 * @return Serial_Status_t 操作状态
 */
Serial_Status Serial_Driver_SendData(Serial_Device_t* dev, const uint8_t* data, uint32_t length);

/**
 * @brief 串口中断服务总处理函数
 * @note  在具体的stm32f4xx_it.c中的USARTx_IRQHandler内调用此函数
 * @param dev 发生中断的设备实例
 */
void Serial_Driver_IRQHandler(Serial_Device_t* dev);

#endif // __SERIAL_DRIVER_H
