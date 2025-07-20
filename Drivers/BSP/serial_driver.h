/**
 * =====================================================================================
 * @file        serial_driver.h
 * @brief       通用串口驱动头文件 (Refactored)
 * @author      23Elapse & Gemini
 * @version     2.1 (Refactored)
 * @date        2025-06-14
 * @note        本驱动为所有串口通信提供底层硬件操作。
 * =====================================================================================
 */
#ifndef __SERIAL_DRIVER_H
#define __SERIAL_DRIVER_H

#include "stm32f4xx.h"
#include "ring_buffer.h"
#include "rtos_abstraction.h" // For mutex and delay

// 串口接收缓冲区默认大小
#define SERIAL_RX_BUFFER_DEFAULT_SIZE 512

// 串口操作状态码
typedef enum {
    SERIAL_OK = 0,
    SERIAL_ERR_PARAM,        // 参数错误
    SERIAL_ERR_INIT,         // 初始化失败
    SERIAL_ERR_TRANSMIT,     // 发送失败
    SERIAL_ERR_BUFFER_FULL,  // 接收缓冲区满
    SERIAL_ERR_NO_DATA,      // 没有可读取的数据
    SERIAL_ERR_FRAME,        // 帧错误 (校验、溢出等)
    SERIAL_ERR_MUTEX_TIMEOUT // 获取互斥锁超时
} Serial_Status_t;

// 串口工作模式
typedef enum {
    UART_MODE,  // 普通UART模式
    RS485_MODE  // RS485模式，需要DE引脚控制收发
} Serial_Mode_t;

// 串口设备结构体 (作为配置和句柄)
typedef struct {
    // 硬件配置
    USART_TypeDef* instance;  // USART 外设实例 (如 USART1, USART2)
    GPIO_TypeDef* tx_port;    // TX GPIO 端口
    uint16_t       tx_pin;    // TX GPIO 引脚
    GPIO_TypeDef* rx_port;    // RX GPIO 端口
    uint16_t       rx_pin;    // RX GPIO 引脚
    GPIO_TypeDef* de_port;    // RS485 DE/RE 控制引脚, UART模式下为NULL
    uint16_t       de_pin;    // RS485 DE/RE 引脚
    uint32_t       baudrate;  // 波特率
    uint8_t        af;        // GPIO复用功能 (例如 GPIO_AF_USART1)
    IRQn_Type      irqn;      // 中断号 (例如 USART1_IRQn)

    // 运行时数据
    RingBuffer_t   rx_buffer;    // 接收环形缓冲区
    Serial_Mode_t  mode;         // 工作模式
    void* tx_mutex;     // 用于保护发送的互斥锁
    void* rx_semaphore; // 用于通知有新数据的信号量 (可选，如果由环形缓冲区内部处理则不需要)
    // uint32_t       silent_ticks; // RS485发送后的静默时间 (如果需要精确控制总线反转时间)
} Serial_Device_t;

// --- 公共API ---

/**
 * @brief 初始化一个串口设备
 * @param dev 指向要初始化的设备结构体
 * @return Serial_Status_t 操作状态
 */
Serial_Status_t Serial_Driver_Init(Serial_Device_t* dev);

/**
 * @brief 反初始化一个串口设备
 * @param dev 指向要反初始化的设备结构体
 * @return Serial_Status_t 操作状态
 */
Serial_Status_t Serial_Driver_Deinit(Serial_Device_t* dev);

/**
 * @brief 发送数据 (阻塞方式)
 * @param dev 指向设备结构体
 * @param data 要发送的数据缓冲区
 * @param length 数据长度
 * @return Serial_Status_t 操作状态
 */
Serial_Status_t Serial_Driver_SendData(Serial_Device_t* dev, const uint8_t* data, uint32_t length);

/**
 * @brief 从串口接收数据 (阻塞，带超时)
 * @param dev 指向设备结构体
 * @param data 接收数据缓冲区
 * @param length 期望接收的字节数
 * @param timeout_ms 超时时间 (毫秒)
 * @return Serial_Status_t 操作状态
 */
Serial_Status_t Serial_Driver_ReceiveData(Serial_Device_t* dev, uint8_t* data, uint32_t length, uint32_t timeout_ms);

/**
 * @brief 从串口环形缓冲区读取一个字节 (非阻塞)
 * @param dev 指向设备结构体
 * @param data 接收字节存储指针
 * @return Serial_Status_t 操作状态 (SERIAL_OK 或 SERIAL_ERR_NO_DATA)
 */
Serial_Status_t Serial_Driver_ReadByte(Serial_Device_t* dev, uint8_t* data);


/**
 * @brief 串口中断服务总处理函数
 * @note  在具体的stm32f4xx_it.c中的USARTx_IRQHandler内调用此函数
 * @param dev 发生中断的设备实例
 */
void Serial_Driver_IRQHandler(Serial_Device_t* dev);

#endif // __SERIAL_DRIVER_H
