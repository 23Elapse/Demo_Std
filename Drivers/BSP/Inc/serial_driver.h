#ifndef __SERIAL_DRIVER_H
#define __SERIAL_DRIVER_H


#include "stdint.h"
#include "stdbool.h"
#include "stm32f4xx.h"
#include "ring_buffer.h"
/**
 * @brief  串口模式枚举
 */
typedef enum {
    RS485_MODE = 0,         // RS485模式（带DE引脚）
    UART_MODE               // 标准UART模式（无DE引脚）
} Serial_Mode_t;

/**
 * @brief  串口状态枚举
 */
typedef enum {
    SERIAL_OK = 0,          // 操作成功
    SERIAL_ERR_INIT,        // 初始化失败
    SERIAL_ERR_TIMEOUT,     // 操作超时
    SERIAL_ERR_TRANSMIT,    // 传输失败
    SERIAL_ERR_BUFFER_FULL, // 环形缓冲区满
    SERIAL_ERR_NO_DATA,     // 无数据可读
    SERIAL_ERR_CRC,         // CRC校验失败（RS485）
    SERIAL_ERR_FRAME        // 帧格式错误（UART）
} Serial_Status;

/**
 * @brief  错误日志类型枚举
 */
typedef enum {
    ERROR_CRC = 0,          // RS485 CRC错误
    ERROR_TIMEOUT,          // 发送超时
    ERROR_FRAME             // UART帧格式错误
} Serial_ErrorType_t;

/**
 * @brief  错误日志结构体
 */
typedef struct {
    Serial_ErrorType_t type; // 错误类型
    TickType_t timestamp;    // 时间戳（ticks）
    USART_TypeDef* instance; // 设备实例
} Serial_ErrorLog_t;

/**
 * @brief  串口接收数据结构体
 */
typedef struct {
    uint8_t data[32];       // 数据帧（RS485: Modbus帧，UART: [0xAA][长度][数据][0x55]）
    uint32_t length;        // 实际帧长度
} Serial_RxData_t;

/**
 * @brief  串口设备结构体
 */
typedef struct {
    USART_TypeDef* instance;  // USART实例（如USART1）
    GPIO_TypeDef* tx_port;    // TX引脚端口
    uint16_t tx_pin;          // TX引脚
    GPIO_TypeDef* rx_port;    // RX引脚端口
    uint16_t rx_pin;          // RX引脚
    GPIO_TypeDef* de_port;    // DE引脚端口（RS485模式）
    uint16_t de_pin;          // DE引脚（RS485模式）
    uint32_t baudrate;        // 波特率 (bps)
    uint8_t af;               // GPIO复用功能（如GPIO_AF_USART1）
    uint8_t irqn;             // 中断号（如USART1_IRQn）
    uint8_t slave_addr;       // Modbus从站地址（RS485模式，如0x01）
    Serial_Mode_t mode;       // 串口模式（RS485或UART）
    TIM_TypeDef* timer;       // 定时器实例（RS485模式，如TIM2）
    uint8_t timer_irqn;       // 定时器中断号（如TIM2_IRQn）
    RingBuffer_t rx_buffer;   // 接收环形缓冲区
} Serial_Device_t;

/**
 * @brief  串口操作接口
 */
typedef struct {
    Serial_Status (*Init)(Serial_Device_t*);
    Serial_Status (*Deinit)(Serial_Device_t*);
    Serial_Status (*SendData)(Serial_Device_t*, const uint8_t*, uint32_t);
    Serial_Status (*ReceiveFromBuffer)(Serial_Device_t*, Serial_RxData_t*, TickType_t);
    Serial_Status (*GetErrorLog)(Serial_ErrorLog_t*, TickType_t);
} Serial_Ops_t;

/**
 * @brief  全局变量
 */
static Serial_Device_t *serial_devices[4] = {0}; // 最多4个串口实例
static uint8_t serial_device_count = 0;          // 当前实例数
static RingBuffer_t error_log_buffer;            // 错误日志缓冲区
extern const Serial_Ops_t Serial_Operations;
extern Serial_Device_t RS485_Device;
extern Serial_Device_t UART_Device;
Serial_Status Serial_Deinit(Serial_Device_t *dev);
void Serial_IRQHandler(Serial_Device_t *dev);
void Serial_TIM_IRQHandler(TIM_TypeDef *timer);

#endif
