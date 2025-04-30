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


typedef struct {
    uint8_t sof;
    uint8_t addr1;
    uint8_t addr2;
    uint8_t cmd;
    uint8_t cmd_sub;
    uint8_t length;
    uint8_t *info;
    uint16_t info_len;
    uint16_t crc;
} RS485_Frame_t;

typedef struct {
    union {
        RS485_Frame_t rs485_frame;
        struct {
            uint8_t data[208];
            uint32_t length;
        } uart_data;
    };
    uint8_t is_rs485;
} Serial_RxData_t;

typedef enum {
    FRAME_HEADER_7E = 0x7E,
    FRAME_HEADER_F6 = 0xF6,
    FRAME_HEADER_F7 = 0xF7,
    FRAME_HEADER_52 = 0x52
} Serial_FrameHeader_t;

typedef int (*Serial_FrameHandler_t)(void *dev, Serial_RxData_t *rx_data, uint8_t byte,
                                     uint32_t *index, uint8_t *state, uint8_t *expected_length,
                                     void *xHigherPriorityTaskWoken);

typedef struct {
    Serial_FrameHeader_t header;
    Serial_FrameHandler_t handler;
} FrameHandlerEntry_t;

typedef struct {
    USART_TypeDef* instance;
    GPIO_TypeDef* tx_port;
    uint16_t tx_pin;
    GPIO_TypeDef* rx_port;
    uint16_t rx_pin;
    GPIO_TypeDef* de_port;
    uint16_t de_pin;
    uint32_t baudrate;
    uint8_t af;
    uint8_t irqn;
    Serial_Mode_t mode;
    uint32_t silent_ticks;
    RingBuffer_t rx_buffer;
} Serial_Device_t;

#define RS485_TX_QUEUE_SIZE 10
typedef struct {
    RS485_Frame_t frames[RS485_TX_QUEUE_SIZE];
    uint8_t info_buffers[RS485_TX_QUEUE_SIZE][200];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} RS485_TxFrameQueue_t;

typedef struct {
    Serial_Status (*Init)(Serial_Device_t*);
    Serial_Status (*Deinit)(Serial_Device_t*);
    Serial_Status (*SendData)(Serial_Device_t*, const uint8_t*, uint32_t);
    Serial_Status (*SendFrame)(Serial_Device_t*, RS485_Frame_t*);
    Serial_Status (*ReceiveFromBuffer)(Serial_Device_t*, Serial_RxData_t*, uint32_t);
    Serial_Status (*GetErrorLog)(Serial_ErrorLog_t*, uint32_t);
    Serial_Status (*AddFrameToQueue)(RS485_Frame_t*);
    void (*PollSendRS485)(Serial_Device_t*);
} Serial_Ops_t;

extern const Serial_Ops_t Serial_Operations;

extern Serial_Device_t UART_Device;
extern Serial_Device_t RS485_Device;
extern Serial_ErrorLog_t Serial_ErrorLog;
extern Serial_ErrorLog_t Serial_ErrorLogArray[10];

int HandleFrame_7E(void *dev_ptr, Serial_RxData_t *rx_data, uint8_t byte,
                   uint32_t *index, uint8_t *state, uint8_t *expected_length,
                   void *xHigherPriorityTaskWoken);
int HandleFrame_F6(void *dev_ptr, Serial_RxData_t *rx_data, uint8_t byte,
                   uint32_t *index, uint8_t *state, uint8_t *expected_length,
                   void *xHigherPriorityTaskWoken);
Serial_Status Serial_Deinit(Serial_Device_t *dev);
#endif
