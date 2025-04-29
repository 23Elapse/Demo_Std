/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-03-29 17:44:10
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-29 09:56:33
 * @FilePath: \Demo\Drivers\BSP\Inc\my_rs485.h
 * @Description: 
 * 
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved. 
 */
#ifndef __MY_RS485_H
#define __MY_RS485_H
// #include "pch.h"
//#include "ring_buffer.h"
#define BUFFER_SIZE 512
/**
 * @brief  RS485状态枚举
 */
typedef enum {
    RS485_OK = 0,           // 操作成功
    RS485_ERR_INIT,         // 初始化失败
    RS485_ERR_TIMEOUT,      // 操作超时
    RS485_ERR_TRANSMIT,     // 传输失败
    RS485_ERR_BUFFER_FULL,  // 环形缓冲区满
    RS485_ERR_NO_DATA       // 无数据可读
} RS485_Status;

/**
 * @brief  RS485接收数据结构体
 */
typedef struct {
    uint8_t data[32];       // 固定长度数据帧（32字节）
    uint32_t length;        // 实际数据长度
} RS485_RxData_t;

/**
 * @brief  RS485设备结构体
 */
typedef struct {
    USART_TypeDef* instance;  // USART实例（如USART1）
    GPIO_TypeDef* tx_port;    // TX引脚端口
    uint16_t tx_pin;          // TX引脚
    GPIO_TypeDef* rx_port;    // RX引脚端口
    uint16_t rx_pin;          // RX引脚
    GPIO_TypeDef* de_port;    // DE引脚端口（方向控制）
    uint16_t de_pin;          // DE引脚
    uint32_t baudrate;        // 波特率 (bps)
    RingBuffer_t rx_buffer;   // 接收环形缓冲区
} RS485_Device_t;

//RingBuffer tx_buffer, rx_buffer;
#define RS485_TX_MODE() GPIO_SetBits(GPIOG, GPIO_Pin_8)
#define RS485_RX_MODE() GPIO_ResetBits(GPIOG, GPIO_Pin_8)

extern USART_ConfigTypeDef USART2_Config;
extern GPIO_ConfigTypeDef GPIO2_Config;

void RS485_GPIO_Init(void) ;
void rs485_init(void) ;
void USART2_IRQHandler(void) ;
void Buffer_Write_Tx(uint8_t data) ;
void Buffer_Write_Rx(uint8_t data) ;
uint8_t Buffer_Read_Rx(void) ;
uint8_t Buffer_Read_Rx(void) ;
void RS485_Send(void) ;
void Buffer_Clear_Tx(void) ;
void Buffer_Clear_Rx(void) ;



#endif
