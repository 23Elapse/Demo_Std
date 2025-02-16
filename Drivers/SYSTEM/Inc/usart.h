/*
 * @Author: Elapse userszy@163.com
 * @Date: 2024-10-26 15:40:32
 * @LastEditors: Elapse userszy@163.com
 * @LastEditTime: 2024-12-23 19:03:43
 * @FilePath: \Demo\Drivers\SYSTEM\Inc\usart.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _USART_H
#define _USART_H

#include "stdio.h"
#include "sys.h"


/*******************************************************************************************************/
/* 引脚 和 串口 定义 
 * 默认是针对USART1的.
 * 注意: 通过修改这12个宏定义,可以支持USART1~UART7任意一个串口.
 */

#define USART_TX_GPIO_PORT              GPIOA
#define USART_TX_GPIO_PIN               GPIO_Pin_9
#define USART_TX_GPIO_AF                GPIO_AF_USART1
#define USART_TX_GPIO_CLK_ENABLE()      do{ RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); }while(0)   /* 发送引脚时钟使能 */

#define USART_RX_GPIO_PORT              GPIOA
#define USART_RX_GPIO_PIN               GPIO_Pin_10
#define USART_RX_GPIO_AF                GPIO_AF_USART1
#define USART_RX_GPIO_CLK_ENABLE()      do{ RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); }while(0)   /* 接收引脚时钟使能 */

#define USART_UX                        USART1
#define USART_UX_IRQn                   USART1_IRQn
#define USART_UX_IRQHandler             USART1_IRQHandler
#define USART_UX_CLK_ENABLE()           do{ RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE); }while(0)  /* USART1 时钟使能 */


#define RX_BUFFER_SIZE 2000  // 接收缓冲区大小
#define TX_BUFFER_SIZE 2000  // 发送缓冲区大小

// 环形缓冲区结构体
typedef struct {
    uint8_t buffer[RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer;
extern RingBuffer rx_buffer;  // 接收缓冲区
extern RingBuffer tx_buffer;  // 发送缓冲区
typedef struct {
    USART_TypeDef* USARTx;       // 串口编号（如 USART1, USART2）
    uint32_t BaudRate;           // 波特率
    uint16_t USART_Mode;         // 模式（发送、接收或两者）
    uint16_t USART_Parity;       // 校验位
    uint16_t USART_StopBits;     // 停止位
    uint16_t USART_WordLength;   // 数据位长度
    uint8_t USART_IRQChannel;    // 中断通道（如 USART1_IRQn）
    uint8_t NVIC_Priority;       // 中断优先级
    GPIO_TypeDef* GPIOx;         // GPIO 端口（如 GPIOA, GPIOB）
    uint16_t GPIO_Pin_TX;        // TX 引脚（如 GPIO_Pin_9）
    uint16_t GPIO_Pin_RX;        // RX 引脚（如 GPIO_Pin_10）
    uint8_t GPIO_AF;            // GPIO 复用功能（如 GPIO_AF_USART1）
} USART_ConfigTypeDef;
uint8_t Get_GPIO_PinSource(uint16_t GPIO_Pin);
void USART_InitWithInterrupt(USART_ConfigTypeDef* USART_Config);
void USART1_Init(uint32_t baudrate);
void USART1_SendString(const char *str);
uint16_t USART1_ReceiveData(uint8_t *buf, uint16_t len);
void USART1_IRQHandler(void);

#endif







