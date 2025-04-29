/*
 * @Author: Elapse userszy@163.com
 * @Date: 2024-12-14 15:03:29
 * @LastEditors: Elapse userszy@163.com
 * @LastEditTime: 2024-12-14 15:04:24
 * @FilePath: \Demo\Drivers\BSP\Inc\rs485.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef __RS485_H
#define __RS485_H



/******************************************************************************************/
/* RS485 引脚 和 串口 定义 */

#define RS485_TX_GPIO_PORT                  GPIOA
#define RS485_TX_GPIO_PIN                   GPIO_PIN_2
#define RS485_TX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

#define RS485_RX_GPIO_PORT                  GPIOA
#define RS485_RX_GPIO_PIN                   GPIO_PIN_3
#define RS485_RX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

#define RS485_UX                            USART2
#define RS485_UX_IRQn                       USART2_IRQn
#define RS485_UX_IRQHandler                 USART2_IRQHandler
#define RS485_UX_CLK_ENABLE()               do{ __HAL_RCC_USART2_CLK_ENABLE(); }while(0)  /* USART2 时钟使能 */

/******************************************************************************************/

/* 如果想串口中断接收，设置EN_USART2_RX为1，否则设置为0 */
#define EN_USART2_RX               1                /* 0,不接收;1,接收. */
#define RS485_REC_LEN              64               /* 定义最大接收字节数 64 */

extern uint8_t g_rs485_rx_buf[RS485_REC_LEN];       /* 接收缓冲,最大RS485_REC_LEN个字节 */
extern uint8_t g_rs485_rx_cnt;                      /* 接收到的数据长度 */

/******************************************************************************************/

void rs485_init(uint32_t bound);
void rs485_send_data(uint8_t *buf, uint8_t len);
void rs485_receive_data(uint8_t *buf, uint8_t *len);
void rs485_tx_set(uint8_t en);

#endif
