#ifndef __MY_RS485_H
#define __MY_RS485_H
#include "pch.h"
#define BUFFER_SIZE 512
//typedef struct {
//    uint8_t buffer[BUFFER_SIZE];
//    uint16_t head; // 写指针
//    uint16_t tail; // 读指针
//} RingBuffer;

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
