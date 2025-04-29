/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-05 21:24:07
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-29 09:02:08
 * @FilePath: \Demo\Drivers\BSP\Inc\pcf8574.h
 * @Description:    
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef __PCF8574_H
#define __PCF8574_H

#include "iic_core.h"

/******************************************************************************************/
/* 引脚 定义 */

#define PCF8574_GPIO_PORT                  GPIOB
#define PCF8574_GPIO_PIN                   GPIO_Pin_12
#define PCF8574_GPIO_CLK_ENABLE()          do{ RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); }while(0)  /* PB口时钟使能 */

/******************************************************************************************/

#define PCF8574_INT  GPIO_ReadOutputDataBit(PCF8574_GPIO_PORT, PCF8574_GPIO_PIN)              /* PCF8574 INT脚 */


#define PCF8574_ADDR  0X40      /* PCF8574地址(左移了一位) */
/* PCF8574各个IO的功能 */
#define BEEP_IO         0       /* 蜂鸣器控制引脚        P0 */
#define AP_INT_IO       1       /* AP3216C中断引脚       P1 */
#define DCMI_PWDN_IO    2       /* DCMI的电源控制引脚    P2 */
#define USB_PWR_IO      3       /* USB电源控制引脚       P3 */
#define EX_IO           4       /* 扩展IO,自定义使用     P4 */
#define MPU_INT_IO      5       /* SH3001中断引脚        P5 */
#define RS485_RE_IO     6       /* RS485_RE引脚          P6 */
#define ETH_RESET_IO    7       /* 以太网复位引脚        P7 */

/******************************************************************************************/

typedef struct IIC_Config_t IIC_Config_t; // 前向声明
//extern const IIC_Ops_t IIC1_PCF8574; /* IIC操作接口结构体 */
uint8_t pcf8574_init(void); 
void rs485_tx_set(uint8_t en); /* RS485发送模式设置 */

uint8_t PCF8574_ReadByte(uint8_t *data); /* 读取PCF8574的IO状态 */
IIC_Status PCF8574_WriteBit(uint8_t bit, uint8_t sta); /* 设置PCF8574某个IO的高低电平 */
IIC_Status PCF8574_ReadBit(uint8_t reg, uint8_t* bit); /* 读取PCF8574某个IO的高低电平 */
uint8_t PCF8574_ReadByte(uint8_t *data); /* 读取PCF8574的IO状态 */
void PCF8574_WriteByte(uint8_t data); /* 设置PCF8574的IO状态 */






#endif

