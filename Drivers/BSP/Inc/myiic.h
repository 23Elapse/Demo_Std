/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-05 21:24:07
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-03-29 15:25:51
 * @FilePath: \Demo\Drivers\BSP\Inc\myiic.h
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef __MYIIC_H
#define __MYIIC_H

#include "pch.h"
typedef struct {
    uint8_t IIC_x;
    uint8_t IIC_SCL_PIN;
    uint8_t IIC_SDA_PIN;
    GPIO_TypeDef *IIC_SCL_GPIO_PORT;
    GPIO_TypeDef *IIC_SDA_GPIO_PORT;
} IIC_TypeDef;

extern IIC_TypeDef IIC1_Struct;


void IIC_Init(IIC_TypeDef *IIC_Struct);
void IIC_Start(IIC_TypeDef *IIC_Struct);
void IIC_Stop(IIC_TypeDef *IIC_Struct);
uint8_t IIC_WriteByte(IIC_TypeDef *IIC_Struct, uint8_t data);
uint8_t IIC_ReadByte(IIC_TypeDef *IIC_Struct, uint8_t ack);
uint8_t IIC_ReadOneByte(IIC_TypeDef *IIC_Struct, uint8_t devAddr, uint8_t addr);
void IIC_WriteOneByte(IIC_TypeDef *IIC_Struct, uint8_t devAddr, uint8_t addr, uint8_t data);
void IIC_ReadBytes(IIC_TypeDef *IIC_Struct, uint8_t devAddr, uint8_t addr, uint8_t *pbuf, uint16_t len);
void IIC_WriteBytes(IIC_TypeDef *IIC_Struct, uint8_t devAddr, uint8_t addr, uint8_t *pbuf, uint16_t len);
uint8_t IIC_CheckDevice(IIC_TypeDef *IIC_Struct, uint8_t devAddr, uint16_t timeout);

#endif

