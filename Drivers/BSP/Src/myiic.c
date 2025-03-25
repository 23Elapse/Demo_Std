/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-05 21:24:07
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-03-25 00:56:59
 * @FilePath: \Demo\Drivers\BSP\Src\myiic.c
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
#include "pch.h"

IIC_TypeDef IIC1_Struct = {
    .IIC_x = IIC1,
    .IIC_SCL_PIN = IIC1_SCL_PIN,
    .IIC_SDA_PIN = IIC1_SDA_PIN,
    .IIC_SCL_GPIO_PORT = IIC1_SCL_GPIO_PORT,
    .IIC_SDA_GPIO_PORT = IIC1_SDA_GPIO_PORT
};

void IIC_Init(IIC_TypeDef *IIC_Struct) {
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
    
    // SCL配置为开漏输出
    GPIO_InitStruct.GPIO_Pin = IIC_Struct->IIC_SCL_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;  // 开漏模式
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;  // 开漏输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IIC_Struct->IIC_SCL_GPIO_PORT, &GPIO_InitStruct);
    
    // SDA配置为开漏输出
    GPIO_InitStruct.GPIO_Pin = IIC_Struct->IIC_SDA_PIN;
    GPIO_Init(IIC_Struct->IIC_SDA_GPIO_PORT, &GPIO_InitStruct);
    
    IIC_SCL_1(IIC_Struct->IIC_x);  // 初始拉高
    IIC_SDA_1(IIC_Struct->IIC_x);
}


// 起始信号
void IIC_Start(IIC_TypeDef *IIC_Struct) {
    IIC_SDA_1(IIC_Struct->IIC_x);  // 释放SDA
    IIC_SCL_1(IIC_Struct->IIC_x);
    delay_us(5);
    IIC_SDA_0(IIC_Struct->IIC_x);   // SDA拉低
    delay_us(5);
    IIC_SCL_0(IIC_Struct->IIC_x);  // SCL拉低
}

void IIC_Stop(IIC_TypeDef *IIC_Struct) {
    IIC_SDA_0(IIC_Struct->IIC_x);
    IIC_SCL_0(IIC_Struct->IIC_x);
    delay_us(5);
    IIC_SCL_1(IIC_Struct->IIC_x);
    delay_us(5);
    IIC_SDA_1(IIC_Struct->IIC_x);
}

uint8_t IIC_ReadByte(IIC_TypeDef *IIC_Struct, uint8_t ack) {
    uint8_t i, data = 0;
    IIC_SDA_1(IIC_Struct->IIC_x);  // 释放SDA
    for (i = 0; i < 8; i++) 
    {
        IIC_SCL_1(IIC_Struct->IIC_x);
        data <<= 1;
        delay_us(5);
        if (IIC_SDA_READ(IIC_Struct->IIC_x)) 
        {
            data |= 0x01;
        }
        IIC_SCL_0(IIC_Struct->IIC_x);
        delay_us(5);
    }
    if (ack) 
    {
        IIC_SDA_0(IIC_Struct->IIC_x);  // 发送ACK
    } 
    else 
    {
        IIC_SDA_1(IIC_Struct->IIC_x);  // 发送NACK
    }
    delay_us(5);
    IIC_SCL_1(IIC_Struct->IIC_x);
    delay_us(5);
    IIC_SCL_0(IIC_Struct->IIC_x);
    return data;
}

uint8_t IIC_SendByte(IIC_TypeDef *IIC_Struct, uint8_t data) {
    uint8_t i;
    for (i = 0; i < 8; i++) 
    {
        if (data & 0x80) {
            IIC_SDA_1(IIC_Struct->IIC_x);
        } else {
            IIC_SDA_0(IIC_Struct->IIC_x);
        }
        data <<= 1;
        IIC_SCL_1(IIC_Struct->IIC_x);
        delay_us(5);
        IIC_SCL_0(IIC_Struct->IIC_x);
        delay_us(5);
    }
    IIC_SDA_1(IIC_Struct->IIC_x);  // 释放SDA线
    delay_us(5);
    IIC_SCL_1(IIC_Struct->IIC_x);
    uint8_t ack = IIC_SDA_READ(IIC_Struct->IIC_x);  // 读取ACK
    delay_us(5);
    IIC_SCL_0(IIC_Struct->IIC_x);
    return ack;  // 返回ACK状态
    // if (IIC_SDA_READ(IIC_Struct->IIC_x) == RESET) 
    // {
    //     IIC_SCL_0(IIC_Struct->IIC_x);
    //     return 0;    // 收到ACK
    // } 
    // else 
    // {
    //     IIC_SCL_0(IIC_Struct->IIC_x);
    //     return 1;    // 未收到ACK
    // }
}
// 读取一个字节
uint8_t IIC_ReadOneByte(IIC_TypeDef *IIC_Struct, uint8_t devAddr, uint8_t addr) {
    uint8_t data;
    IIC_Start(IIC_Struct);
    I2C_WAIT_WRITE(IIC_SendByte(IIC_Struct, devAddr));  // 发送设备地址
    I2C_WAIT_WRITE(IIC_SendByte(IIC_Struct, addr));     // 发送地址
    IIC_Start(IIC_Struct);              // 重复起始条件
    I2C_WAIT_WRITE(IIC_SendByte(IIC_Struct, devAddr + 1));  // 发送设备地址
    data = IIC_ReadByte(IIC_Struct, 0);     // 读取数据
    IIC_Stop(IIC_Struct);
    return data;
}
// 写入一个字节
void IIC_WriteOneByte(IIC_TypeDef *IIC_Struct, uint8_t devAddr, uint8_t addr, uint8_t data) {
    IIC_Start(IIC_Struct);
    I2C_WAIT_WRITE(IIC_SendByte(IIC_Struct, devAddr));  // 发送设备地址
    I2C_WAIT_WRITE(IIC_SendByte(IIC_Struct, addr));     // 发送地址
    I2C_WAIT_WRITE(IIC_SendByte(IIC_Struct, data));     // 发送数据
    IIC_Stop(IIC_Struct);
    delay_ms(5);  // 等待写入完成
}
// 读取多个字节
void IIC_ReadBytes(IIC_TypeDef *IIC_Struct, uint8_t devAddr, uint8_t addr, uint8_t *pbuf, uint16_t len) {
    uint16_t i;
    IIC_Start(IIC_Struct);
    I2C_WAIT_WRITE(IIC_SendByte(IIC_Struct, devAddr));  // 发送设备地址
    I2C_WAIT_WRITE(IIC_SendByte(IIC_Struct, addr));     // 发送地址
    IIC_Start(IIC_Struct);              // 重复起始条件
    I2C_WAIT_WRITE(IIC_SendByte(IIC_Struct, devAddr + 1));  // 发送设备地址
    for (i = 0; i < len; i++) 
    {
        pbuf[i] = IIC_ReadByte(IIC_Struct, i == len - 1 ? 0 : 1);  // 读取数据
    }
    IIC_Stop(IIC_Struct);
}
// 写入多个字节
void IIC_WriteBytes(IIC_TypeDef *IIC_Struct, uint8_t devAddr, uint8_t addr, uint8_t *pbuf, uint16_t len) {
    uint16_t i;
    IIC_Start(IIC_Struct);
    I2C_WAIT_WRITE(IIC_SendByte(IIC_Struct, devAddr));  // 发送设备地址
    I2C_WAIT_WRITE(IIC_SendByte(IIC_Struct, addr));     // 发送地址
    for (i = 0; i < len; i++) 
    {
        I2C_WAIT_WRITE(IIC_SendByte(IIC_Struct, pbuf[i]));  // 发送数据
    }
    IIC_Stop(IIC_Struct);
    delay_ms(5);  // 等待写入完成
}
// 检查设备
uint8_t IIC_CheckDevice(IIC_TypeDef *IIC_Struct, uint8_t devAddr, uint16_t timeout) {
    uint8_t ack;

    IIC_Start(IIC_Struct);  // 发送起始条件
    ack = IIC_SendByte(IIC_Struct, devAddr);  // 发送设备地址

    // 等待ACK或超时
    while (ack && timeout) {
        ack = IIC_SendByte(IIC_Struct, devAddr);  // 重试发送设备地址
        timeout--;
    }

    IIC_Stop(IIC_Struct);  // 发送停止条件

    if (timeout == 0) {
        return 0xFF;  // 超时错误
    }

    return ack;  // 返回ACK状态
}


