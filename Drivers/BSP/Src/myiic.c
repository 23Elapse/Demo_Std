/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-17
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-17
 * @FilePath: \Demo\Drivers\BSP\Src\myiic.c
 * @Description: IIC软件驱动 - 全面升级版
 */

#include "pch.h"

// 定义IIC地址处理工具
#define IIC_IS_7BIT_ADDR(addr)   (((addr) & 0x80) == 0)
#define IIC_WRITE_ADDR(addr)     (IIC_IS_7BIT_ADDR(addr) ? ((addr) << 1) : ((addr) & ~0x01))
#define IIC_READ_ADDR(addr)      (IIC_IS_7BIT_ADDR(addr) ? (((addr) << 1) | 0x01) : ((addr) | 0x01))

IIC_TypeDef IIC1_Struct = {
    .IIC_x = IIC1,
    .IIC_SCL_PIN = IIC1_SCL_PIN,
    .IIC_SDA_PIN = IIC1_SDA_PIN,
    .IIC_SCL_GPIO_PORT = IIC1_SCL_GPIO_PORT,
    .IIC_SDA_GPIO_PORT = IIC1_SDA_GPIO_PORT
};

/**
 * @brief IIC总线初始化
 */
void IIC_Init(IIC_TypeDef *IIC_Struct) {
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStruct.GPIO_Pin = IIC_Struct->IIC_SCL_PIN;
    GPIO_Init(IIC_Struct->IIC_SCL_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = IIC_Struct->IIC_SDA_PIN;
    GPIO_Init(IIC_Struct->IIC_SDA_GPIO_PORT, &GPIO_InitStruct);

    IIC_SCL_1(IIC_Struct->IIC_x);
    IIC_SDA_1(IIC_Struct->IIC_x);
}

/**
 * @brief 发送START信号
 */
void IIC_Start(IIC_TypeDef *IIC_Struct) {
    IIC_SDA_1(IIC_Struct->IIC_x);
    IIC_SCL_1(IIC_Struct->IIC_x);
    delay_us(5);
    IIC_SDA_0(IIC_Struct->IIC_x);
    delay_us(5);
    IIC_SCL_0(IIC_Struct->IIC_x);
}

/**
 * @brief 发送STOP信号
 */
void IIC_Stop(IIC_TypeDef *IIC_Struct) {
    IIC_SDA_0(IIC_Struct->IIC_x);
    IIC_SCL_1(IIC_Struct->IIC_x);
    delay_us(5);
    IIC_SDA_1(IIC_Struct->IIC_x);
    delay_us(5);
}

/**
 * @brief 写入一个字节
 */
uint8_t IIC_WriteByte(IIC_TypeDef *IIC_Struct, uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) IIC_SDA_1(IIC_Struct->IIC_x);
        else IIC_SDA_0(IIC_Struct->IIC_x);

        data <<= 1;
        IIC_SCL_1(IIC_Struct->IIC_x);
        delay_us(5);
        IIC_SCL_0(IIC_Struct->IIC_x);
        delay_us(5);
    }

    IIC_SDA_1(IIC_Struct->IIC_x);
    delay_us(5);
    IIC_SCL_1(IIC_Struct->IIC_x);
    uint8_t ack = IIC_SDA_READ(IIC_Struct->IIC_x);
    delay_us(5);
    IIC_SCL_0(IIC_Struct->IIC_x);
    return ack;
}

/**
 * @brief 读取一个字节
 */
uint8_t IIC_ReadByte(IIC_TypeDef *IIC_Struct, uint8_t ack) {
    uint8_t data = 0;

    IIC_SDA_1(IIC_Struct->IIC_x);

    for (uint8_t i = 0; i < 8; i++) {
        IIC_SCL_1(IIC_Struct->IIC_x);
        delay_us(5);
        data <<= 1;
        if (IIC_SDA_READ(IIC_Struct->IIC_x)) data |= 0x01;
        IIC_SCL_0(IIC_Struct->IIC_x);
        delay_us(5);
    }

    if (ack) IIC_SDA_0(IIC_Struct->IIC_x);
    else IIC_SDA_1(IIC_Struct->IIC_x);

    delay_us(5);
    IIC_SCL_1(IIC_Struct->IIC_x);
    delay_us(5);
    IIC_SCL_0(IIC_Struct->IIC_x);

    return data;
}

/**
 * @brief 写入一个字节到指定地址
 */
void IIC_WriteOneByte(IIC_TypeDef *IIC_Struct, uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    IIC_Start(IIC_Struct);
    I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, IIC_WRITE_ADDR(devAddr)));
    I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, regAddr));
    I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, data));
    IIC_Stop(IIC_Struct);
    delay_ms(5);
}

/**
 * @brief 读取指定地址的一个字节
 */
uint8_t IIC_ReadOneByte(IIC_TypeDef *IIC_Struct, uint8_t devAddr, uint8_t regAddr) {
    uint8_t data;
    IIC_Start(IIC_Struct);
    I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, IIC_WRITE_ADDR(devAddr)));
    I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, regAddr));
    IIC_Start(IIC_Struct);
    I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, IIC_READ_ADDR(devAddr)));
    data = IIC_ReadByte(IIC_Struct, 0);
    IIC_Stop(IIC_Struct);
    return data;
}

/**
 * @brief 写入多个字节
 */
void IIC_WriteBytes(IIC_TypeDef *IIC_Struct, uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint16_t len) {
    IIC_Start(IIC_Struct);
    I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, IIC_WRITE_ADDR(devAddr)));
    I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, regAddr));
    for (uint16_t i = 0; i < len; i++) {
        I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, buf[i]));
    }
    IIC_Stop(IIC_Struct);
    delay_ms(5);
}

/**
 * @brief 读取多个字节
 */
void IIC_ReadBytes(IIC_TypeDef *IIC_Struct, uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint16_t len) {
    IIC_Start(IIC_Struct);
    I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, IIC_WRITE_ADDR(devAddr)));
    I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, regAddr));
    IIC_Start(IIC_Struct);
    I2C_WAIT_WRITE(IIC_WriteByte(IIC_Struct, IIC_READ_ADDR(devAddr)));

    for (uint16_t i = 0; i < len; i++) {
        buf[i] = IIC_ReadByte(IIC_Struct, (i != len - 1));
    }

    IIC_Stop(IIC_Struct);
}

/**
 * @brief 检测设备是否存在
 */
uint8_t IIC_CheckDevice(IIC_TypeDef *IIC_Struct, uint8_t devAddr, uint16_t timeout) {
    uint8_t ack;

    do {
        IIC_Start(IIC_Struct);
        ack = IIC_WriteByte(IIC_Struct, IIC_WRITE_ADDR(devAddr));
        IIC_Stop(IIC_Struct);
    } while (ack && (--timeout));

    return timeout ? 0x00 : 0xFF;
}

/**
 * @brief 备用复位总线
 */
void IIC_ResetBus(IIC_TypeDef *IIC_Struct) {
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.GPIO_Pin = IIC_Struct->IIC_SCL_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(IIC_Struct->IIC_SCL_GPIO_PORT, &GPIO_InitStruct);

    for (uint8_t i = 0; i < 9; i++) {
        IIC_SCL_1(IIC_Struct->IIC_x);
        delay_us(5);
        IIC_SCL_0(IIC_Struct->IIC_x);
        delay_us(5);
    }

    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(IIC_Struct->IIC_SCL_GPIO_PORT, &GPIO_InitStruct);
}
