/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-03-26 20:19:40
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 15:00:00
 * @FilePath: \Demo\Drivers\BSP\Inc\iic_core.h
 * @Description: IIC 核心头文件，支持 RTOS 抽象
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __IIC_CORE_H
#define __IIC_CORE_H

#include "stm32f4xx.h"
#include "stdint.h"
#include "stdbool.h"
#include "rtos_abstraction.h"

#ifdef IIC_DEBUG
#define LOG_IIC_EVENT(IICx, event) printf("[IIC%d] %s\r\n", IICx->instance_id, event)
#else
#define LOG_IIC_EVENT(IICx, event)
#endif

#define IIC1 1
#define IIC2 2
#define IIC1_SCL_GPIO_PORT GPIOH
#define IIC1_SCL_PIN GPIO_Pin_4
#define IIC1_SDA_GPIO_PORT GPIOH
#define IIC1_SDA_PIN GPIO_Pin_5
#define IIC2_SCL_GPIO_PORT GPIOB
#define IIC2_SCL_PIN GPIO_Pin_10
#define IIC2_SDA_GPIO_PORT GPIOB
#define IIC2_SDA_PIN GPIO_Pin_11
#define MAX_IIC_DEVICES 4

/* IO 操作 */
#define IIC_SCL_1(IIC_x)                                    \
    do                                                      \
    {                                                       \
        if (IIC_x == IIC1)                                  \
            GPIO_SetBits(IIC1_SCL_GPIO_PORT, IIC1_SCL_PIN); \
        else                                                \
            GPIO_SetBits(IIC2_SCL_GPIO_PORT, IIC2_SCL_PIN); \
    } while (0)
#define IIC_SCL_0(IIC_x)                                      \
    do                                                        \
    {                                                         \
        if (IIC_x == IIC1)                                    \
            GPIO_ResetBits(IIC1_SCL_GPIO_PORT, IIC1_SCL_PIN); \
        else                                                  \
            GPIO_ResetBits(IIC2_SCL_GPIO_PORT, IIC2_SCL_PIN); \
    } while (0)
#define IIC_SDA_1(IIC_x)                                    \
    do                                                      \
    {                                                       \
        if (IIC_x == IIC1)                                  \
            GPIO_SetBits(IIC1_SDA_GPIO_PORT, IIC1_SDA_PIN); \
        else                                                \
            GPIO_SetBits(IIC2_SDA_GPIO_PORT, IIC2_SDA_PIN); \
    } while (0)
#define IIC_SDA_0(IIC_x)                                      \
    do                                                        \
    {                                                         \
        if (IIC_x == IIC1)                                    \
            GPIO_ResetBits(IIC1_SDA_GPIO_PORT, IIC1_SDA_PIN); \
        else                                                  \
            GPIO_ResetBits(IIC2_SDA_GPIO_PORT, IIC2_SDA_PIN); \
    } while (0)
#define IIC_SDA_READ(IIC_x) ((IIC_x == IIC1) ? GPIO_ReadInputDataBit(IIC1_SDA_GPIO_PORT, IIC1_SDA_PIN) : GPIO_ReadInputDataBit(IIC2_SDA_GPIO_PORT, IIC2_SDA_PIN))
#define I2C_TIMEOUT 1000

/**
 * @brief IIC 操作状态枚举
 */
typedef enum
{
    IIC_OK = 0,
    IIC_ERR_INIT,
    IIC_ERR_TIMEOUT,
    IIC_ERR_NACK,
    IIC_ERR_BUS_BUSY,
    IIC_ERR_READ
} IIC_Status;
/**
 * @brief IIC 操作接口函数指针类型
 */
typedef IIC_Status (*IIC_ReadFunc)(uint8_t reg, uint8_t *val);
typedef IIC_Status (*IIC_WriteFunc)(uint8_t reg, uint8_t val);

/**
 * @brief IIC 统一操作接口结构体
 */
typedef struct
{
    IIC_ReadFunc ReadByte;
    IIC_WriteFunc WriteByte;
    uint8_t dev_addr; // 设备地址（7位）
} IIC_Ops_t;
/**
 * @brief IIC 配置结构体
 */
typedef struct
{
    GPIO_TypeDef *scl_port;
    GPIO_TypeDef *sda_port;
    uint16_t scl_pin;
    uint16_t sda_pin;
    uint8_t instance_id;
    void *mutex;
    IIC_Ops_t *devices[MAX_IIC_DEVICES]; // 支持挂载多个设备
    uint8_t device_count;                // 当前挂载设备数
} IIC_Config_t;

/* 全局变量声明 */
extern IIC_Config_t IIC1_config;
extern IIC_Ops_t IIC1_ops;

/* 函数声明 */
IIC_Status IICx_Init(IIC_Config_t *IICx);
IIC_Status IIC_ResetBus(IIC_Config_t *IICx);
IIC_Status IIC_Start(uint8_t instance_id);
IIC_Status IIC_Stop(uint8_t instance_id);
IIC_Status IIC_ReadByte(uint8_t instance_id, uint8_t ack, uint8_t *data);
IIC_Status IIC_WriteByte(uint8_t instance_id, uint8_t data);
IIC_Status IIC_WaitWriteComplete(uint8_t instance_id, uint8_t dev_addr);
void IIC_INIT(void);
IIC_Status IIC_AttachDevice(IIC_Config_t *IICx, IIC_Ops_t *device);
uint8_t IIC_Check(IIC_Config_t *IICx, const IIC_Ops_t *i2c_dev);
IIC_Status IICx_DevWrite(IIC_Ops_t *i2c_dev, uint8_t reg, uint8_t *buf, uint16_t len);
IIC_Status IICx_DevRead(IIC_Ops_t *i2c_dev, uint8_t reg, uint8_t *buf, uint16_t len);

#endif /* __IIC_CORE_H */
