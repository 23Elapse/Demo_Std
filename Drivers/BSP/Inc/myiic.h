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

#ifdef IIC_DEBUG
#define LOG_IIC_EVENT(dev, event) printf("[IIC%d] %s\n", dev->instance_id, event)
#else
#define LOG_IIC_EVENT(dev, event)
#endif
/* 设备状态码 */
// typedef enum {
//     IIC_OK = 0,
//     IIC_ERR_INIT,
//     IIC_ERR_TIMEOUT,
//     IIC_ERR_NACK,
//     IIC_ERR_BUS_BUSY
// } IIC_Status;

#define IIC1                  1
#define IIC2                  2
#define IIC1_SCL_GPIO_PORT    GPIOH
#define IIC1_SCL_PIN          GPIO_Pin_4
#define IIC1_SDA_GPIO_PORT    GPIOH
#define IIC1_SDA_PIN          GPIO_Pin_5

#define IIC2_SCL_GPIO_PORT    GPIOB
#define IIC2_SCL_PIN          GPIO_Pin_10
#define IIC2_SDA_GPIO_PORT    GPIOB
#define IIC2_SDA_PIN          GPIO_Pin_11

/* IO 操作 */ 
#define IIC_SCL_1(IIC_x)        do{ if(IIC_x == IIC1) GPIO_SetBits(IIC1_SCL_GPIO_PORT, IIC1_SCL_PIN); else GPIO_SetBits(IIC2_SCL_GPIO_PORT, IIC2_SCL_PIN);; }while(0)    /* SCL */
#define IIC_SCL_0(IIC_x)        do{ if(IIC_x == IIC1) GPIO_ResetBits(IIC1_SCL_GPIO_PORT, IIC1_SCL_PIN); else GPIO_ResetBits(IIC2_SCL_GPIO_PORT, IIC2_SCL_PIN); }while(0)    /* SCL */
#define IIC_SDA_1(IIC_x)        do{ if(IIC_x == IIC1) GPIO_SetBits(IIC1_SDA_GPIO_PORT, IIC1_SDA_PIN); else GPIO_SetBits(IIC2_SDA_GPIO_PORT, IIC2_SDA_PIN); }while(0)    /* SDA */
#define IIC_SDA_0(IIC_x)        do{ if(IIC_x == IIC1) GPIO_ResetBits(IIC1_SDA_GPIO_PORT, IIC1_SDA_PIN); else GPIO_ResetBits(IIC2_SDA_GPIO_PORT, IIC2_SDA_PIN); }while(0)    /* SDA */
#define IIC_SDA_READ(IIC_x)     ((IIC_x == IIC1) ? GPIO_ReadInputDataBit(IIC1_SDA_GPIO_PORT, IIC1_SDA_PIN) : \
                                    GPIO_ReadInputDataBit(IIC2_SDA_GPIO_PORT, IIC2_SDA_PIN))    /* 读SDA */ 


                                    
#define I2C_TIMEOUT 1000   // I2C超时时间
#define I2C_ERROR_TIMEOUT 0xFF
// 轮询带超时的I2C写入
#define I2C_WAIT_WRITE(cmd) do { \
    uint16_t timeout = I2C_TIMEOUT; \
    while (cmd) if (--timeout == 0) return I2C_ERROR_TIMEOUT; \
} while (0)

typedef struct {
    uint8_t IIC_x;
    uint8_t IIC_SCL_PIN;
    uint8_t IIC_SDA_PIN;
    GPIO_TypeDef *IIC_SCL_GPIO_PORT;
    GPIO_TypeDef *IIC_SDA_GPIO_PORT;
} IIC_TypeDef;

extern IIC_TypeDef IIC1_Struct;

// typedef struct {
//     uint8_t instance_id;       // 设备实例标识
//     GPIO_TypeDef* scl_port;    // SCL端口
//     uint16_t scl_pin;          // SCL引脚
//     GPIO_TypeDef* sda_port;    // SDA端口
//     uint16_t sda_pin;          // SDA引脚
//     uint8_t dev_addr;          // 设备地址（7位）
//     uint32_t timeout;          // 超时时间
//     void*           user_data;   // 用户扩展数据
// } IIC_Device_t;

// typedef struct {
//     void (*Init)(IIC_Device_t*);
//     uint8_t (*ReadByte)(IIC_Device_t*, uint8_t reg);
//     void (*WriteByte)(IIC_Device_t*, uint8_t reg, uint8_t data);
// } IIC_Ops_t;

// static const IIC_Ops_t IIC_Operations = {
//     .Init = IIC_Init,
//     .ReadByte = IIC_ReadByte,
//     .WriteByte = IIC_WriteByte
// };

// /* 操作接口函数指针类型 */
// typedef IIC_Status (*IIC_InitFunc)(struct IIC_Device*);
// typedef IIC_Status (*IIC_ReadFunc)(struct IIC_Device*, uint8_t reg, uint8_t* val);
// typedef IIC_Status (*IIC_WriteFunc)(struct IIC_Device*, uint8_t reg, uint8_t val);

// /* 统一操作接口结构体 */
// typedef struct {
//     IIC_InitFunc    Init;
//     IIC_ReadFunc    ReadByte;
//     IIC_WriteFunc   WriteByte;
// } IIC_Ops_t;

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

