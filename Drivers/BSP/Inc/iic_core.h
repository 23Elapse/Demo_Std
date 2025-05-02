/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-03-26 20:19:40
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-30 14:54:40
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
#define IIC_SDA_READ_BIT(IIC_x) ((IIC_x == IIC1) ? GPIO_ReadInputDataBit(IIC1_SDA_GPIO_PORT, IIC1_SDA_PIN) : \
                                    GPIO_ReadInputDataBit(IIC2_SDA_GPIO_PORT, IIC2_SDA_PIN))    /* 读SDA */
#define I2C_TIMEOUT 1000   // I2C超时时间
#define I2C_ERROR_TIMEOUT 0xFF
// 轮询带超时的I2C写入
// #define I2C_WAIT_WRITE(cmd) do { \
//     uint16_t timeout = I2C_TIMEOUT; \
//     while (cmd) if (--timeout == 0) return I2C_ERROR_TIMEOUT; \
// } while (0)
#define SPI_FLASH_WAIT_UNTIL_TIMEOUT(condition)	WAIT_UNTIL_TIMEOUT(condition, 30000);


#define I2C_WAIT_WRITE(cmd) do { \
    uint16_t timeout = I2C_TIMEOUT; \
    IIC_Status status; \
    do { \
        status = (cmd); \
        if (--timeout == 0) { \
            return IIC_ERR_TIMEOUT; \
        } \
    } while (status != IIC_OK); \
    } while (0)


/**
 * @brief IIC 操作状态枚举
 */
typedef enum
{
    IIC_OK = 0,      // 操作成功
    IIC_ERR_INIT,    // 初始化失败
    IIC_ERR_TIMEOUT, // 操作超时
    IIC_ERR_NACK,    // 未收到 ACK
    IIC_ERR_BUS_BUSY, // 总线忙碌
    IIC_ERR_READ     // 读取错误
} IIC_Status;

/**
 * @brief IIC 配置结构体
 */
typedef struct
{
    GPIO_TypeDef *scl_port; // SCL 引脚端口
    GPIO_TypeDef *sda_port; // SDA 引脚端口
    uint16_t scl_pin;       // SCL 引脚
    uint16_t sda_pin;       // SDA 引脚
    uint8_t instance_id;    // IIC 实例 ID
    void *mutex;            // 互斥锁
} IIC_Config_t;

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

/* 全局变量声明 */
extern IIC_Config_t IIC1_config; // IIC1 设备实例
extern const IIC_Ops_t IIC1_EEPROM;

/* 函数声明 */
IIC_Status IICx_Init(IIC_Config_t *IICx);
IIC_Status IIC_ResetBus(IIC_Config_t *IICx);
IIC_Status IIC_Start(uint8_t instance_id);
IIC_Status IIC_Stop(uint8_t instance_id);
IIC_Status IIC_ReadByte(uint8_t instance_id, uint8_t ack, uint8_t *data);
IIC_Status IIC_WriteByte(uint8_t instance_id, uint8_t data);
IIC_Status IIC_WaitWriteComplete(uint8_t instance_id, uint8_t dev_addr);
uint8_t IIC_Check(IIC_Config_t *IICx, const IIC_Ops_t *i2c_dev);
void IIC_INIT(void);
IIC_Status IICx_DevWrite(IIC_Ops_t *i2c_dev, uint8_t reg, uint8_t *buf, uint16_t len);
IIC_Status IICx_DevRead(IIC_Ops_t *i2c_dev, uint8_t reg, uint8_t *buf, uint16_t len);

#endif /* __IIC_CORE_H */
