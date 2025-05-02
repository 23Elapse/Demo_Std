/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-05 21:24:07
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-30 14:52:37
 * @FilePath: \Demo\Drivers\BSP\Src\pcf8574.c
 * @Description: PCF8574 IIC 扩展 IO 驱动实现
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */

#include "pcf8574.h"
#include "iic_core.h"
#include "pch.h"
/* PCF8574 的 IIC 操作接口 */
const IIC_Ops_t IIC1_PCF8574 = {
    .dev_addr = PCF8574_ADDR, /* 设备地址（7位地址，左移一位后为 0x40） */
    .ReadByte = PCF8574_ReadBit,
    .WriteByte = PCF8574_WriteBit,
};

/**
 * @brief 初始化 PCF8574 设备
 * @return 0: 成功, 1: 失败
 */
uint8_t pcf8574_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    PCF8574_GPIO_CLK_ENABLE(); /* 使能 GPIOB 时钟 */

    gpio_init_struct.GPIO_Pin = PCF8574_GPIO_PIN;   /* PB12 */
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN;      /* 输入 */
    gpio_init_struct.GPIO_OType = GPIO_OType_PP;    /* 推挽输出 */
    gpio_init_struct.GPIO_PuPd = GPIO_PuPd_UP;      /* 上拉 */
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz; /* 50MHz */

    GPIO_Init(PCF8574_GPIO_PORT, &gpio_init_struct); /* 初始化 PB12 引脚 */

    IIC_INIT(); /* IIC 初始化 */

    /* 检查 PCF8574 是否在位 */
    if (IIC_Check(&IIC1_config, &IIC1_PCF8574) == 0)
    {
        printf("[PCF8574] Device check success, addr: 0x%02X\n", IIC1_PCF8574.dev_addr);
        return 0;
    }
    else
    {
        printf("[PCF8574] Device check failed, addr: 0x%02X\n", IIC1_PCF8574.dev_addr);
        return 1;
    }
}

/**
 * @brief 设置 RS485 发送模式
 * @param en 1: 使能发送, 0: 禁用发送
 */
void rs485_tx_set(uint8_t en)
{
    IIC_Status status = PCF8574_WriteBit(RS485_RE_IO, en);
    if (status != IIC_OK)
    {
        printf("[PCF8574] Failed to set RS485 mode: %d\n", status);
    }
}

/**
 * @brief 设置 PCF8574 某个 IO 的高低电平
 * @param bit 要设置的 IO 编号 (0~7)
 * @param sta IO 状态 (0: 低电平, 1: 高电平)
 * @return IIC 操作状态
 */
IIC_Status PCF8574_WriteBit(uint8_t bit, uint8_t sta)
{
    if (bit > 7)
    {
        printf("[PCF8574] Invalid bit number: %d\n", bit);
        return IIC_ERR_INIT;
    }

    uint8_t data = 0;
    if (PCF8574_ReadByte(&data) != 0)
    {
        printf("[PCF8574] Failed to read current state\n");
        return IIC_ERR_READ;
    }

    if (sta == 0)
    {
        data &= ~(1 << bit);
    }
    else
    {
        data |= (1 << bit);
    }

    PCF8574_WriteByte(data);
    return IIC_OK;
}

/**
 * @brief 读取 PCF8574 某个 IO 的状态
 * @param bit 要读取的 IO 编号 (0~7)
 * @param bit_val 读取到的状态值 (0: 低电平, 1: 高电平)
 * @return IIC 操作状态
 */
IIC_Status PCF8574_ReadBit(uint8_t bit, uint8_t *bit_val)
{
    if (bit > 7 || !bit_val)
    {
        printf("[PCF8574] Invalid bit number or pointer: %d\n", bit);
        return IIC_ERR_INIT;
    }

    uint8_t data;
    if (PCF8574_ReadByte(&data) != 0)
    {
        printf("[PCF8574] Failed to read IO state\n");
        return IIC_ERR_READ;
    }

    *bit_val = (data & (1 << bit)) ? 1 : 0;
    return IIC_OK;
}

/**
 * @brief 读取 PCF8574 的 8 位 IO 状态
 * @param val 读取到的 8 位状态值
 * @return 0: 成功, 1: 失败
 */
uint8_t PCF8574_ReadByte(uint8_t *val)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !val)
    {
        printf("[PCF8574] Invalid RTOS ops or pointer\n");
        return 1;
    }

    uint8_t instance_id = IIC1;

    if (!rtos_ops->SemaphoreTake(IIC1_config.mutex, 100))
    {
        printf("[PCF8574] Failed to take mutex, timeout after 100ms\n");
        return 1;
    }

    IIC_Status status = IIC_OK;
    status = IIC_Start(instance_id);
    if (status != IIC_OK)
    {
        printf("[PCF8574] Failed to send start signal: %d\n", status);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return 1;
    }

    status = IIC_WriteByte(instance_id, PCF8574_ADDR | 0x01);
    if (status != IIC_OK)
    {
        printf("[PCF8574] Failed to write address (read mode): %d\n", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return 1;
    }

    status = IIC_ReadByte(instance_id, 0, val);
    if (status != IIC_OK)
    {
        printf("[PCF8574] Failed to read byte: %d\n", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return 1;
    }

    IIC_Stop(instance_id);
    rtos_ops->SemaphoreGive(IIC1_config.mutex);
    return 0;
}

/**
 * @brief 设置 PCF8574 的 8 位 IO 状态
 * @param data 要写入的 8 位数据
 */
void PCF8574_WriteByte(uint8_t data)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops)
    {
        printf("[PCF8574] Invalid RTOS ops\n");
        return;
    }

    uint8_t instance_id = IIC1;

    if (!rtos_ops->SemaphoreTake(IIC1_config.mutex, 100))
    {
        printf("[PCF8574] Failed to take mutex, timeout after 100ms\n");
        return;
    }

    IIC_Status status = IIC_OK;
    status = IIC_Start(instance_id);
    if (status != IIC_OK)
    {
        printf("[PCF8574] Failed to send start signal: %d\n", status);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return;
    }

    status = IIC_WriteByte(instance_id, PCF8574_ADDR | 0x00);
    if (status != IIC_OK)
    {
        printf("[PCF8574] Failed to write address (write mode): %d\n", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return;
    }

    status = IIC_WriteByte(instance_id, data);
    if (status != IIC_OK)
    {
        printf("[PCF8574] Failed to write byte: %d\n", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return;
    }

    IIC_Stop(instance_id);
    status = IIC_WaitWriteComplete(instance_id, PCF8574_ADDR);
    if (status != IIC_OK)
    {
        printf("[PCF8574] Write operation timeout: %d\n", status);
    }

    rtos_ops->SemaphoreGive(IIC1_config.mutex);
}

/*
 * 示例用法：
 * 1. 设置 RTOS 抽象层
 * RTOS_SetOps(&FreeRTOS_Ops); // 或 RTThread_Ops
 *
 * 2. 初始化 PCF8574
 * pcf8574_init();
 *
 * 3. 设置 RS485 模式
 * rs485_tx_set(1); // 使能发送
 *
 * 4. 操作 IO
 * PCF8574_WriteBit(BEEP_IO, 1); // 打开蜂鸣器
 * uint8_t val;
 * PCF8574_ReadBit(AP_INT_IO, &val); // 读取 AP3216C 中断引脚状态
 * uint8_t all_ios;
 * PCF8574_ReadByte(&all_ios); // 读取所有 IO 状态
 * PCF8574_WriteByte(0xFF); // 设置所有 IO 为高电平
 */