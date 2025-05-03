/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-02-05 21:24:07
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 15:00:00
 * @FilePath: \Demo\Drivers\BSP\Src\pcf8574.c
 * @Description: PCF8574 IIC 扩展 IO 驱动实现
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "pcf8574.h"
#include "iic_core.h"
#include "log_system.h"
#include "rtos_abstraction.h"
#include "common_driver.h"
#include "pch.h"

/* PCF8574 的 IIC 操作接口 */
const IIC_Ops_t IIC1_PCF8574 = {
    .dev_addr = PCF8574_ADDR,
    .ReadByte = PCF8574_ReadBit,
    .WriteByte = PCF8574_WriteBit,
};

/**
 * @brief 初始化 PCF8574 设备
 * @return 0: 成功, 1: 失败
 */
uint8_t pcf8574_init(void)
{
    // 使用 common_driver 初始化 GPIO
    if (Common_GPIO_Init(PCF8574_GPIO_PORT, PCF8574_GPIO_PIN, GPIO_Mode_IN,
                         GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz, 0) != COMMON_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Failed to init GPIO");
        return 1;
    }

    // IIC 初始化由 IIC_INIT 统一处理
    if (IIC_Check(&IIC1_config, &IIC1_PCF8574) == IIC_OK)
    {
        Log_Message(LOG_LEVEL_INFO, "[PCF8574] Device check success, addr: 0x%02X", IIC1_PCF8574.dev_addr);
        return 0;
    }
    else
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Device check failed, addr: 0x%02X", IIC1_PCF8574.dev_addr);
        return 1;
    }
}

/**
 * @brief 设置 RS485 发送模式
 * @param en 1: 使能发送, 0: 禁用发送
 * @return IIC_Status 操作状态
 */
IIC_Status rs485_tx_set(uint8_t en)
{
    IIC_Status status = PCF8574_WriteBit(RS485_RE_IO, en);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Failed to set RS485 mode: %d", status);
    }
    return status;
}

/**
 * @brief 设置 PCF8574 某个 IO 的高低电平
 * @param bit 要设置的 IO 编号 (0~7)
 * @param sta IO 状态 (0: 低电平, 1: 高电平)
 * @return IIC_Status 操作状态
 */
IIC_Status PCF8574_WriteBit(uint8_t bit, uint8_t sta)
{
    if (bit > 7)
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Invalid bit number: %d", bit);
        return IIC_ERR_INIT;
    }

    uint8_t data = 0;
    if (PCF8574_ReadByte(&data) != 0)
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Failed to read current state");
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

    return PCF8574_WriteByte(data);
}

/**
 * @brief 读取 PCF8574 某个 IO 的状态
 * @param bit 要读取的 IO 编号 (0~7)
 * @param bit_val 读取到的状态值 (0: 低电平, 1: 高电平)
 * @return IIC_Status 操作状态
 */
IIC_Status PCF8574_ReadBit(uint8_t bit, uint8_t *bit_val)
{
    if (bit > 7 || !bit_val)
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Invalid bit number or pointer: %d", bit);
        return IIC_ERR_INIT;
    }

    uint8_t data;
    if (PCF8574_ReadByte(&data) != 0)
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Failed to read IO state");
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
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Invalid RTOS ops or pointer");
        return 1;
    }

    uint8_t instance_id = IIC1;

    if (!rtos_ops->SemaphoreTake(IIC1_config.mutex, 100))
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Failed to take mutex, timeout after 100ms");
        return 1;
    }

    IIC_Status status = IIC_Start(instance_id);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Failed to send start signal: %d", status);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return 1;
    }

    status = IIC_WriteByte(instance_id, PCF8574_ADDR | 0x01);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Failed to write address (read mode): %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return 1;
    }

    status = IIC_ReadByte(instance_id, 0, val);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Failed to read byte: %d", status);
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
 * @return IIC_Status 操作状态
 */
IIC_Status PCF8574_WriteByte(uint8_t data)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops)
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Invalid RTOS ops");
        return IIC_ERR_INIT;
    }

    uint8_t instance_id = IIC1;

    if (!rtos_ops->SemaphoreTake(IIC1_config.mutex, 100))
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Failed to take mutex, timeout after 100ms");
        return IIC_ERR_TIMEOUT;
    }

    IIC_Status status = IIC_Start(instance_id);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Failed to send start signal: %d", status);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }

    status = IIC_WriteByte(instance_id, PCF8574_ADDR | 0x00);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Failed to write address (write mode): %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }

    status = IIC_WriteByte(instance_id, data);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Failed to write byte: %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }

    IIC_Stop(instance_id);
    status = IIC_WaitWriteComplete(instance_id, PCF8574_ADDR);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Write operation timeout: %d", status);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }

    rtos_ops->SemaphoreGive(IIC1_config.mutex);
    return IIC_OK;
}
