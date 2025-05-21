/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-18 20:46:08
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-05 00:59:33
 * @FilePath: \Demo\Middlewares\Src\api_eeprom.c
 * @Description: EEPROM 驱动实现
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "api_eeprom.h"
#include "log_system.h"
#include "rtos_abstraction.h"

const IIC_Ops_t IIC1_EEPROM = {
    .dev_addr = EEPROM_ADDR,
    .ReadByte = EEPROMReadByteFromReg,
    .WriteByte = EEPROMWriteByteToReg};

/**
 * @brief 从指定寄存器读取一个字节
 * @param reg 寄存器地址
 * @param val 读取的数据存储指针
 * @return IIC_Status 操作状态
 */
IIC_Status EEPROMReadByteFromReg(uint8_t reg, uint8_t *val)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !val)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Invalid RTOS ops or pointer");
        return IIC_ERR_INIT;
    }

    if (!rtos_ops->SemaphoreTake(IIC1_config.mutex, 100))
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to take mutex, timeout after 100ms");
        return IIC_ERR_TIMEOUT;
    }

    uint8_t instance_id = IIC1;
    IIC_Status status = IIC_Start(instance_id);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to send start signal: %d", status);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }

#if (EEPROM_TYPE > AT24C16)
    status = IIC_WriteByte(instance_id, EEPROM_ADDR);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write device address: %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }
    status = IIC_WriteByte(instance_id, reg >> 8);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write high address: %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }
#else
    uint8_t devAddr = EEPROM_ADDR | ((reg >> 7) & 0x0E);
    status = IIC_WriteByte(instance_id, devAddr);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write device address: %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }
#endif

    status = IIC_WriteByte(instance_id, reg & 0xFF);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write low address: %d, line is %d", status, __LINE__);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }

    status = IIC_Start(instance_id);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to send repeated start: %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }

    status = IIC_WriteByte(instance_id, EEPROM_ADDR | 0x01);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write read mode address: %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }

    status = IIC_ReadByte(instance_id, 0, val);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to read byte: %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }

    IIC_Stop(instance_id);
    rtos_ops->SemaphoreGive(IIC1_config.mutex);
    Log_Message(LOG_LEVEL_INFO, "[EEPROM] Read byte from reg 0x%02X: 0x%02X", reg, *val);
    return IIC_OK;
}

/**
 * @brief 向指定寄存器写入一个字节
 * @param reg 寄存器地址
 * @param val 要写入的数据
 * @return IIC_Status 操作状态
 */
IIC_Status EEPROMWriteByteToReg(uint8_t reg, uint8_t val)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Invalid RTOS ops");
        return IIC_ERR_INIT;
    }

    if (!rtos_ops->SemaphoreTake(IIC1_config.mutex, 100))
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to take mutex, timeout after 100ms");
        return IIC_ERR_TIMEOUT;
    }

    uint8_t instance_id = IIC1;
    IIC_Status status = IIC_Start(instance_id);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to send start signal: %d", status);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }

#if (EEPROM_TYPE > AT24C16)
    status = IIC_WriteByte(instance_id, EEPROM_ADDR);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write device address: %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }
    status = IIC_WriteByte(instance_id, reg >> 8);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write high address: %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }
#else
    uint8_t devAddr = EEPROM_ADDR | ((reg >> 7) & 0x0E);
    status = IIC_WriteByte(instance_id, devAddr);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write device address: %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }
#endif

    status = IIC_WriteByte(instance_id, reg & 0xFF);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write low address: %d, line is %d", status, __LINE__);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }

    status = IIC_WriteByte(instance_id, val);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write byte: %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }

    IIC_Stop(instance_id);
    status = IIC_WaitWriteComplete(instance_id, devAddr);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Write operation timeout: %d", status);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }

    rtos_ops->SemaphoreGive(IIC1_config.mutex);
    Log_Message(LOG_LEVEL_INFO, "[EEPROM] Wrote byte 0x%02X to reg 0x%02X", val, reg);
    return IIC_OK;
}

/**
 * @brief 从指定寄存器读取多字节
 * @param reg 起始寄存器地址
 * @param buffer 数据存储缓冲区
 * @param length 要读取的字节数
 * @return IIC_Status 操作状态
 */
IIC_Status EEPROMReadBytesFromReg(uint8_t reg, uint8_t *buffer, uint16_t length)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !buffer || length == 0 || (reg + length - 1) > EE_TYPE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Invalid parameters or address overflow");
        return IIC_ERR_INIT;
    }

    if (!rtos_ops->SemaphoreTake(IIC1_config.mutex, 100))
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to take mutex, timeout after 100ms");
        return IIC_ERR_TIMEOUT;
    }

    uint8_t instance_id = IIC1;
    IIC_Status status = IIC_Start(instance_id);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to send start signal: %d", status);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }

#if (EEPROM_TYPE > AT24C16)
    status = IIC_WriteByte(instance_id, EEPROM_ADDR);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write device address: %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }
    status = IIC_WriteByte(instance_id, reg >> 8);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write high address: %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }
#else
    uint8_t devAddr = EEPROM_ADDR | ((reg >> 7) & 0x0E);
    status = IIC_WriteByte(instance_id, devAddr);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write device address: %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }
#endif

    status = IIC_WriteByte(instance_id, reg & 0xFF);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write low address: %d ,line is %d", status, __LINE__);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }

    status = IIC_Start(instance_id);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to send repeated start: %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }

    status = IIC_WriteByte(instance_id, EEPROM_ADDR | 0x01);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write read mode address: %d", status);
        IIC_Stop(instance_id);
        rtos_ops->SemaphoreGive(IIC1_config.mutex);
        return status;
    }

    for (uint16_t i = 0; i < length; i++)
    {
        status = IIC_ReadByte(instance_id, (i == length - 1) ? 0 : 1, &buffer[i]);
        if (status != IIC_OK)
        {
            Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to read byte %d: %d", i, status);
            IIC_Stop(instance_id);
            rtos_ops->SemaphoreGive(IIC1_config.mutex);
            return status;
        }
    }

    IIC_Stop(instance_id);
    rtos_ops->SemaphoreGive(IIC1_config.mutex);
    Log_Message(LOG_LEVEL_INFO, "[EEPROM] Read %d bytes from reg 0x%02X", length, reg);
    return IIC_OK;
}

/**
 * @brief 向指定寄存器写入多字节
 * @param reg 起始寄存器地址
 * @param buffer 要写入的数据缓冲区
 * @param length 要写入的字节数
 * @return IIC_Status 操作状态
 */
IIC_Status EEPROMWriteBytesToReg(uint8_t reg, const uint8_t *buffer, uint16_t length)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !buffer || length == 0 || (reg + length - 1) > EE_TYPE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Invalid parameters or address overflow");
        return IIC_ERR_INIT;
    }

    if (!rtos_ops->SemaphoreTake(IIC1_config.mutex, 100))
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to take mutex, timeout after 100ms");
        return IIC_ERR_TIMEOUT;
    }

    uint8_t instance_id = IIC1;
    uint16_t bytes_written = 0;

    while (bytes_written < length)
    {
        uint8_t page_offset = reg % EEPROM_PAGE_SIZE;
        uint8_t bytes_in_page = EEPROM_PAGE_SIZE - page_offset;
        uint8_t bytes_to_write = (length - bytes_written) < bytes_in_page ? (length - bytes_written) : bytes_in_page;

        IIC_Status status = IIC_Start(instance_id);
        if (status != IIC_OK)
        {
            Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to send start signal: %d", status);
            rtos_ops->SemaphoreGive(IIC1_config.mutex);
            return status;
        }

#if (EEPROM_TYPE > AT24C16)
        status = IIC_WriteByte(instance_id, EEPROM_ADDR);
        if (status != IIC_OK)
        {
            Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write device address: %d", status);
            IIC_Stop(instance_id);
            rtos_ops->SemaphoreGive(IIC1_config.mutex);
            return status;
        }
        status = IIC_WriteByte(instance_id, reg >> 8);
        if (status != IIC_OK)
        {
            Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write high address: %d", status);
            IIC_Stop(instance_id);
            rtos_ops->SemaphoreGive(IIC1_config.mutex);
            return status;
        }
#else
        uint8_t devAddr = EEPROM_ADDR | ((reg >> 7) & 0x0E);
        status = IIC_WriteByte(instance_id, devAddr);
        if (status != IIC_OK)
        {
            Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write device address: %d", status);
            IIC_Stop(instance_id);
            rtos_ops->SemaphoreGive(IIC1_config.mutex);
            return status;
        }
#endif

        status = IIC_WriteByte(instance_id, reg & 0xFF);
        if (status != IIC_OK)
        {
            Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write low address: %d, line is %d", status, __LINE__);
            IIC_Stop(instance_id);
            rtos_ops->SemaphoreGive(IIC1_config.mutex);
            return status;
        }

        for (uint8_t i = 0; i < bytes_to_write; i++)
        {
            status = IIC_WriteByte(instance_id, buffer[bytes_written + i]);
            if (status != IIC_OK)
            {
                Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to write byte %d: %d", bytes_written + i, status);
                IIC_Stop(instance_id);
                rtos_ops->SemaphoreGive(IIC1_config.mutex);
                return status;
            }
        }

        IIC_Stop(instance_id);
        status = IIC_WaitWriteComplete(instance_id, devAddr);
        if (status != IIC_OK)
        {
            Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Write operation timeout: %d", status);
            rtos_ops->SemaphoreGive(IIC1_config.mutex);
            return status;
        }

        bytes_written += bytes_to_write;
        reg += bytes_to_write;
    }

    rtos_ops->SemaphoreGive(IIC1_config.mutex);
    Log_Message(LOG_LEVEL_INFO, "[EEPROM] Wrote %d bytes to reg 0x%02X", length, reg - length);
    return IIC_OK;
}
