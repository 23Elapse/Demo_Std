/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 19:10:06
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 01:27:02
 * @FilePath: \Demo\Middlewares\Src\tsk_eeprom.c
 * @Description: EEPROM 任务管理实现
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "tsk_eeprom.h"
#include "api_eeprom.h"
#include "log_system.h"
#include <string.h>

/**
 * @brief 全局互斥锁定义
 */
SemaphoreHandle_t xEepromMutex = NULL;

/**
 * @brief 定义 RAM 变量
 */
uint16_t temperature = 25;
uint16_t pressure = 1000;

/**
 * @brief 定义 EEPROM 表
 */
EepromTableEntry eepromTable[] = {
    //id,   EE_Addr,    RAM_Addr,          DefaultValue,    MaxValue,   MinValue,   dirty,      ResetBehavior,      Version,    Checksum     
    { 1,    0x1000,     &temperature,      25,              100,        0,          false,      EERESET,            1,          0 },
    { 2,    0x1001,     &pressure,         1000,            2000,       500,        false,      NO_RESET,           1,          0 }
};
uint16_t table_size = sizeof(eepromTable) / sizeof(EepromTableEntry);

/**
 * @brief 计算校验和
 * @param entry 配置条目
 * @return uint32_t 校验和
 */
static uint32_t calculate_checksum(const EepromTableEntry *entry)
{
    return entry->id + entry->EE_Addr + *(entry->RAM_Addr);
}

/**
 * @brief 从 EEPROM 读取数据到 RAM
 * @param entry 配置条目
 * @return EepromErrorCode 操作状态
 */
EepromErrorCode EepromReadToRam(EepromTableEntry *entry)
{
    if (!entry)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Invalid entry pointer");
        return EEPROM_INVALID_PARAM;
    }

    uint8_t eeprom_value = 0;
    IIC_Status status = IIC1_EEPROM.ReadByte(entry->EE_Addr, &eeprom_value);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Read failed for entry %d: %d", entry->id, status);
        return EEPROM_DATA_INVALID;
    }
    entry->Checksum = calculate_checksum(entry);

    if (eeprom_value < entry->MinValue || eeprom_value > entry->MaxValue)
    {
        DefaultEEData(entry);
        Log_Message(LOG_LEVEL_WARNING, "[EEPROM] Data out of range for entry %d", entry->id);
        return EEPROM_DATA_INVALID;
    }

    *(entry->RAM_Addr) = eeprom_value;
    Log_Message(LOG_LEVEL_INFO, "[EEPROM] Read successful for entry %d: %d", entry->id, eeprom_value);
    return EEPROM_OK;
}

/**
 * @brief 将 RAM 数据写入 EEPROM
 * @param entry 配置条目
 * @return EepromErrorCode 操作状态
 */
EepromErrorCode RamSaveToEeprom(EepromTableEntry *entry)
{
    if (!entry)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Invalid entry pointer");
        return EEPROM_INVALID_PARAM;
    }

    uint8_t ram_value = *(entry->RAM_Addr);
    if (ram_value < entry->MinValue || ram_value > entry->MaxValue)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Data out of range for entry %d", entry->id);
        return EEPROM_DATA_INVALID;
    }

    IIC_Status status = IIC1_EEPROM.WriteByte(entry->EE_Addr, ram_value);
    if (status != IIC_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Write failed for entry %d: %d", entry->id, status);
        return EEPROM_WRITE_FAILED;
    }
    entry->dirty = false;
    Log_Message(LOG_LEVEL_INFO, "[EEPROM] Write successful for entry %d: %d", entry->id, ram_value);
    return EEPROM_OK;
}

/**
 * @brief 设置默认 EEPROM 数据
 * @param entry 配置条目
 */
void DefaultEEData(EepromTableEntry *entry)
{
    if (!entry)
        return;
    *(entry->RAM_Addr) = entry->DefaultValue;
    entry->dirty = true;
    Log_Message(LOG_LEVEL_INFO, "[EEPROM] Default value set for entry %d: %d", entry->id, entry->DefaultValue);
}

/**
 * @brief 恢复出厂设置
 * @param table 配置表
 * @param table_size 表大小
 * @return EepromErrorCode 操作状态
 */
EepromErrorCode EepromReset(EepromTableEntry *table, uint16_t table_size)
{
    if (!table)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Invalid table pointer");
        return EEPROM_INVALID_PARAM;
    }

    for (uint16_t i = 0; i < table_size; i++)
    {
        if (table[i].ResetBehavior == EERESET)
        {
            DefaultEEData(&table[i]);
            EepromErrorCode err = RamSaveToEeprom(&table[i]);
            if (err != EEPROM_OK)
            {
                Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Reset failed for entry %d: %d", table[i].id, err);
            }
        }
    }
    Log_Message(LOG_LEVEL_INFO, "[EEPROM] Factory reset completed");
    return EEPROM_OK;
}

/**
 * @brief 按 ID 查找配置条目
 * @param table 配置表
 * @param table_size 表大小
 * @param id 条目 ID
 * @param entry 找到的条目指针
 * @return EepromErrorCode 操作状态
 */
EepromErrorCode EepromFindEntryById(EepromTableEntry *table, uint16_t table_size, uint8_t id, EepromTableEntry **entry)
{
    if (!table || !entry)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Invalid parameters");
        return EEPROM_INVALID_PARAM;
    }
    for (uint16_t i = 0; i < table_size; i++)
    {
        if (table[i].id == id)
        {
            *entry = &table[i];
            return EEPROM_OK;
        }
    }
    Log_Message(LOG_LEVEL_WARNING, "[EEPROM] Entry not found for ID %d", id);
    return EEPROM_ENTRY_NOT_FOUND;
}

/**
 * @brief 批量保存脏数据
 * @param table 配置表
 * @param table_size 表大小
 * @return EepromErrorCode 操作状态
 */
EepromErrorCode EepromBulkSaveDirtyEntries(EepromTableEntry *table, uint16_t table_size)
{
    if (!table)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Invalid table pointer");
        return EEPROM_INVALID_PARAM;
    }
    EepromErrorCode final_status = EEPROM_OK;

    if (xSemaphoreTake(xEepromMutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Failed to take mutex");
        return EEPROM_LOCK_TIMEOUT;
    }

    for (uint16_t i = 0; i < table_size; i++)
    {
        if (table[i].dirty)
        {
            EepromErrorCode err = RamSaveToEeprom(&table[i]);
            if (err != EEPROM_OK)
            {
                Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Save failed for entry %d: %d", table[i].id, err);
                final_status = err;
            }
        }
    }

    xSemaphoreGive(xEepromMutex);
    Log_Message(LOG_LEVEL_INFO, "[EEPROM] Bulk save completed with status %d", final_status);
    return final_status;
}

/**
 * @brief 初始化 EEPROM 数据
 * @param table 配置表
 * @param table_size 表大小
 * @return EepromErrorCode 操作状态
 */
EepromErrorCode EepromInitialize(EepromTableEntry *table, uint16_t table_size)
{
    if (!table)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Invalid table pointer");
        return EEPROM_INVALID_PARAM;
    }

    for (uint16_t i = 0; i < table_size; i++)
    {
        EepromErrorCode err = EepromReadToRam(&table[i]);
        if (err != EEPROM_OK)
        {
            Log_Message(LOG_LEVEL_WARNING, "[EEPROM] Initialization failed for entry %d: %d", table[i].id, err);
        }
    }
    Log_Message(LOG_LEVEL_INFO, "[EEPROM] Initialization completed");
    return EEPROM_OK;
}

/**
 * @brief EEPROM 监控任务
 * @param pvParameters 任务参数
 */
void EepromMonitorTask(void *pvParameters)
{
    EepromTableEntry **params = (EepromTableEntry **)pvParameters;
    EepromTableEntry *table = params[0];
    uint16_t table_size = (uint16_t)params[1];

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000)); // 每 5 秒检查一次
        EepromErrorCode status = EepromBulkSaveDirtyEntries(table, table_size);
        if (status != EEPROM_OK && status != EEPROM_LOCK_TIMEOUT)
        {
            Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Bulk save failed: %d", status);
        }
    }
}

/*
 * 示例用法：
 * 1. 设置 RTOS 抽象层
 * RTOS_SetOps(&FreeRTOS_Ops);
 *
 * 2. 初始化互斥锁
 * xEepromMutex = xSemaphoreCreateMutex();
 *
 * 3. 初始化 EEPROM 数据
 * EepromInitialize(eepromTable, table_size);
 *
 * 4. 创建 EEPROM 监控任务
 * EepromTableEntry *params[] = {eepromTable, (EepromTableEntry*)table_size};
 * xTaskCreate(EepromMonitorTask, "EepromMonitor", 256, params, 1, NULL);
 */
