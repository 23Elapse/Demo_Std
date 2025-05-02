/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 19:10:06
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 00:55:12
 * @FilePath: \Demo\Middlewares\Inc\tsk_eeprom.h
 * @Description: EEPROM 任务管理头文件
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __TSK_EEPROM_H
#define __TSK_EEPROM_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "log_system.h"

/**
 * @brief EEPROM 错误码定义
 */
typedef enum
{
    EEPROM_OK,
    EEPROM_INVALID_PARAM,
    EEPROM_ENTRY_NOT_FOUND,
    EEPROM_DATA_INVALID,
    EEPROM_WRITE_FAILED,
    EEPROM_LOCK_TIMEOUT
} EepromErrorCode;

/**
 * @brief 恢复出厂行为枚举
 */
typedef enum
{
    EERESET,
    NO_RESET
} FactoryResetBehavior;

/**
 * @brief EEPROM 配置条目
 */
typedef struct
{
    uint16_t id;                        // 条目 ID
    uint16_t EE_Addr;                   // EEPROM 地址
    uint16_t *RAM_Addr;                 // RAM 地址
    uint16_t DefaultValue;              // 默认值
    uint16_t MaxValue;                  // 最大值
    uint16_t MinValue;                  // 最小值
    bool dirty;                         // 脏数据标志
    FactoryResetBehavior ResetBehavior; // 恢复出厂行为
    uint16_t Version;                   // 版本号
    uint32_t Checksum;                  // 校验和
} EepromTableEntry;

// 全局互斥锁
extern SemaphoreHandle_t xEepromMutex;

/**
 * @brief 从 EEPROM 读取数据到 RAM
 * @param entry 配置条目
 * @return EepromErrorCode 操作状态
 */
EepromErrorCode EepromReadToRam(EepromTableEntry *entry);

/**
 * @brief 将 RAM 数据写入 EEPROM
 * @param entry 配置条目
 * @return EepromErrorCode 操作状态
 */
EepromErrorCode RamSaveToEeprom(EepromTableEntry *entry);

/**
 * @brief 设置默认 EEPROM 数据
 * @param entry 配置条目
 */
void DefaultEEData(EepromTableEntry *entry);

/**
 * @brief 恢复出厂设置
 * @param table 配置表
 * @param table_size 表大小
 * @return EepromErrorCode 操作状态
 */
EepromErrorCode EepromReset(EepromTableEntry *table, uint16_t table_size);

/**
 * @brief 按 ID 查找配置条目
 * @param table 配置表
 * @param table_size 表大小
 * @param id 条目 ID
 * @param entry 找到的条目指针
 * @return EepromErrorCode 操作状态
 */
EepromErrorCode EepromFindEntryById(EepromTableEntry *table, uint16_t table_size, uint8_t id, EepromTableEntry **entry);

/**
 * @brief 批量保存脏数据
 * @param table 配置表
 * @param table_size 表大小
 * @return EepromErrorCode 操作状态
 */
EepromErrorCode EepromBulkSaveDirtyEntries(EepromTableEntry *table, uint16_t table_size);

/**
 * @brief 初始化 EEPROM 数据
 * @param table 配置表
 * @param table_size 表大小
 * @return EepromErrorCode 操作状态
 */
EepromErrorCode EepromInitialize(EepromTableEntry *table, uint16_t table_size);

/**
 * @brief EEPROM 监控任务
 * @param pvParameters 任务参数
 */
void EepromMonitorTask(void *pvParameters);

#endif /* __TSK_EEPROM_H */