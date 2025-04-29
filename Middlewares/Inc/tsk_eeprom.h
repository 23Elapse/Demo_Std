/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 19:10:06
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-27 19:10:36
 * @FilePath: \Demo\Middlewares\Inc\tsk_eeprom.h
 * @Description: 
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
#include "pch.h"
// 错误码定义
typedef enum {
    EEPROM_OK,              // 操作成功
    EEPROM_INVALID_PARAM,   // 无效参数
    EEPROM_ENTRY_NOT_FOUND, // 条目未找到
    EEPROM_DATA_INVALID,    // 数据校验失败
    EEPROM_WRITE_FAILED,    // 写入失败
    EEPROM_LOCK_TIMEOUT     // 互斥锁获取超时
} EepromErrorCode;

// 恢复出厂行为枚举
typedef enum {
    EERESET,      // 恢复出厂时重置
    NO_RESET    // 恢复出厂时保留
} FactoryResetBehavior;

// 日志级别
typedef enum {
    LOG_INFO,    // 信息日志
    LOG_WARNING, // 警告日志
    LOG_ERROR    // 错误日志
} LogLevel;

// EEPROM 配置条目
typedef struct {
    uint16_t id;                  // 自定义唯一标识符
    uint16_t EE_Addr;            // EEPROM 物理地址
    uint16_t* RAM_Addr;           // 指向 RAM 变量的指针
    uint16_t DefaultValue;        // 默认值
    uint16_t MaxValue;            // 最大值
    uint16_t MinValue;            // 最小值
    bool dirty;                  // 数据变更标志（需保存到 EEPROM）
    FactoryResetBehavior ResetBehavior; 
    uint16_t Version;
    uint32_t Checksum;
} EepromTableEntry;

// 全局互斥锁（保护脏标志和 EEPROM 操作）
extern SemaphoreHandle_t xEepromMutex;

// 函数声明
EepromErrorCode EepromReadToRam(EepromTableEntry* entry);
EepromErrorCode RamSaveToEeprom(EepromTableEntry* entry);
void DefaultEEData(EepromTableEntry* entry);
EepromErrorCode EepromReset(EepromTableEntry* table, uint16_t table_size);
EepromErrorCode EepromFindEntryById(EepromTableEntry* table, uint16_t table_size, uint8_t id, EepromTableEntry** entry);
EepromErrorCode EepromBulkSaveDirtyEntries(EepromTableEntry* table, uint16_t table_size);
EepromErrorCode EepromInitialize(EepromTableEntry* table, uint16_t table_size);
void log_message(LogLevel level, const char* format, ...);
void EepromMonitorTask(void* pvParameters);

#endif // EEPROM_MANAGER_H

