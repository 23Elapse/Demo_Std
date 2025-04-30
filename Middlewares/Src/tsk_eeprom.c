#include "tsk_eeprom.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "api_eeprom.h"
// 全局互斥锁定义
SemaphoreHandle_t xEepromMutex = NULL;
// 定义 RAM 变量
uint16_t temperature = 25;
uint16_t pressure = 1000;

// 定义 EEPROM 表
EepromTableEntry eepromTable[] = {
    //id,   EE_Addr,    RAM_Addr,          DefaultValue,    MaxValue,   MinValue,   dirty,      ResetBehavior,      Version,    Checksum     
    { 1,    0x1000,     &temperature,      25,              100,        0,          false,      EERESET,            1,          0 },
    { 2,    0x1001,     &pressure,         1000,            2000,       500,        false,      NO_RESET,           1,          0 }
};
uint16_t table_size = sizeof(eepromTable) / sizeof(EepromTableEntry);


// 计算校验和（示例）
static uint32_t calculate_checksum(const EepromTableEntry* entry) {
    return entry->id + entry->EE_Addr + *(entry->RAM_Addr);
}

//--------------------- 新增函数实现 ---------------------

// 从 EEPROM 读取数据到 RAM
EepromErrorCode EepromReadToRam(EepromTableEntry* entry) {
    if (entry == NULL) return EEPROM_INVALID_PARAM;

    // 从 EEPROM 读取数据
    uint8_t eeprom_value = 0;
    IIC1_EEPROM.ReadByte(entry->EE_Addr, &eeprom_value); // 读取数据
    entry->Checksum = calculate_checksum(entry); // 计算校验和

    // 校验数据范围
    if (eeprom_value < entry->MinValue || eeprom_value > entry->MaxValue) {
        DefaultEEData(entry); // 恢复默认值
        return EEPROM_DATA_INVALID;
    }

    // 更新 RAM 数据
    *(entry->RAM_Addr) = eeprom_value;
    return EEPROM_OK;
}

// 将 RAM 数据写入 EEPROM
EepromErrorCode RamSaveToEeprom(EepromTableEntry* entry) {
    if (entry == NULL) return EEPROM_INVALID_PARAM;

    // 校验 RAM 数据有效性
    uint8_t ram_value = *(entry->RAM_Addr);
    if (ram_value < entry->MinValue || ram_value > entry->MaxValue) {
        return EEPROM_DATA_INVALID;
    }

    // 写入 EEPROM
    IIC1_EEPROM.WriteByte(entry->EE_Addr, ram_value);
    entry->dirty = false; // 清除脏标志
    return EEPROM_OK;
}

// 恢复 RAM 数据为默认值（并标记为脏数据）
void DefaultEEData(EepromTableEntry* entry) {
    if (entry == NULL) return;
    *(entry->RAM_Addr) = entry->DefaultValue;
    entry->dirty = true; // 标记需要保存
}

// 恢复出厂设置（根据 ResetBehavior 重置条目）
EepromErrorCode EepromReset(EepromTableEntry* table, uint16_t table_size) {
    if (table == NULL) return EEPROM_INVALID_PARAM;

    for (uint16_t i = 0; i < table_size; i++) {
        if (table[i].ResetBehavior == RESET) {
            DefaultEEData(&table[i]); // 恢复默认值
            RamSaveToEeprom(&table[i]); // 立即写入 EEPROM
        }
    }
    return EEPROM_OK;
}

// 初始化 EEPROM 数据（系统启动时调用）
EepromErrorCode EepromInitialize(EepromTableEntry* table, uint16_t table_size) {
    if (table == NULL) return EEPROM_INVALID_PARAM;

    for (uint16_t i = 0; i < table_size; i++) {
        EepromErrorCode err = EepromReadToRam(&table[i]);
        if (err != EEPROM_OK) {
            log_message(LOG_WARNING, "Erase %d entries failed, retrying...", table[i].id); 
        }
    }
    return EEPROM_OK;
}

//--------------------- 原有函数 ---------------------

// 按 ID 查找条目
EepromErrorCode EepromFindEntryById(EepromTableEntry* table, uint16_t table_size, uint8_t id, EepromTableEntry** entry) {
    if (table == NULL || entry == NULL) return EEPROM_INVALID_PARAM;
    for (uint16_t i = 0; i < table_size; i++) {
        if (table[i].id == id) {
            *entry = &table[i];
            return EEPROM_OK;
        }
    }
    return EEPROM_ENTRY_NOT_FOUND;
}

// 批量保存所有脏数据
EepromErrorCode EepromBulkSaveDirtyEntries(EepromTableEntry* table, uint16_t table_size) {
    if (table == NULL) return EEPROM_INVALID_PARAM;
    EepromErrorCode final_status = EEPROM_OK;

    // 获取互斥锁（等待 100ms）
    if (xSemaphoreTake(xEepromMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return EEPROM_LOCK_TIMEOUT;
    }

    for (uint16_t i = 0; i < table_size; i++) {
        if (table[i].dirty) {
            EepromErrorCode err = RamSaveToEeprom(&table[i]);
            if (err != EEPROM_OK) {
                log_message(LOG_ERROR, "条目 %d 保存失败，错误码: %d", table[i].id, err);
                final_status = err;
            }
        }
    }

    xSemaphoreGive(xEepromMutex);
    return final_status;
}

// 日志记录函数
void log_message(LogLevel level, const char* format, ...) {
    const char* level_str[] = {"[INFO]", "[WARN]", "[ERROR]"};
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    printf("%s %s\n", level_str[level], buffer);
}

// FreeRTOS 任务：定时检查并保存脏数据
void EepromMonitorTask(void* pvParameters) {
    EepromTableEntry** params = (EepromTableEntry**)pvParameters;
    EepromTableEntry* table = params[0];
    uint16_t table_size = (uint16_t)params[1];

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000)); // 每 5 秒检查一次
        EepromErrorCode status = EepromBulkSaveDirtyEntries(table, table_size);
        if (status != EEPROM_OK && status != EEPROM_LOCK_TIMEOUT) {
            log_message(LOG_ERROR, "批量保存失败，错误码: %d", status);
        }
    }
}

