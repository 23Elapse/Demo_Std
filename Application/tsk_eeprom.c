/**
 * =====================================================================================
 * @file        tsk_eeprom.c
 * @brief       EEPROM 参数管理任务实现 (已封装)
 * @author      23Elapse & Gemini
 * @version     2.1 (Refactored & Encapsulated)
 * @date        2025-06-08
 * =====================================================================================
 */
#include "tsk_eeprom.h"
#include "api_eeprom.h"
#include "dev_config.h"
#include "log_system.h"
#include <string.h>

/*
 * =====================================================================================
 * 模块内部定义
 * =====================================================================================
 */

// --- 硬件配置 ---
#define EEPROM_BUS          &g_i2c1_bus
#define EEPROM_DEVICE_ADDR  0xA0
#define EEPROM_PAGE_SIZE    8

// --- 内部数据结构 ---
typedef enum { EERESET, NO_RESET } FactoryResetBehavior;

typedef struct {
    uint16_t id;
    uint16_t EE_Addr;
    uint16_t* RAM_Addr;
    uint16_t DefaultValue;
    uint16_t MaxValue;
    uint16_t MinValue;
    bool dirty;
    FactoryResetBehavior ResetBehavior;
} EepromTableEntry;

/*
 * =====================================================================================
 * 模块私有变量 (Static Data)
 * =====================================================================================
 */

// --- 参数定义 (RAM) ---
static uint16_t g_ram_temperature = 25;
static uint16_t g_ram_pressure = 1000;

// --- 参数配置表 (完全封装在模块内部) ---
static EepromTableEntry g_eeprom_table[] = {
    //id, EE_Addr, RAM_Addr,            DefaultValue, MaxValue, MinValue, dirty, ResetBehavior
    { 1,  0x0001,  &g_ram_temperature,  25,           100,      0,        false, EERESET },
    { 2,  0x0002,  &g_ram_pressure,     1000,         2000,     500,      false, NO_RESET },
};
static const uint16_t g_table_size = sizeof(g_eeprom_table) / sizeof(EepromTableEntry);

// --- 模块句柄 ---
static void* g_eeprom_mutex = NULL;


/*
 * =====================================================================================
 * 模块私有函数 (Static Functions)
 * =====================================================================================
 */
static EepromMgr_Status_t find_entry_by_id(uint16_t id, EepromTableEntry** entry);
static void set_default_data(EepromTableEntry* entry);
static EepromMgr_Status_t read_entry_to_ram(EepromTableEntry* entry);
static EepromMgr_Status_t save_entry_to_eeprom(EepromTableEntry* entry);
static EepromMgr_Status_t bulk_save_dirty_entries(void);
static void eeprom_monitor_task(void* params);


/*
 * =====================================================================================
 * 公共API函数实现 (Public API Implementation)
 * =====================================================================================
 */
EepromMgr_Status_t Tsk_Eeprom_Init(void) {
    g_eeprom_mutex = g_rtos_ops->SemaphoreCreate();
    if (g_eeprom_mutex == NULL) {
        Log_Message(LOG_LEVEL_ERROR, "Error: EEPROM mutex creation failed.");
        return EEPROM_MGR_ERROR;
    }

    for (uint16_t i = 0; i < g_table_size; i++) {
        if (read_entry_to_ram(&g_eeprom_table[i]) != EEPROM_MGR_OK) {
            Log_Message(LOG_LEVEL_WARNING, "[EEPROM] Init failed for entry %d, using default.", g_eeprom_table[i].id);
            set_default_data(&g_eeprom_table[i]);
        }
    }
    Log_Message(LOG_LEVEL_INFO, "[EEPROM] Parameter module initialized.");

    if (g_rtos_ops->TaskCreate(eeprom_monitor_task, "EepromMonitor", 256, NULL, 1) != true) {
        Log_Message(LOG_LEVEL_ERROR, "Error: Failed to create EepromMonitorTask.");
        return EEPROM_MGR_ERROR;
    }
    return EEPROM_MGR_OK;
}

EepromMgr_Status_t Tsk_Eeprom_FactoryReset(void) {
    if (g_rtos_ops->SemaphoreTake(g_eeprom_mutex, 100) != true) {
        return EEPROM_MGR_LOCK_TIMEOUT;
    }
    for (uint16_t i = 0; i < g_table_size; i++) {
        if (g_eeprom_table[i].ResetBehavior == EERESET) {
            set_default_data(&g_eeprom_table[i]);
            save_entry_to_eeprom(&g_eeprom_table[i]);
        }
    }
    g_rtos_ops->SemaphoreGive(g_eeprom_mutex);
    Log_Message(LOG_LEVEL_INFO, "[EEPROM] Factory reset completed.");
    return EEPROM_MGR_OK;
}

EepromMgr_Status_t Tsk_Eeprom_GetParam(uint16_t id, uint16_t* p_value) {
    EepromTableEntry* entry = NULL;
    if (!p_value) return EEPROM_MGR_INVALID_PARAM;
    if (find_entry_by_id(id, &entry) != EEPROM_MGR_OK) return EEPROM_MGR_INVALID_ID;
    
    if (g_rtos_ops->SemaphoreTake(g_eeprom_mutex, 100) != true) {
        return EEPROM_MGR_LOCK_TIMEOUT;
    }
    *p_value = *(entry->RAM_Addr);
    g_rtos_ops->SemaphoreGive(g_eeprom_mutex);
    return EEPROM_MGR_OK;
}

EepromMgr_Status_t Tsk_Eeprom_SetParam(uint16_t id, uint16_t value) {
    EepromTableEntry* entry = NULL;
    if (find_entry_by_id(id, &entry) != EEPROM_MGR_OK) return EEPROM_MGR_INVALID_ID;

    if (value < entry->MinValue || value > entry->MaxValue) {
        return EEPROM_MGR_INVALID_PARAM;
    }

    if (g_rtos_ops->SemaphoreTake(g_eeprom_mutex, 100) != true) {
        return EEPROM_MGR_LOCK_TIMEOUT;
    }
    if (*(entry->RAM_Addr) != value) {
        *(entry->RAM_Addr) = value;
        entry->dirty = true;
    }
    g_rtos_ops->SemaphoreGive(g_eeprom_mutex);
    return EEPROM_MGR_OK;
}

/*
 * =====================================================================================
 * 内部函数实现
 * =====================================================================================
 */
static EepromMgr_Status_t read_entry_to_ram(EepromTableEntry* entry) {
    uint16_t val_from_eeprom;
    I2C_Status_t status = EEPROM_ReadBytes(EEPROM_BUS, EEPROM_DEVICE_ADDR, entry->EE_Addr, (uint8_t*)&val_from_eeprom, sizeof(uint16_t));
    if (status != I2C_OK) {
        return EEPROM_MGR_ERROR;
    }
    if (val_from_eeprom < entry->MinValue || val_from_eeprom > entry->MaxValue) {
        return EEPROM_MGR_ERROR;
    }
    *(entry->RAM_Addr) = val_from_eeprom;
    return EEPROM_MGR_OK;
}

static EepromMgr_Status_t save_entry_to_eeprom(EepromTableEntry* entry) {
    uint16_t ram_value = *(entry->RAM_Addr);
    I2C_Status_t status = EEPROM_WritePage(EEPROM_BUS, EEPROM_DEVICE_ADDR, entry->EE_Addr, (const uint8_t*)&ram_value, sizeof(uint16_t));
    if (status != I2C_OK) {
        return EEPROM_MGR_ERROR;
    }
    entry->dirty = false;
    return EEPROM_MGR_OK;
}

static EepromMgr_Status_t bulk_save_dirty_entries(void) {
    if (g_rtos_ops->SemaphoreTake(g_eeprom_mutex, 100) != true) {
        return EEPROM_MGR_LOCK_TIMEOUT;
    }
    for (uint16_t i = 0; i < g_table_size; i++) {
        if (g_eeprom_table[i].dirty) {
            save_entry_to_eeprom(&g_eeprom_table[i]);
        }
    }
    g_rtos_ops->SemaphoreGive(g_eeprom_mutex);
    return EEPROM_MGR_OK;
}

static void eeprom_monitor_task(void* params) {
    (void)params; // 未使用参数
    for (;;) {
        g_rtos_ops->Delay(5000); // 每5秒检查一次
        bulk_save_dirty_entries();
    }
}

static void set_default_data(EepromTableEntry* entry) {
    *(entry->RAM_Addr) = entry->DefaultValue;
    entry->dirty = true;
}

static EepromMgr_Status_t find_entry_by_id(uint16_t id, EepromTableEntry** entry) {
    for (uint16_t i = 0; i < g_table_size; i++) {
        if (g_eeprom_table[i].id == id) {
            *entry = &g_eeprom_table[i];
            return EEPROM_MGR_OK;
        }
    }
    return EEPROM_MGR_INVALID_ID;
}
