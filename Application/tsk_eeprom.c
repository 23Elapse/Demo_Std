/**
 * =====================================================================================
 * @file        tsk_eeprom.c
 * @brief       EEPROM 参数管理任务实现 (Refactored & Encapsulated)
 * @author      23Elapse & Gemini
 * @version     2.2 (Refactored & Encapsulated)
 * @date        2025-06-14
 * @note        本模块负责所有需要掉电保存的参数的管理。
 * =====================================================================================
 */
#include "tsk_eeprom.h"
#include "api_eeprom.h"  // 依赖EEPROM驱动接口
#include "dev_config.h"  // 依赖全局设备实例（如I2C总线）
#include "log_system.h"
#include <string.h>      // For memcpy, memset
#include "pch.h"         // For pdTRUE, pdMS_TO_TICKS etc.

/*
 * =====================================================================================
 * 模块内部定义
 * =====================================================================================
 */

// --- EEPROM 硬件配置 ---
// 这里定义EEPROM的设备实例，包含其连接的总线、地址、容量和页大小。
// 这些值应与实际使用的AT24Cxx芯片型号匹配。
#define EEPROM_DEVICE_7BIT_ADDR 0x50 // EEPROM的7位I2C从设备地址（例如AT24C02通常是0x50）
#define EEPROM_CHIP_CAPACITY    AT24C02_CAPACITY // 使用AT24C02示例，256字节
#define EEPROM_CHIP_PAGE_SIZE   8    // AT24C02的页大小是8字节

// --- 内部数据结构 ---
typedef enum { EEPROM_RESET_BEHAVIOR, EEPROM_NO_RESET_BEHAVIOR } FactoryResetBehavior_t; // 明确枚举类型名

/**
 * @brief EEPROM参数表条目结构体
 * @note  定义了每个需要持久化参数的属性。
 */
typedef struct {
    uint16_t id;                // 参数唯一ID
    uint16_t ee_addr;           // EEPROM中的存储地址
    uint16_t* ram_addr;          // RAM中对应参数的指针
    uint16_t default_value;     // 参数的默认值
    uint16_t max_value;         // 参数的最大值
    uint16_t min_value;         // 参数的最小值
    bool dirty;                 // 标志位：true表示RAM中的值已修改，需要保存到EEPROM
    FactoryResetBehavior_t reset_behavior; // 恢复出厂设置时的行为
} EepromTableEntry_t; // 统一为 _t 后缀

/*
 * =====================================================================================
 * 模块私有变量 (Static Data)
 * =====================================================================================
 */

// --- EEPROM 设备实例 ---
static EEPROM_Device_t s_eeprom_dev; // 全局唯一的EEPROM设备实例

// --- 参数定义 (RAM中实际存储参数的变量) ---
static uint16_t s_ram_temperature = 0; // 初始值可以设为0或任何默认值
static uint16_t s_ram_pressure = 0;

// --- 参数配置表 (完全封装在模块内部) ---
static EepromTableEntry_t g_eeprom_table[] = {
    // ID, EE_Addr, RAM_Addr,        DefaultValue, MaxValue, MinValue, dirty, ResetBehavior
    { 1,  0x0000,  &s_ram_temperature, 25,         100,      0,        false, EEPROM_RESET_BEHAVIOR },
    { 2,  0x0002,  &s_ram_pressure,    1000,       2000,     500,      false, EEPROM_NO_RESET_BEHAVIOR },
    // 注意：确保EEPROM地址不重叠，且每次写入的长度不超过页大小
    // 0x0000和0x0002地址适用于16位数据，且不跨页（如果页大小为2字节或以上）
};
static const uint16_t g_table_size = sizeof(g_eeprom_table) / sizeof(EepromTableEntry_t);

// --- 模块句柄 ---
static void* s_eeprom_mutex = NULL; // 用于保护参数表访问的互斥锁

/*
 * =====================================================================================
 * 模块私有函数 (Static Functions)
 * =====================================================================================
 */
static EepromMgr_Status_t _EepromMgr_FindEntryById(uint16_t id, EepromTableEntry_t** entry);
static void _EepromMgr_SetDefaultData(EepromTableEntry_t* entry);
static EepromMgr_Status_t _EepromMgr_ReadEntryToRam(EepromTableEntry_t* entry);
static EepromMgr_Status_t _EepromMgr_SaveEntryToEeprom(EepromTableEntry_t* entry);
static EepromMgr_Status_t _EepromMgr_BulkSaveDirtyEntries(void);
static void _EepromMgr_MonitorTask(void* params);


/*
 * =====================================================================================
 * 公共API函数实现 (Public API Implementation)
 * =====================================================================================
 */

/**
 * @brief 初始化EEPROM参数管理模块。
 * @return EepromMgr_Status_t 初始化状态。
 */
EepromMgr_Status_t Tsk_Eeprom_Init(void) {
    // 1. 创建互斥锁
    s_eeprom_mutex = g_rtos_ops->SemaphoreCreate();
    if (s_eeprom_mutex == NULL) {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM Mgr] Mutex creation failed.");
        return EEPROM_MGR_INIT_FAILED;
    }
    
    // 2. 初始化EEPROM设备句柄
    // 假设 g_i2c1_bus 已经由 App_Driver_Init 初始化
    if (EEPROM_Device_Init(&s_eeprom_dev, &g_i2c1_bus, EEPROM_DEVICE_7BIT_ADDR, EEPROM_CHIP_CAPACITY, EEPROM_CHIP_PAGE_SIZE) != I2C_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM Mgr] EEPROM device initialization failed. Check I2C bus and device.");
        g_rtos_ops->SemaphoreDelete(s_eeprom_mutex); // 失败时释放互斥锁
        return EEPROM_MGR_INIT_FAILED;
    }

    // 3. 从EEPROM加载所有参数到RAM，或使用默认值
    for (uint16_t i = 0; i < g_table_size; i++) {
        // 尝试从EEPROM读取
        if (_EepromMgr_ReadEntryToRam(&g_eeprom_table[i]) != EEPROM_MGR_OK) {
            Log_Message(LOG_LEVEL_WARNING, "[EEPROM Mgr] Load failed for param ID %u (Addr 0x%04X), using default.",
                        g_eeprom_table[i].id, g_eeprom_table[i].ee_addr);
            _EepromMgr_SetDefaultData(&g_eeprom_table[i]); // 读取失败或值非法，则设为默认值
            // 标记为脏数据，将在后续的监控任务中写入EEPROM
            g_eeprom_table[i].dirty = true;
        }
    }
    Log_Message(LOG_LEVEL_INFO, "[EEPROM Mgr] Parameter module initialized.");

    // 4. 创建后台监控任务
    if (g_rtos_ops->TaskCreate(_EepromMgr_MonitorTask, "EepromMonitor", 256, NULL, 1) != pdTRUE) { // 优先级可调整
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM Mgr] Failed to create EepromMonitorTask.");
        g_rtos_ops->SemaphoreDelete(s_eeprom_mutex); // 失败时释放互斥锁
        return EEPROM_MGR_INIT_FAILED;
    }
    return EEPROM_MGR_OK;
}

/**
 * @brief 恢复所有参数到出厂默认设置。
 * @return EepromMgr_Status_t 操作状态。
 */
EepromMgr_Status_t Tsk_Eeprom_FactoryReset(void) {
    // 获取互斥锁，保护参数表的访问
    if (g_rtos_ops->SemaphoreTake(s_eeprom_mutex, pdMS_TO_TICKS(500)) != pdTRUE) { // 500ms超时
        Log_Message(LOG_LEVEL_WARNING, "[EEPROM Mgr] Factory Reset: Failed to get mutex (timeout).");
        return EEPROM_MGR_LOCK_TIMEOUT;
    }

    for (uint16_t i = 0; i < g_table_size; i++) {
        if (g_eeprom_table[i].reset_behavior == EEPROM_RESET_BEHAVIOR) {
            _EepromMgr_SetDefaultData(&g_eeprom_table[i]); // 设置为默认值
            // 立即保存到EEPROM，确保恢复操作的原子性
            if (_EepromMgr_SaveEntryToEeprom(&g_eeprom_table[i]) != EEPROM_MGR_OK) {
                Log_Message(LOG_LEVEL_ERROR, "[EEPROM Mgr] Factory Reset: Failed to save default for param ID %u.", g_eeprom_table[i].id);
                // 即使部分失败，也继续处理其他参数
            }
        }
    }
    g_rtos_ops->SemaphoreGive(s_eeprom_mutex); // 释放互斥锁
    Log_Message(LOG_LEVEL_INFO, "[EEPROM Mgr] Factory reset completed.");
    return EEPROM_MGR_OK;
}

/**
 * @brief 通过ID获取一个参数的当前值。
 * @param id  要获取的参数ID。
 * @param p_value 指向用于存储参数值的变量。
 * @return EepromMgr_Status_t 操作状态。
 */
EepromMgr_Status_t Tsk_Eeprom_GetParam(uint16_t id, uint16_t* p_value) {
    EepromTableEntry_t* entry = NULL;
    if (!p_value) return EEPROM_MGR_INVALID_PARAM;
    
    // 查找参数条目
    if (_EepromMgr_FindEntryById(id, &entry) != EEPROM_MGR_OK) {
        Log_Message(LOG_LEVEL_WARNING, "[EEPROM Mgr] GetParam: Invalid ID %u.", id);
        return EEPROM_MGR_INVALID_ID;
    }
    
    // 获取互斥锁，保护RAM中参数的读取
    if (g_rtos_ops->SemaphoreTake(s_eeprom_mutex, pdMS_TO_TICKS(100)) != pdTRUE) { // 100ms超时
        Log_Message(LOG_LEVEL_WARNING, "[EEPROM Mgr] GetParam: Failed to get mutex (timeout).");
        return EEPROM_MGR_LOCK_TIMEOUT;
    }
    *p_value = *(entry->ram_addr); // 读取RAM中的值
    g_rtos_ops->SemaphoreGive(s_eeprom_mutex); // 释放互斥锁
    return EEPROM_MGR_OK;
}

/**
 * @brief 通过ID设置一个参数的值。
 * @param id 要设置的参数ID。
 * @param value 要设置的新值。
 * @return EepromMgr_Status_t 操作状态。
 */
EepromMgr_Status_t Tsk_Eeprom_SetParam(uint16_t id, uint16_t value) {
    EepromTableEntry_t* entry = NULL;
    
    // 查找参数条目
    if (_EepromMgr_FindEntryById(id, &entry) != EEPROM_MGR_OK) {
        Log_Message(LOG_LEVEL_WARNING, "[EEPROM Mgr] SetParam: Invalid ID %u.", id);
        return EEPROM_MGR_INVALID_ID;
    }

    // 检查值是否在合法范围内
    if (value < entry->min_value || value > entry->max_value) {
        Log_Message(LOG_LEVEL_WARNING, "[EEPROM Mgr] SetParam: Value %u out of range [%u, %u] for ID %u.",
                    value, entry->min_value, entry->max_value, id);
        return EEPROM_MGR_INVALID_PARAM;
    }

    // 获取互斥锁，保护RAM中参数的写入
    if (g_rtos_ops->SemaphoreTake(s_eeprom_mutex, pdMS_TO_TICKS(100)) != pdTRUE) { // 100ms超时
        Log_Message(LOG_LEVEL_WARNING, "[EEPROM Mgr] SetParam: Failed to get mutex (timeout).");
        return EEPROM_MGR_LOCK_TIMEOUT;
    }
    // 只有当值发生变化时才更新并标记为脏数据
    if (*(entry->ram_addr) != value) {
        *(entry->ram_addr) = value;
        entry->dirty = true;
        Log_Message(LOG_LEVEL_DEBUG, "[EEPROM Mgr] SetParam: Param ID %u updated to %u, marked dirty.", id, value);
    }
    g_rtos_ops->SemaphoreGive(s_eeprom_mutex); // 释放互斥锁
    return EEPROM_MGR_OK;
}

/*
 * =====================================================================================
 * 内部函数实现
 * =====================================================================================
 */

/**
 * @brief 内部函数：通过ID查找参数表条目。
 * @param id 要查找的参数ID。
 * @param entry_out 指向存储找到的条目指针的指针。
 * @return EepromMgr_Status_t 查找结果。
 */
static EepromMgr_Status_t _EepromMgr_FindEntryById(uint16_t id, EepromTableEntry_t** entry_out) {
    for (uint16_t i = 0; i < g_table_size; i++) {
        if (g_eeprom_table[i].id == id) {
            *entry_out = &g_eeprom_table[i];
            return EEPROM_MGR_OK;
        }
    }
    *entry_out = NULL;
    return EEPROM_MGR_INVALID_ID;
}

/**
 * @brief 内部函数：设置参数为默认值。
 * @param entry 指向要设置的参数表条目。
 */
static void _EepromMgr_SetDefaultData(EepromTableEntry_t* entry) {
    *(entry->ram_addr) = entry->default_value;
    entry->dirty = true; // 标记为脏数据，待保存
    Log_Message(LOG_LEVEL_DEBUG, "[EEPROM Mgr] Set default for ID %u to %u.", entry->id, entry->default_value);
}

/**
 * @brief 内部函数：从EEPROM读取一个参数条目的值到RAM。
 * @param entry 指向要读取的参数表条目。
 * @return EepromMgr_Status_t 操作状态。
 */
static EepromMgr_Status_t _EepromMgr_ReadEntryToRam(EepromTableEntry_t* entry) {
    uint16_t val_from_eeprom;
    // 从EEPROM读取两个字节到val_from_eeprom
    EEPROM_Status_t status = EEPROM_ReadBytes(&s_eeprom_dev, entry->ee_addr, (uint8_t*)&val_from_eeprom, sizeof(uint16_t));
    if (status != I2C_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM Mgr] Read to RAM failed for ID %u (Addr 0x%04X). Status: %d.", entry->id, entry->ee_addr, status);
        return EEPROM_MGR_ERROR;
    }
    // 校验读取到的值是否在合法范围内
    if (val_from_eeprom < entry->min_value || val_from_eeprom > entry->max_value) {
        Log_Message(LOG_LEVEL_WARNING, "[EEPROM Mgr] Read to RAM: Value %u for ID %u (Addr 0x%04X) is out of range [%u, %u].",
                    val_from_eeprom, entry->id, entry->ee_addr, entry->min_value, entry->max_value);
        return EEPROM_MGR_ERROR; // 认为读取到的数据非法
    }
    *(entry->ram_addr) = val_from_eeprom; // 更新RAM中的值
    Log_Message(LOG_LEVEL_DEBUG, "[EEPROM Mgr] Read to RAM for ID %u: 0x%04X -> %u.", entry->id, entry->ee_addr, val_from_eeprom);
    return EEPROM_MGR_OK;
}

/**
 * @brief 内部函数：保存一个参数条目的值从RAM到EEPROM。
 * @param entry 指向要保存的参数表条目。
 * @return EepromMgr_Status_t 操作状态。
 */
static EepromMgr_Status_t _EepromMgr_SaveEntryToEeprom(EepromTableEntry_t* entry) {
    uint16_t ram_value = *(entry->ram_addr);
    // 使用EEPROM_WritePage写入2字节（uint16_t）的数据
    // 确保mem_addr和len符合WritePage的要求 (不跨页，len <= page_size)
    if (sizeof(uint16_t) > s_eeprom_dev.page_size) {
        // 如果 uint16_t 长度大于页大小，需要分多次写入，或者使用EEPROM_WriteBytes
        // 简化起见，这里假设 sizeof(uint16_t) <= page_size
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM Mgr] Save entry: uint16_t size (%lu) larger than page size (%u). WritePage not suitable.",
                    sizeof(uint16_t), s_eeprom_dev.page_size);
        return EEPROM_MGR_ERROR;
    }
    
    EEPROM_Status_t status = EEPROM_WritePage(&s_eeprom_dev, entry->ee_addr, (const uint8_t*)&ram_value, sizeof(uint16_t));
    if (status != I2C_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM Mgr] Save to EEPROM failed for ID %u (Addr 0x%04X, Value %u). Status: %d.",
                    entry->id, entry->ee_addr, ram_value, status);
        return EEPROM_MGR_ERROR;
    }
    entry->dirty = false; // 成功保存，清除脏标志
    Log_Message(LOG_LEVEL_DEBUG, "[EEPROM Mgr] Saved ID %u (Value %u) to EEPROM Addr 0x%04X.", entry->id, ram_value, entry->ee_addr);
    return EEPROM_MGR_OK;
}

/**
 * @brief 内部函数：批量保存所有脏数据到EEPROM。
 * @return EepromMgr_Status_t 操作状态。
 */
static EepromMgr_Status_t _EepromMgr_BulkSaveDirtyEntries(void) {
    // 尝试获取互斥锁，保护批量保存过程
    if (g_rtos_ops->SemaphoreTake(s_eeprom_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        Log_Message(LOG_LEVEL_WARNING, "[EEPROM Mgr] Bulk Save: Failed to get mutex (timeout).");
        return EEPROM_MGR_LOCK_TIMEOUT;
    }
    uint16_t saved_count = 0;
    for (uint16_t i = 0; i < g_table_size; i++) {
        if (g_eeprom_table[i].dirty) {
            if (_EepromMgr_SaveEntryToEeprom(&g_eeprom_table[i]) == EEPROM_MGR_OK) {
                saved_count++;
            }
            // 如果保存失败，该条目仍然保持dirty状态，下次轮询会再次尝试
        }
    }
    g_rtos_ops->SemaphoreGive(s_eeprom_mutex); // 释放互斥锁
    if (saved_count > 0) {
        Log_Message(LOG_LEVEL_INFO, "[EEPROM Mgr] Bulk save completed. %u entries saved.", saved_count);
    }
    return EEPROM_MGR_OK; // 即使有部分失败，也返回OK，因为是后台任务
}

/**
 * @brief EEPROM 参数监控任务。
 * @param params 未使用参数。
 */
static void _EepromMgr_MonitorTask(void* params) {
    (void)params; // 避免编译器警告
    Log_Message(LOG_LEVEL_INFO, "[EEPROM Monitor] Task started.");
    for (;;) {
        g_rtos_ops->Delay(pdMS_TO_TICKS(5000)); // 每5秒检查一次
        _EepromMgr_BulkSaveDirtyEntries(); // 尝试保存所有脏数据
    }
}
