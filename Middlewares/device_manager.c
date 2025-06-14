/**
 * =====================================================================================
 * @file        device_manager.c
 * @brief       设备管理器实现 (已优化)
 * @author      23Elapse & Gemini
 * @version     2.0 (Refactored)
 * @date        2025-06-08
 * =====================================================================================
 */
#include "device_manager.h"
#include "log_system.h"
#include <string.h>

void DeviceManager_Init(Device_Manager_t* mgr, Device_Handle_t* device_array, uint8_t max_size) {
    if (!g_rtos_ops || !mgr || !device_array || max_size == 0) {
        // 此时日志系统可能还未就绪，但尝试记录
        Log_Message(LOG_LEVEL_ERROR, "Error: DeviceManager_Init invalid parameters.");
        return;
    }

    mgr->devices = device_array;
    mgr->max_devices = max_size;
    mgr->count = 0;
    mgr->mutex = g_rtos_ops->SemaphoreCreate();
    if (!mgr->mutex) {
        Log_Message(LOG_LEVEL_ERROR, "Error: DeviceManager mutex creation failed.");
        return;
    }

    memset(mgr->devices, 0, sizeof(Device_Handle_t) * max_size);
    Log_Message(LOG_LEVEL_INFO, "[DeviceMgr] Initialized with capacity for %d devices.", max_size);
}

Device_Handle_t* DeviceManager_Register(Device_Manager_t* mgr, const void* device, Device_Type_t type, uint8_t id) {
    if (!g_rtos_ops || !mgr || !mgr->mutex || !device) {
        Log_Message(LOG_LEVEL_ERROR, "Error: DeviceManager_Register invalid parameters.");
        return NULL;
    }

    if (g_rtos_ops->SemaphoreTake(mgr->mutex, 1000) != 1) {
        Log_Message(LOG_LEVEL_ERROR, "Error: DeviceManager_Register failed to take mutex.");
        return NULL;
    }

    if (mgr->count >= mgr->max_devices) {
        Log_Message(LOG_LEVEL_ERROR, "Error: DeviceManager is full. Cannot register new device.");
        g_rtos_ops->SemaphoreGive(mgr->mutex);
        return NULL;
    }
    
    // 检查是否重复注册 (同一类型和ID)
    for (uint8_t i = 0; i < mgr->count; i++) {
        if (mgr->devices[i].type == type && mgr->devices[i].id == id) {
            Log_Message(LOG_LEVEL_ERROR, "Error: Device with type %d and ID %d already registered.", type, id);
            g_rtos_ops->SemaphoreGive(mgr->mutex);
            return NULL;
        }
    }

    // 找到空槽位并注册
    // 注意：由于我们不进行注销操作，可以直接在末尾添加
    uint8_t slot = mgr->count;
    mgr->devices[slot].device = (void*)device;
    mgr->devices[slot].type = type;
    mgr->devices[slot].id = id;
    mgr->devices[slot].status = DEVICE_STATUS_NOT_INITIALIZED;
    mgr->count++;
    
    Log_Message(LOG_LEVEL_INFO, "[DeviceMgr] Registered device (Type: %d, ID: %d) at slot %d.", type, id, slot);
    g_rtos_ops->SemaphoreGive(mgr->mutex);
    return &mgr->devices[slot];
}

Device_Handle_t* DeviceManager_Find(Device_Manager_t* mgr, Device_Type_t type, uint8_t id) {
    if (!g_rtos_ops || !mgr || !mgr->mutex) {
        return NULL;
    }

    if (g_rtos_ops->SemaphoreTake(mgr->mutex, 1000) != 1) {
        Log_Message(LOG_LEVEL_ERROR, "Error: DeviceManager_Find failed to take mutex.");
        return NULL;
    }

    for (uint8_t i = 0; i < mgr->count; i++) {
        if (mgr->devices[i].type == type && mgr->devices[i].id == id) {
            g_rtos_ops->SemaphoreGive(mgr->mutex);
            return &mgr->devices[i];
        }
    }

    g_rtos_ops->SemaphoreGive(mgr->mutex);
    Log_Message(LOG_LEVEL_WARNING, "Warning: Device not found (Type: %d, ID: %d).", type, id);
    return NULL;
}

void DeviceManager_Unregister(Device_Manager_t* mgr, Device_Handle_t* handle) {
    // 注销操作在嵌入式系统中通常不常用，且会使数组管理复杂化（产生空洞）。
    // 在此保留函数框架，但一般不建议使用。如果确实需要，需要更复杂的逻辑来处理数组空洞。
    Log_Message(LOG_LEVEL_WARNING, "Warning: DeviceManager_Unregister is not fully implemented and not recommended for use.");
}
