/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 19:10:06
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-04 21:33:45
 * @FilePath: \Demo\Middlewares\device_manager.c
 * @Description: 设备管理器实现
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "device_manager.h"
#include "log_system.h"
#include "api_wifi.h" // 需要包含 ESP32_Shared_Device_t 定义
#include "serial_driver.h"
#include "can_driver.h"
#include "spi_flash_driver.h" // 假设包含 SPI_Flash_Device_t 定义
// #include "api_eeprom.h" // 如果需要检查EEPROM特定结构
#include "pcf8574.h"    // 如果需要检查PCF8574特定结构
#include "rtos_abstraction.h"
#include <string.h>
#include "pch.h" // 假设包含 IIC_Ops_t, EEPROM_ADDR, PCF8574_ADDR (如果需要)


/**
 * @brief 初始化设备管理器
 */
void DeviceManager_Init(Device_Manager_t *mgr, Device_Handle_t *device_array, uint8_t max_size)
{
    if (!g_rtos_ops || !mgr || !device_array || max_size == 0)
    {
        Log_Message(LOG_LEVEL_ERROR, "[DeviceMgr] Init: Invalid parameters");
        return;
    }

    mgr->devices = device_array;
    mgr->max_devices = max_size;
    mgr->count = 0;
    mgr->mutex = g_rtos_ops->SemaphoreCreate();
    if (!mgr->mutex)
    {
        Log_Message(LOG_LEVEL_ERROR, "[DeviceMgr] Init: Failed to create mutex");
        // 严重错误，可能需要停机或特殊处理
        return;
    }

    for (uint8_t i = 0; i < max_size; i++)
    {
        mgr->devices[i].device = NULL;
        mgr->devices[i].type = DEVICE_TYPE_NONE;
        mgr->devices[i].id = 0;
        mgr->devices[i].status = DEVICE_STATUS_NOT_INITIALIZED;
    }
    Log_Message(LOG_LEVEL_INFO, "[DeviceMgr] Initialized with max devices: %d", max_size);
}

/**
 * @brief 注册设备
 */
Device_Handle_t *DeviceManager_Register(Device_Manager_t *mgr, const void *device, Device_Type_t type, uint8_t id)
{
    if (!g_rtos_ops || !mgr || !mgr->mutex || !device || mgr->count >= mgr->max_devices)
    {
        Log_Message(LOG_LEVEL_ERROR, "[DeviceMgr] Register: Invalid params or max devices reached (count %d, max %d)", mgr ? mgr->count : -1, mgr ? mgr->max_devices : -1);
        return NULL;
    }

    if (!g_rtos_ops->SemaphoreTake(mgr->mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[DeviceMgr] Register: Failed to take mutex");
        return NULL;
    }

    for (uint8_t i = 0; i < mgr->max_devices; i++)
    {
        if (mgr->devices[i].device == NULL) // 找到空槽
        {
            mgr->devices[i].device = (void*)device; // 移除 const 限定符以存储
            mgr->devices[i].type = type;
            mgr->devices[i].id = id;
            mgr->devices[i].status = DEVICE_STATUS_NOT_INITIALIZED; // 初始状态
            mgr->count++;
            Log_Message(LOG_LEVEL_INFO, "[DeviceMgr] Registered device type %d, ID %d at slot %d", type, id, i);
            g_rtos_ops->SemaphoreGive(mgr->mutex);
            return &mgr->devices[i];
        }
    }
    Log_Message(LOG_LEVEL_ERROR, "[DeviceMgr] Register: No free slots available");
    g_rtos_ops->SemaphoreGive(mgr->mutex);
    return NULL;
}

/**
 * @brief 查找设备
 */
Device_Handle_t *DeviceManager_Find(Device_Manager_t *mgr, Device_Type_t type, uint8_t id)
{
    if (!g_rtos_ops || !mgr || !mgr->mutex)
    {
        Log_Message(LOG_LEVEL_ERROR, "[DeviceMgr] Find: Invalid manager or mutex");
        return NULL;
    }

    if (!g_rtos_ops->SemaphoreTake(mgr->mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[DeviceMgr] Find: Failed to take mutex");
        return NULL;
    }

    for (uint8_t i = 0; i < mgr->count; i++) // 只需遍历已注册的设备 (mgr->count)
    {
        if (mgr->devices[i].device != NULL && // 确保槽位有效
            mgr->devices[i].type == type &&
            mgr->devices[i].id == id)
        {
            g_rtos_ops->SemaphoreGive(mgr->mutex);
            return &mgr->devices[i];
        }
    }
    Log_Message(LOG_LEVEL_WARNING, "[DeviceMgr] Device not found: type %d, ID %d", type, id);
    g_rtos_ops->SemaphoreGive(mgr->mutex);
    return NULL;
}

/**
 * @brief 注销设备
 */
void DeviceManager_Unregister(Device_Manager_t *mgr, Device_Handle_t *handle)
{
    if (!g_rtos_ops || !mgr || !mgr->mutex || !handle)
    {
        Log_Message(LOG_LEVEL_ERROR, "[DeviceMgr] Unregister: Invalid parameters");
        return;
    }

    if (!g_rtos_ops->SemaphoreTake(mgr->mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[DeviceMgr] Unregister: Failed to take mutex");
        return;
    }

    for (uint8_t i = 0; i < mgr->max_devices; i++) // 遍历所有槽位
    {
        if (&mgr->devices[i] == handle && mgr->devices[i].device != NULL)
        {
            Log_Message(LOG_LEVEL_INFO, "[DeviceMgr] Unregistered device type %d, ID %d from slot %d",
                        mgr->devices[i].type, mgr->devices[i].id, i);
            mgr->devices[i].device = NULL;
            mgr->devices[i].type = DEVICE_TYPE_NONE;
            mgr->devices[i].id = 0;
            mgr->devices[i].status = DEVICE_STATUS_NOT_INITIALIZED;
            mgr->count--; // 确保在找到并清除后才减少计数
            break;
        }
    }
    g_rtos_ops->SemaphoreGive(mgr->mutex);
}

/**
 * @brief 检查设备状态
 */
Device_Status_t DeviceManager_CheckStatus(Device_Handle_t *handle)
{
    if (!handle || !handle->device)
    {
        // Log_Message(LOG_LEVEL_ERROR, "[DeviceMgr] CheckStatus: Invalid handle or device pointer");
        // 对于未初始化的句柄，不打印错误，直接返回状态
        return DEVICE_STATUS_NOT_INITIALIZED;
    }

    // 状态检查应在获取设备句柄后，由调用者或特定设备初始化函数更新。
    // DeviceManager_CheckStatus 本身不应改变状态，而是读取已设置的状态，
    // 或执行一个非常轻量级的检查。
    // 此处简化为返回句柄中已有的状态，该状态应由设备初始化函数设置。
    // 如果需要动态检查，则需要更复杂的逻辑。

    // 示例：根据类型进行非常基础的指针检查（假设初始化会设置这些指针）
    switch (handle->type)
    {
    case DEVICE_TYPE_SERIAL:
        handle->status = (((Serial_Device_t *)handle->device)->instance != NULL) ? DEVICE_STATUS_OK : DEVICE_STATUS_NOT_INITIALIZED;
        break;
    case DEVICE_TYPE_CAN:
        handle->status = (((CAN_Device_t *)handle->device)->instance != NULL) ? DEVICE_STATUS_OK : DEVICE_STATUS_NOT_INITIALIZED;
        break;
    case DEVICE_TYPE_SPI_FLASH:
        handle->status = (((SPI_Flash_Device_t *)handle->device)->config != NULL && ((SPI_Flash_Device_t *)handle->device)->config->SPIx != NULL) ? DEVICE_STATUS_OK : DEVICE_STATUS_NOT_INITIALIZED;
        break;
    case DEVICE_TYPE_ESP32: // 新增对 ESP32 类型的检查
        {
            ESP32_Shared_Device_t *esp_dev = (ESP32_Shared_Device_t *)handle->device;
            handle->status = (esp_dev->serial_dev != NULL && esp_dev->mutex != NULL && esp_dev->reset_port != NULL) ? DEVICE_STATUS_OK : DEVICE_STATUS_NOT_INITIALIZED;
        }
        break;
    // 移除 DEVICE_TYPE_WIFI 和 DEVICE_TYPE_BLE，因为它们已被 DEVICE_TYPE_ESP32 替代
    // case DEVICE_TYPE_WIFI:
    // case DEVICE_TYPE_BLE:
    //     // ... old logic ...
    //     break;
    case DEVICE_TYPE_EEPROM: // 假设 EEPROM 和 PCF8574 通过 IIC_Ops_t 结构体访问
    case DEVICE_TYPE_PCF8574:
        // 这里的检查比较困难，因为 IIC_Ops_t 可能没有明显的 "instance" 成员
        // 依赖于初始化函数设置状态，或有一个通用的IIC设备活性检查函数
        // 暂时假设，如果device指针非NULL，则至少是注册了
        handle->status = (handle->device != NULL) ? DEVICE_STATUS_OK : DEVICE_STATUS_NOT_INITIALIZED;
        // 更具体的检查可能需要访问 IIC_Ops_t 内部，例如 dev_addr
        // if (handle->type == DEVICE_TYPE_EEPROM && ((IIC_Ops_t*)handle->device)->dev_addr == EEPROM_ADDR) handle->status = DEVICE_STATUS_OK;
        // else if (handle->type == DEVICE_TYPE_PCF8574 && ((IIC_Ops_t*)handle->device)->dev_addr == PCF8574_ADDR) handle->status = DEVICE_STATUS_OK;
        break;
    case DEVICE_TYPE_NONE:
    default:
        handle->status = DEVICE_STATUS_NOT_INITIALIZED;
        break;
    }

    // Log_Message(LOG_LEVEL_DEBUG, "[DeviceMgr] CheckStatus: Device type %d, ID %d, status: %d",
    //             handle->type, handle->id, handle->status);
    return handle->status;
}
