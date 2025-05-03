/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 19:10:06
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 15:00:00
 * @FilePath: \Demo\Middlewares\Src\device_manager.c
 * @Description: 设备管理器实现
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "device_manager.h"
#include "log_system.h"
#include "pch.h"

/**
 * @brief 初始化设备管理器
 * @param mgr 设备管理器实例
 * @param device_array 设备句柄数组
 * @param max_size 最大设备数
 */
void DeviceManager_Init(Device_Manager_t *mgr, Device_Handle_t *device_array, uint8_t max_size)
{
    if (!mgr || !device_array || max_size == 0)
    {
        Log_Message(LOG_LEVEL_ERROR, "[DeviceMgr] Invalid parameters");
        return;
    }

    mgr->devices = device_array;
    mgr->max_devices = max_size;
    mgr->count = 0;
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
 * @param mgr 设备管理器实例
 * @param device 设备实例
 * @param type 设备类型
 * @param id 设备 ID
 * @return Device_Handle_t* 设备句柄，失败返回 NULL
 */
Device_Handle_t *DeviceManager_Register(Device_Manager_t *mgr, void *device, Device_Type_t type, uint8_t id)
{
    if (!mgr || !device || mgr->count >= mgr->max_devices)
    {
        Log_Message(LOG_LEVEL_ERROR, "[DeviceMgr] Invalid device or max devices reached");
        return NULL;
    }

    for (uint8_t i = 0; i < mgr->max_devices; i++)
    {
        if (mgr->devices[i].device == NULL)
        {
            mgr->devices[i].device = device;
            mgr->devices[i].type = type;
            mgr->devices[i].id = id;
            mgr->devices[i].status = DEVICE_STATUS_NOT_INITIALIZED;
            mgr->count++;
            Log_Message(LOG_LEVEL_INFO, "[DeviceMgr] Registered device type %d, ID %d", type, id);
            return &mgr->devices[i];
        }
    }
    Log_Message(LOG_LEVEL_ERROR, "[DeviceMgr] No free slots available");
    return NULL;
}

/**
 * @brief 查找设备
 * @param mgr 设备管理器实例
 * @param type 设备类型
 * @param id 设备 ID
 * @return Device_Handle_t* 设备句柄，未找到返回 NULL
 */
Device_Handle_t *DeviceManager_Find(Device_Manager_t *mgr, Device_Type_t type, uint8_t id)
{
    if (!mgr)
    {
        Log_Message(LOG_LEVEL_ERROR, "[DeviceMgr] Invalid manager");
        return NULL;
    }

    for (uint8_t i = 0; i < mgr->max_devices; i++)
    {
        if (mgr->devices[i].device != NULL &&
            mgr->devices[i].type == type &&
            mgr->devices[i].id == id)
        {
            return &mgr->devices[i];
        }
    }
    Log_Message(LOG_LEVEL_WARNING, "[DeviceMgr] Device not found: type %d, ID %d", type, id);
    return NULL;
}

/**
 * @brief 注销设备
 * @param mgr 设备管理器实例
 * @param handle 设备句柄
 */
void DeviceManager_Unregister(Device_Manager_t *mgr, Device_Handle_t *handle)
{
    if (!mgr || !handle)
    {
        Log_Message(LOG_LEVEL_ERROR, "[DeviceMgr] Invalid parameters");
        return;
    }

    for (uint8_t i = 0; i < mgr->max_devices; i++)
    {
        if (&mgr->devices[i] == handle)
        {
            Log_Message(LOG_LEVEL_INFO, "[DeviceMgr] Unregistered device type %d, ID %d",
                        mgr->devices[i].type, mgr->devices[i].id);
            mgr->devices[i].device = NULL;
            mgr->devices[i].type = DEVICE_TYPE_NONE;
            mgr->devices[i].id = 0;
            mgr->devices[i].status = DEVICE_STATUS_NOT_INITIALIZED;
            mgr->count--;
            break;
        }
    }
}

/**
 * @brief 检查设备状态
 * @param handle 设备句柄
 * @return Device_Status_t 设备状态
 */
Device_Status_t DeviceManager_CheckStatus(Device_Handle_t *handle)
{
    if (!handle || !handle->device)
    {
        Log_Message(LOG_LEVEL_ERROR, "[DeviceMgr] Invalid handle");
        return DEVICE_STATUS_ERROR;
    }

    switch (handle->type)
    {
    case DEVICE_TYPE_SERIAL:
        if (((Serial_Device_t *)handle->device)->instance != NULL)
        {
            handle->status = DEVICE_STATUS_OK;
        }
        else
        {
            handle->status = DEVICE_STATUS_NOT_INITIALIZED;
        }
        break;
    case DEVICE_TYPE_EEPROM:
        if (((IIC_Ops_t *)handle->device)->dev_addr == EEPROM_ADDR)
        {
            handle->status = DEVICE_STATUS_OK;
        }
        else
        {
            handle->status = DEVICE_STATUS_NOT_INITIALIZED;
        }
        break;
    case DEVICE_TYPE_PCF8574:
        if (((IIC_Ops_t *)handle->device)->dev_addr == PCF8574_ADDR)
        {
            handle->status = DEVICE_STATUS_OK;
        }
        else
        {
            handle->status = DEVICE_STATUS_NOT_INITIALIZED;
        }
        break;
    case DEVICE_TYPE_CAN:
        if (((CAN_Device_t *)handle->device)->instance != NULL)
        {
            handle->status = DEVICE_STATUS_OK;
        }
        else
        {
            handle->status = DEVICE_STATUS_NOT_INITIALIZED;
        }
        break;
    case DEVICE_TYPE_WIFI:
        if (((Serial_Device_t *)handle->device)->instance != NULL)
        {
            handle->status = DEVICE_STATUS_OK;
        }
        else
        {
            handle->status = DEVICE_STATUS_NOT_INITIALIZED;
        }
        break;
    default:
        handle->status = DEVICE_STATUS_NOT_INITIALIZED;
        break;
    }

    Log_Message(LOG_LEVEL_INFO, "[DeviceMgr] Device type %d, ID %d, status: %d",
                handle->type, handle->id, handle->status);
    return handle->status;
}

/*
 * 示例用法：
 * 1. 初始化设备管理器
 * Device_Manager_t mgr;
 * Device_Handle_t devices[MAX_DEVICES];
 * DeviceManager_Init(&mgr, devices, MAX_DEVICES);
 *
 * 2. 注册设备
 * Serial_Device_t serial_dev = {...};
 * DeviceManager_Register(&mgr, &serial_dev, DEVICE_TYPE_SERIAL, 1);
 *
 * 3. 查找设备
 * Device_Handle_t *handle = DeviceManager_Find(&mgr, DEVICE_TYPE_SERIAL, 1);
 *
 * 4. 检查设备状态
 * Device_Status_t status = DeviceManager_CheckStatus(handle);
 */
