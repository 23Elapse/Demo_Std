/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 19:10:06
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-04 15:09:25
 * @FilePath: \Demo\Drivers\BSP\Inc\device_manager.h
 * @Description: 设备管理器头文件
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __DEVICE_MANAGER_H
#define __DEVICE_MANAGER_H

#include <stdint.h>
#include "api_wifi.h"
#include "rtos_abstraction.h"

/**
 * @brief 设备类型枚举
 */
typedef enum
{
    DEVICE_TYPE_NONE = 0,
    DEVICE_TYPE_SERIAL,
    DEVICE_TYPE_EEPROM,
    DEVICE_TYPE_PCF8574,
    DEVICE_TYPE_CAN,
    DEVICE_TYPE_SPI_FLASH,
    DEVICE_TYPE_WIFI,
    DEVICE_TYPE_BLE,
    DEVICE_TYPE_ESP32,
    DEVICE_TYPE_MAX
} Device_Type_t;

/**
 * @brief 设备状态枚举
 */
typedef enum
{
    DEVICE_STATUS_OK = 0,
    DEVICE_STATUS_NOT_INITIALIZED,
    DEVICE_STATUS_ERROR
} Device_Status_t;

/**
 * @brief 设备句柄结构体
 */
typedef struct
{
    void *device;           // 设备实例指针（如 Serial_Device_t 或 WiFi_Device_t）
    Device_Type_t type;     // 设备类型
    uint8_t id;             // 设备 ID
    Device_Status_t status; // 设备状态
} Device_Handle_t;

/**
 * @brief 设备管理器结构体
 */
typedef struct
{
    Device_Handle_t *devices; // 设备句柄数组
    uint8_t max_devices;      // 最大设备数
    uint8_t count;            // 当前设备数
    void *mutex;              // RTOS 互斥锁
} Device_Manager_t;

/**
 * @brief 初始化设备管理器
 * @param mgr 设备管理器实例
 * @param device_array 设备句柄数组
 * @param max_size 最大设备数
 */
void DeviceManager_Init(Device_Manager_t *mgr, Device_Handle_t *device_array, uint8_t max_size);

/**
 * @brief 注册设备
 * @param mgr 设备管理器实例
 * @param device 设备实例
 * @param type 设备类型
 * @param id 设备 ID
 * @return Device_Handle_t* 设备句柄，失败返回 NULL
 */
Device_Handle_t *DeviceManager_Register(Device_Manager_t *mgr, const void *device, Device_Type_t type, uint8_t id);

/**
 * @brief 查找设备
 * @param mgr 设备管理器实例
 * @param type 设备类型
 * @param id 设备 ID
 * @return Device_Handle_t* 设备句柄，未找到返回 NULL
 */
Device_Handle_t *DeviceManager_Find(Device_Manager_t *mgr, Device_Type_t type, uint8_t id);

/**
 * @brief 注销设备
 * @param mgr 设备管理器实例
 * @param handle 设备句柄
 */
void DeviceManager_Unregister(Device_Manager_t *mgr, Device_Handle_t *handle);

/**
 * @brief 检查设备状态
 * @param handle 设备句柄
 * @return Device_Status_t 设备状态
 */
Device_Status_t DeviceManager_CheckStatus(Device_Handle_t *handle);

#endif /* __DEVICE_MANAGER_H */

