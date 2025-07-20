/**
 * =====================================================================================
 * @file        device_manager.h
 * @brief       设备管理器头文件 (已优化)
 * @author      23Elapse & Gemini
 * @version     2.0 (Refactored)
 * @date        2025-06-08
 * @note        统一管理和查找系统中的物理设备和总线。
 * =====================================================================================
 */
#ifndef __DEVICE_MANAGER_H
#define __DEVICE_MANAGER_H

#include <stdint.h>
#include "rtos_abstraction.h"

/**
 * @brief 设备类型枚举 (已精简和优化)
 * @note  只包含物理总线或独立的物理设备类型。
 */
typedef enum
{
    DEVICE_TYPE_NONE = 0,
    DEVICE_TYPE_SERIAL,       // 物理串口 (USART/UART)
    DEVICE_TYPE_CAN_BUS,      // CAN 总线
    DEVICE_TYPE_I2C_BUS,      // I2C 总线
    DEVICE_TYPE_SPI_BUS,      // SPI 总线
    DEVICE_TYPE_SPI_FLASH,    // SPI Flash 设备 (作为一个完整的设备)
    DEVICE_TYPE_ESP32,        // ESP32 复合设备
    DEVICE_TYPE_I2C_SLAVE,    // I2C 从设备 (如 PCF8574)
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
    void* device; // 指向设备/总线实例的指针 (例如 Serial_Device_t* 或 I2C_Bus_t*)
    Device_Type_t   type;   // 设备类型
    uint8_t         id;     // 设备ID (同类型设备中的唯一标识)
    Device_Status_t status; // 设备状态
} Device_Handle_t;

/**
 * @brief 设备管理器结构体
 */
typedef struct
{
    Device_Handle_t* devices;     // 设备句柄数组
    uint8_t          max_devices; // 数组最大容量
    uint8_t          count;       // 当前已注册的设备数量
    void* mutex;       // 用于保护管理器访问的互斥锁
} Device_Manager_t;

/**
 * @brief 初始化设备管理器
 * @param mgr 要初始化的设备管理器实例
 * @param device_array 用于存储设备句柄的数组
 * @param max_size 数组的最大容量
 */
void DeviceManager_Init(Device_Manager_t* mgr, Device_Handle_t* device_array, uint8_t max_size);

/**
 * @brief 注册一个新设备到管理器
 * @param mgr 设备管理器实例
 * @param device 指向设备实例的指针
 * @param type 设备类型
 * @param id 设备ID
 * @return Device_Handle_t* 成功则返回设备句柄，失败返回NULL
 */
Device_Handle_t* DeviceManager_Register(Device_Manager_t* mgr, const void* device, Device_Type_t type, uint8_t id);

/**
 * @brief 根据类型和ID查找设备
 * @param mgr 设备管理器实例
 * @param type 要查找的设备类型
 * @param id 要查找的设备ID
 * @return Device_Handle_t* 成功则返回设备句柄，未找到返回NULL
 */
Device_Handle_t* DeviceManager_Find(Device_Manager_t* mgr, Device_Type_t type, uint8_t id);

/**
 * @brief 从管理器中注销一个设备
 * @param mgr 设备管理器实例
 * @param handle 要注销的设备句柄
 */
void DeviceManager_Unregister(Device_Manager_t* mgr, Device_Handle_t* handle);


#endif /* __DEVICE_MANAGER_H */

