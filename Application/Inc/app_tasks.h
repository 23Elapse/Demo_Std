/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 19:10:06
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-04 17:00:00
 * @FilePath: \Demo\Application\Inc\app_tasks.h
 * @Description: 应用任务头文件
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __APP_TASKS_H
#define __APP_TASKS_H

#include "device_manager.h"
#include "api_eeprom.h"
#include "api_wifi.h"
#include "can_driver.h"
#include "serial_driver.h"
#include "rtos_abstraction.h"
#include "log_system.h"
#include "spi_flash.h"
#include "tsk_wifi.h"

extern Serial_Device_t RS485_Device;
extern Serial_Device_t UART_Device;
/**
 * @brief 初始化应用任务
 */
void App_Init(void);

/**
 * @brief RS485 轮询任务
 * @param pvParameters 任务参数（串口设备实例）
 */
void App_RS485_PollTask(void *pvParameters);

/**
 * @brief 串口接收任务
 * @param pvParameters 任务参数（串口设备实例）
 */
void App_SerialRxTask(void *pvParameters);

/**
 * @brief 错误日志任务
 * @param pvParameters 任务参数
 */
void App_ErrorLogTask(void *pvParameters);

/**
 * @brief EEPROM 管理任务
 * @param pvParameters 任务参数
 */
void App_EEPROMTask(void *pvParameters);

/**
 * @brief WiFi 管理任务
 * @param pvParameters 任务参数（WiFi 设备实例）
 */
void App_WifiTask(void *pvParameters);

/**
 * @brief CAN 管理任务
 * @param pvParameters 任务参数（CAN 设备实例）
 */
void App_CANTask(void *pvParameters);

/**
 * @brief SPI Flash 管理任务
 * @param pvParameters 任务参数（SPI Flash 设备实例）
 */
void App_SPIFlashTask(void *pvParameters);

/**
 * @brief 初始化 WiFi 任务
 * @param device WiFi 设备实例
 * @param serial_dev 串口设备实例
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_TaskInit(WiFi_Device_t *device, Serial_Device_t *serial_dev);

#endif /* __APP_TASKS_H */
