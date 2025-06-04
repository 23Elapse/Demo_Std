/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 19:10:06
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-04 20:43:21
 * @FilePath: \Demo\Application\app_tasks.h
 * @Description: 应用任务头文件
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __APP_TASKS_H
#define __APP_TASKS_H

#include "device_manager.h"
#include "api_wifi.h"
#include "can_driver.h"
#include "serial_driver.h"
#include "rtos_abstraction.h"
#include "log_system.h"
#include "spi_flash.h"

extern Serial_Device_t RS485_Device;
extern Serial_Device_t UART_Device;
extern Serial_Device_t ESP32_Serial;
extern CAN_Device_t CAN1_Device;

extern void USART1_IRQHandler(void);
extern void USART2_IRQHandler(void);
extern void USART6_IRQHandler(void);
extern void CAN1_RX0_IRQHandler(void);



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
 * @brief WiFi 管理任务
 * @param pvParameters 任务参数（未使用）
 * @note 管理 WiFi 模块的初始化和 TCP 通信
 */
void App_WifiTask(void *pvParameters);

/**
 * @brief BLE 管理任务
 * @param pvParameters 任务参数（未使用）
 * @note 管理 BLE 模块的初始化和数据通信
 */
void App_BLETask(void *pvParameters);

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

#endif /* __APP_TASKS_H */
