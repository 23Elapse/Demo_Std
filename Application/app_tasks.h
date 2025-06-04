/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 19:10:06
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-04 21:33:01
 * @FilePath: \Demo\Application\app_tasks.h
 * @Description: 应用任务头文件
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __APP_TASKS_H
#define __APP_TASKS_H

#include "device_manager.h"
#include "api_wifi.h"       // 应包含更名为 api_esp32.h (如果已重命名)
#include "can_driver.h"
#include "serial_driver.h"
#include "rtos_abstraction.h"
#include "log_system.h"
#include "spi_flash_driver.h" // 假设包含 SPI_Flash_Device_t

// 全局设备实例的 extern 声明
extern Serial_Device_t RS485_Device;
extern Serial_Device_t UART_Device;
extern Serial_Device_t ESP32_Serial; // ESP32模块的底层串口
extern CAN_Device_t CAN1_Device;
extern SPI_Flash_Device_t SPIFlash_Device; // 假设有这个全局实例
// ESP32_Device 的 extern 声明已在 api_wifi.h (或 api_esp32.h) 中

// 移除旧的独立 WiFi_Device_t 和 BLE_Device_t extern 声明
// extern WiFi_Device_t WiFi_Device; // <--- 移除
// extern BLE_Device_t BLE_Device;   // <--- 移除


// 中断服务程序原型
extern void USART1_IRQHandler(void);
extern void USART2_IRQHandler(void);
extern void USART6_IRQHandler(void);
extern void CAN1_RX0_IRQHandler(void);

// 应用初始化函数
void App_Init(void);

// 任务函数原型
void App_RS485_PollTask(void *pvParameters);
void App_SerialRxTask(void *pvParameters);
void App_ErrorLogTask(void *pvParameters);
void App_WifiTask(void *pvParameters);
void App_BLETask(void *pvParameters);
void App_CANTask(void *pvParameters);
void App_SPIFlashTask(void *pvParameters);

#endif /* __APP_TASKS_H */
