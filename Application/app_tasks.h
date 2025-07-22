/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 19:10:06
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-14 19:06:11
 * @FilePath: \Demo_backup\Application\app_tasks.h
 * @Description: 应用任务头文件 (Refactored)
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __APP_TASKS_H
#define __APP_TASKS_H

#include "device_manager.h"     // 设备管理器
#include "api_wifi.h"           // ESP32 (WiFi/BLE) 统一驱动
#include "can_driver.h"         // CAN 驱动
#include "serial_driver.h"      // 串口底层驱动
#include "rtos_abstraction.h"   // RTOS 抽象层
#include "log_system.h"         // 日志系统
#include "spi_flash.h"          // SPI Flash 驱动
#include "i2c_driver.h"         // I2C 驱动
#include "pcf8574.h"            // PCF8574 扩展IO驱动 (依赖I2C)

// 注意：全局设备实例的 extern 声明已统一在 dev_config.h 中，
// 任务函数中直接使用 dev_config.h 中声明的 g_xxx_dev 实例即可。
// 因此，以下旧的 extern 声明可以移除，避免冗余。
// extern Serial_Device_t RS485_Device;
// extern Serial_Device_t UART_Device;
// extern Serial_Device_t ESP32_Serial;
// extern CAN_Device_t CAN1_Device;
// extern SPI_Flash_Device_t SPIFlash_Device;

// 中断服务程序原型 (这些通常在 stm32f4xx_it.c 中定义，并调用驱动的 IRQHandler)
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
void App_WifiBLETask(void *pvParameters); // WiFi和BLE可以合并到一个任务，根据功能切换
void App_CANTask(void *pvParameters);
void App_SPIFlashTask(void *pvParameters);
void App_I2CTask(void *pvParameters); // 新增I2C任务，用于测试PCF8574

#endif /* __APP_TASKS_H */
