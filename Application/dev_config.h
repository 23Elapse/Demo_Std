/**
 * =====================================================================================
 * @file        dev_config.h
 * @brief       声明项目中所有硬件设备实例，作为全局配置的唯一头文件
 * @author      23Elapse & Gemini
 * @version     1.1
 * @date        2025-06-08
 * =====================================================================================
 */

#ifndef __DEV_CONFIG_H
#define __DEV_CONFIG_H

// 包含所有需要被配置的设备驱动头文件
#include "serial_driver.h"
#include "can_driver.h"
#include "i2c_driver.h"
#include "spi_flash.h"
#include "api_wifi.h" // 因为 ESP32 设备定义需要它

/*
 * =====================================================================================
 * 设备实例外部声明
 * =====================================================================================
 */

// --- 串口设备 ---
extern Serial_Device_t g_rs485_serial; // 用于RS485通信的底层串口
extern Serial_Device_t g_uart_dev;     // 另一个通用目的的串口
extern Serial_Device_t g_esp32_serial; // 用于与ESP32通信的底层串口

// --- CAN 设备 ---
extern CAN_Device_t g_can1_dev;

// --- I2C 总线 ---
// 注意：只声明总线本身。具体的I2C从设备（如PCF8574, EEPROM）
// 将通过总线句柄和其设备地址来访问，不再需要单独的全局实例。
extern I2C_Bus_t g_i2c1_bus;

// --- SPI Flash 设备 ---
extern SPI_Flash_Device_t g_spi_flash_dev;

// --- 复合设备 ---
extern ESP32_Shared_Device_t g_esp32_dev; // ESP32 复合设备

#endif // __DEV_CONFIG_H
