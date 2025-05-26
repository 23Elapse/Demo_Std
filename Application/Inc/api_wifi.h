/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-01 20:52:45
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-25 19:53:00
 * @FilePath: \Demo\Middlewares\Inc\api_wifi.h
 * @Description: WiFi 和 BLE 模块驱动头文件
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __API_WIFI_H
#define __API_WIFI_H

#include "log_system.h"
#include "rtos_abstraction.h"
#include "serial_driver.h"

/**
 * @brief WiFi/BLE 配置宏
 */
#define WIFI_SSID       "104"
#define WIFI_PASSWORD   "ABC104104"
#define TCP_SERVER_IP   "192.168.2.100"
#define TCP_PORT        "5000"
#define UART_TIMEOUT    5000
#define TCP_BUFFER_SIZE 256

/**
 * @brief AT 指令错误码枚举
 */
typedef enum {
    AT_ERR_NONE,
    AT_ERR_TIMEOUT,
    AT_ERR_UNEXPECTED_RESPONSE,
    AT_ERR_SEND_FAILED,
    AT_ERR_CONNECTION_FAILED
} AT_Error_Code;

/**
 * @brief AT 指令配置结构体
 */
typedef struct {
    const char* at_cmd;
    const char* expected_resp;
    uint32_t timeout_ms;
    uint8_t retries;
    const char* description;
} AT_Cmd_Config;

/**
 * @brief WiFi 设备结构体
 */
typedef struct {
    Serial_Device_t *serial_dev;
    void *mutex;
} WiFi_Device_t;

/**
 * @brief BLE 设备结构体
 */
typedef struct {
    Serial_Device_t *serial_dev;
    void *mutex;
} BLE_Device_t;

/**
 * @brief 初始化 WiFi 和 BLE 硬件
 */
void atk_mb026_hw_init(void);

/**
 * @brief 硬件复位 WiFi/BLE 模块
 */
void atk_mb026_hw_reset(void);

/**
 * @brief 获取 WiFi 设备实例
 * @return WiFi_Device_t* WiFi 设备指针
 */
WiFi_Device_t* WiFi_GetDevice(void);

/**
 * @brief 获取 BLE 设备实例
 * @return BLE_Device_t* BLE 设备指针
 */
BLE_Device_t* BLE_GetDevice(void);

/**
 * @brief 发送 WiFi AT 指令
 * @param cmd AT 指令配置
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_SendATCommand(const AT_Cmd_Config *cmd);

/**
 * @brief 发送 WiFi TCP 数据
 * @param data 要发送的数据
 * @param length 数据长度
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_SendTCPData(const uint8_t *data, uint16_t length);

/**
 * @brief 接收 WiFi TCP 数据
 * @param buffer 数据存储缓冲区
 * @param length 期望接收的长度（输出实际接收长度）
 * @param timeout_ms 超时时间（毫秒）
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_ReceiveTCPData(uint8_t *buffer, uint16_t *length, uint32_t timeout_ms);

/**
 * @brief 发送 BLE AT 指令
 * @param cmd AT 指令配置
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code BLE_SendATCommand(const AT_Cmd_Config *cmd);

/**
 * @brief 发送 BLE 数据
 * @param data 要发送的数据
 * @param length 数据长度
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code BLE_SendData(const uint8_t *data, uint16_t length);

/**
 * @brief 接收 BLE 数据
 * @param buffer 数据存储缓冲区
 * @param length 期望接收的长度（输出实际接收长度）
 * @param timeout_ms 超时时间（毫秒）
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code BLE_ReceiveData(uint8_t *buffer, uint16_t *length, uint32_t timeout_ms);

#endif /* __API_WIFI_H */
