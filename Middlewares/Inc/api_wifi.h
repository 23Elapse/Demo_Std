/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-01 20:52:45
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 00:56:23
 * @FilePath: \Demo\Middlewares\Inc\api_wifi.h
 * @Description: WiFi 模块驱动头文件
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __API_WIFI_H
#define __API_WIFI_H

#include "log_system.h"
#include "rtos_abstraction.h"
#include "state_machine.h"

/**
 * @brief WiFi 配置宏
 */
#define WIFI_SSID "104"
#define WIFI_PASSWORD "ABC104104"
#define TCP_SERVER_IP "192.168.2.100"
#define TCP_PORT "5000"
#define UART_TIMEOUT 5000   // 5 秒
#define TCP_BUFFER_SIZE 256 // TCP 数据缓冲区大小

/**
 * @brief AT 指令错误码枚举
 */
typedef enum
{
    AT_ERR_NONE,
    AT_ERR_TIMEOUT,
    AT_ERR_UNEXPECTED_RESPONSE,
    AT_ERR_SEND_FAILED,
    AT_ERR_CONNECTION_FAILED
} AT_Error_Code;

/**
 * @brief AT 指令配置结构体
 */
typedef struct
{
    const char *at_cmd;        // AT 指令（带完整格式）
    const char *expected_resp; // 预期响应内容
    uint32_t timeout_ms;       // 超时时间（毫秒）
    uint8_t retries;           // 最大重试次数
    const char *description;   // 指令描述
} AT_Cmd_Config;

/**
 * @brief WiFi 初始化函数
 * @param serial_dev 串口设备实例
 * @return AT_Error_Code 初始化状态
 */
AT_Error_Code WiFi_Init(Serial_Device_t *serial_dev);

/**
 * @brief 发送 AT 指令
 * @param serial_dev 串口设备实例
 * @param cmd AT 指令配置
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_SendATCommand(Serial_Device_t *serial_dev, const AT_Cmd_Config *cmd);

/**
 * @brief 连接 TCP 服务器
 * @param serial_dev 串口设备实例
 * @param ip 服务器 IP 地址
 * @param port 服务器端口
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_ConnectTCPServer(Serial_Device_t *serial_dev, const char *ip, const char *port);

/**
 * @brief 发送 TCP 数据
 * @param serial_dev 串口设备实例
 * @param data 要发送的数据
 * @param length 数据长度
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_SendTCPData(Serial_Device_t *serial_dev, const uint8_t *data, uint16_t length);

/**
 * @brief 接收 TCP 数据
 * @param serial_dev 串口设备实例
 * @param buffer 数据存储缓冲区
 * @param length 期望接收的长度
 * @param timeout_ms 超时时间（毫秒）
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_ReceiveTCPData(Serial_Device_t *serial_dev, uint8_t *buffer, uint16_t *length, uint32_t timeout_ms);

/**
 * @brief WiFi 任务函数
 * @param pvParameters 任务参数
 */
void vWifiTask(void *pvParameters);

#endif /* __API_WIFI_H */