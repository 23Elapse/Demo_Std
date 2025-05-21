/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-01 20:52:45
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-04 17:00:00
 * @FilePath: \Demo\Middlewares\Inc\api_wifi.h
 * @Description: WiFi 模块驱动头文件，基于 RTOS 抽象层
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
#define WIFI_SSID       "104"
#define WIFI_PASSWORD   "ABC104104"
#define TCP_SERVER_IP   "192.168.2.100"
#define TCP_PORT        "5000"
#define UART_TIMEOUT    5000  // 5 秒
#define TCP_BUFFER_SIZE 256   // TCP 数据缓冲区大小

/**
 * @brief AT 指令错误码枚举
 */
typedef enum {
    AT_ERR_NONE,
    AT_ERR_TIMEOUT,
    AT_ERR_UNEXPECTED_RESPONSE,
    AT_ERR_SEND_FAILED,
    AT_ERR_CONNECTION_FAILED,
    AT_ERR_GET_MUTEX_FAILED
} AT_Error_Code;

/**
 * @brief AT 指令配置结构体
 */
typedef struct {
    const char* at_cmd;          // AT 指令（带完整格式）
    const char* expected_resp;   // 预期响应内容
    uint32_t timeout_ms;         // 超时时间（毫秒）
    uint8_t retries;             // 最大重试次数
    const char* description;     // 指令描述
} AT_Cmd_Config;

/**
 * @brief WiFi 状态结构体
 */
typedef struct {
    uint8_t connected;           // 连接状态（0: 未连接, 1: 已连接）
    char ssid[32];               // 当前 SSID
    char ip_addr[16];            // 当前 IP 地址
} WiFi_Status_t;

/**
 * @brief WiFi 操作接口
 */
typedef struct {
    AT_Error_Code (*init)(void *hw_context);                                      // 初始化 WiFi 模块
    AT_Error_Code (*send_data)(void *hw_context, const uint8_t *data, uint16_t length); // 发送数据
    int (*read_data)(void *hw_context, uint8_t *buffer, uint16_t *length);        // 读取数据
    AT_Error_Code (*disconnect_tcp)(void *hw_context);                            // 断开 TCP 连接
    AT_Error_Code (*query_status)(void *hw_context, WiFi_Status_t *status);       // 查询 WiFi 状态
    AT_Error_Code (*query_signal)(void *hw_context, int8_t *rssi);                // 查询信号强度
} WiFi_Ops_t;

/**
 * @brief WiFi 设备结构体
 */
typedef struct {
    void *hw_context;            // 硬件上下文（如 Serial_Device_t）
    void *mutex;                 // 互斥锁
    WiFi_Ops_t *ops;             // 操作接口
} WiFi_Device_t;

extern WiFi_Ops_t default_wifi_ops;

/**
 * @brief 设置 WiFi 操作接口
 * @param device WiFi 设备实例
 * @param ops 操作接口指针
 * @param hw_context 硬件上下文
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_SetOps(WiFi_Device_t *device, WiFi_Ops_t *ops, void *hw_context);

/**
 * @brief WiFi 初始化函数
 * @param device WiFi 设备实例
 * @return AT_Error_Code 初始化状态
 */
AT_Error_Code WiFi_Init(WiFi_Device_t *device);

/**
 * @brief 发送 AT 指令
 * @param device WiFi 设备实例
 * @param cmd AT 指令配置
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_SendATCommand(WiFi_Device_t *device, const AT_Cmd_Config *cmd);

/**
 * @brief 连接 TCP 服务器
 * @param device WiFi 设备实例
 * @param ip 服务器 IP 地址
 * @param port 服务器端口
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_ConnectTCPServer(WiFi_Device_t *device, const char *ip, const char *port);

/**
 * @brief 断开 TCP 服务器连接
 * @param device WiFi 设备实例
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_DisconnectTCPServer(WiFi_Device_t *device);

/**
 * @brief 查询 WiFi 连接状态
 * @param device WiFi 设备实例
 * @param status 输出连接状态
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_QueryStatus(WiFi_Device_t *device, WiFi_Status_t *status);

/**
 * @brief 查询 WiFi 信号强度
 * @param device WiFi 设备实例
 * @param rssi 输出信号强度（dBm）
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_QuerySignalStrength(WiFi_Device_t *device, int8_t *rssi);

/**
 * @brief 发送 TCP 数据
 * @param device WiFi 设备实例
 * @param data 要发送的数据
 * @param length 数据长度
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_SendTCPData(WiFi_Device_t *device, const uint8_t *data, uint16_t length);

/**
 * @brief 接收 TCP 数据
 * @param device WiFi 设备实例
 * @param buffer 数据存储缓冲区
 * @param length 期望接收的长度（输出实际接收长度）
 * @param timeout_ms 超时时间（毫秒）
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_ReceiveTCPData(WiFi_Device_t *device, uint8_t *buffer, uint16_t *length, uint32_t timeout_ms);

#endif /* __API_WIFI_H */
