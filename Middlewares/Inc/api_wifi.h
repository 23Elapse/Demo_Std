/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-01 20:52:45
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-16 00:26:21
 * @FilePath: \Demo\Middlewares\Inc\api_wifi.h
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef __API_WIFI_H
#define __API_WIFI_H
#include "pch.h"

/* 配置参数 */
#define WIFI_SSID       "104"
#define WIFI_PASSWORD   "ABC104104"
#define TCP_SERVER_IP   "192.168.2.100"
#define TCP_PORT        "5000"
#define UART_TIMEOUT    5000  // 5秒

// 错误类型枚举（保持原样）
typedef enum {
    AT_ERR_NONE,                
    AT_ERR_TIMEOUT,             
    AT_ERR_UNEXPECTED_RESPONSE, 
    AT_ERR_SEND_FAILED          
} AT_Error_Code;


typedef struct {
    const char* at_cmd;          // AT指令（带完整格式）
    const char* expected_resp;   // 预期响应内容
    uint32_t timeout_ms;         // 超时时间（毫秒）
    uint8_t retries;             // 最大重试次数
    const char* description;     // 指令描述
} AT_Cmd_Config;

void vWifiTask(void const *argument);
#endif
