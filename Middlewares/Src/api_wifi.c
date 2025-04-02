/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-01 20:50:17
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-01 20:57:05
 * @FilePath: \Demo\Middlewares\Src\api_wifi.c
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
#include "api_wifi.h"

// 连接到WiFi
void connectToWiFi(const char* ssid, const char* password) {
    char cmd[128];
    sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password);
    xQueueSend(cmdQueue, cmd, portMAX_DELAY);
}

// 启动TCP服务器（ESP32-C1作为热点）
void startAP() {
    char cmd[] = "AT+CWMODE=2\r\n"; // 设置AP模式
    xQueueSend(cmdQueue, cmd, portMAX_DELAY);
}