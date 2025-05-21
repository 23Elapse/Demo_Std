/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-05-02 10:00:00
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-04 20:11:59
 * @FilePath: \Demo\Middlewares\Inc\tsk_wifi.h
 * @Description: WiFi 任务头文件，基于 RTOS 抽象层
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __TSK_WIFI_H
#define __TSK_WIFI_H

#include "FreeRTOS.h"
#include "queue.h"
#include "api_wifi.h"
#include "serial_driver.h"

#define WIFI_TASK_STACK_SIZE 256
#define CMD_QUEUE_SIZE 10
#define CMD_BUFFER_SIZE 64

/**
 * @brief WiFi 命令结构体
 */
typedef struct {
    char cmd[CMD_BUFFER_SIZE];
} WiFi_Cmd_t;

/**
 * @brief WiFi 命令队列
 */
extern QueueHandle_t cmdQueue;

/**
 * @brief WiFi 任务函数
 * @param pvParameters 任务参数
 */
void vWifiTask(void *pvParameters);

/**
 * @brief 初始化 WiFi 任务和队列
 * @param device WiFi 设备实例
 * @param serial_dev 串口设备实例
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_TaskInit(WiFi_Device_t *device, Serial_Device_t *serial_dev);

#endif /* __TSK_WIFI_H */
