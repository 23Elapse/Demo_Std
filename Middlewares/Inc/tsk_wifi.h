/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-05-02 10:00:00
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 00:55:40
 * @FilePath: \Demo\Middlewares\Inc\tsk_wifi.h
 * @Description: WiFi 任务头文件
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __TSK_WIFI_H
#define __TSK_WIFI_H

#include "FreeRTOS.h"
#include "queue.h"

#define WIFI_TASK_STACK_SIZE 256
#define CMD_QUEUE_SIZE 10

/**
 * @brief WiFi 命令队列
 */
extern QueueHandle_t cmdQueue;

/**
 * @brief WiFi 任务函数
 * @param pvParameters 任务参数
 */
void vWifiTask(void *pvParameters);

#endif /* __TSK_WIFI_H */