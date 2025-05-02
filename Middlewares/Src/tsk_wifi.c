/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-05-02 10:00:00
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 01:22:57
 * @FilePath: \Demo\Middlewares\Src\tsk_wifi.c
 * @Description: WiFi 任务实现
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "tsk_wifi.h"
#include "api_wifi.h"
#include "log_system.h"
#include "serial_driver.h"
/**
 * @brief WiFi 命令队列
 */
QueueHandle_t cmdQueue;

/**
 * @brief WiFi 任务函数
 * @param pvParameters 任务参数（串口设备实例）
 */
void vWifiTask(void *pvParameters)
{
    Serial_Device_t *serial_dev = (Serial_Device_t *)pvParameters;
    char cmdBuffer[64];

    if (WiFi_Init(serial_dev) != AT_ERR_NONE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Initialization failed");
        vTaskDelete(NULL);
    }

    while (1)
    {
        if (xQueueReceive(cmdQueue, cmdBuffer, portMAX_DELAY) == pdPASS)
        {
            if (Serial_Driver_SendData(serial_dev, (uint8_t *)cmdBuffer, strlen(cmdBuffer)) != SERIAL_OK)
            {
                Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send command: %s", cmdBuffer);
            }
            else
            {
                Log_Message(LOG_LEVEL_INFO, "[WiFi] Command sent: %s", cmdBuffer);
            }
        }
    }
}

/*
 * 示例用法：
 * 1. 设置 RTOS 抽象层
 * RTOS_SetOps(&FreeRTOS_Ops);
 *
 * 2. 初始化串口设备
 * Serial_Device_t wifi_serial = {...};
 * Serial_Driver_Init(&wifi_serial);
 *
 * 3. 创建命令队列
 * cmdQueue = xQueueCreate(CMD_QUEUE_SIZE, sizeof(char[64]));
 *
 * 4. 创建 WiFi 任务
 * xTaskCreate(vWifiTask, "WiFiTask", WIFI_TASK_STACK_SIZE, &wifi_serial, 1, NULL);
 *
 * 5. 发送命令
 * char cmd[] = "AT\r\n";
 * xQueueSend(cmdQueue, cmd, portMAX_DELAY);
 */