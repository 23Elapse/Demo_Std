/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-05-02 10:00:00
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-04 17:00:00
 * @FilePath: \Demo\Middlewares\Src\tsk_wifi.c
 * @Description: WiFi 任务实现，基于 RTOS 抽象层
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "tsk_wifi.h"
#include "api_wifi.h"
#include "log_system.h"
#include "serial_driver.h"
#include "pch.h"
/**
 * @brief WiFi 命令队列
 */
QueueHandle_t cmdQueue;

/**
 * @brief WiFi 任务函数
 * @param pvParameters 任务参数（WiFi 设备实例）
 */
void vWifiTask(void *pvParameters)
{
    WiFi_Device_t *device = (WiFi_Device_t *)pvParameters;
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid RTOS ops");
        rtos_ops->TaskDelete(NULL);
    }

    if (WiFi_Init(device) != AT_ERR_NONE) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Initialization failed");
        rtos_ops->TaskDelete(NULL);
    }

    static const AT_Cmd_Config at_cmd_table[] = {
        {"AT\r\n", "OK", 1000, 3, "AT command test"},
        {"AT+GMR\r\n", "OK", 2000, 2, "Check the firmware version"},
        {"AT+CWMODE=1\r\n", "OK", 2000, 2, "Setting Station Mode"},
        {"AT+CWJAP=\"" WIFI_SSID "\",\"" WIFI_PASSWORD "\"\r\n", "OK", 10000, 3, "Connecting to a Router"},
        {"AT+CIPSTA?\r\n", "+CIPSTA:", 2000, 1, "Query IP address"},
        {"AT+CWJAP?\r\n", "+CWJAP:", 2000, 2, "Query connection status"},
        {"AT+CWJAP_CUR?\r\n", "+CWJAP_CUR:", 3000, 2, "Query current WiFi"},
        {"AT+CWLAP\r\n", "+CWLAP:", 10000, 1, "Scan WiFi networks"},
        {"AT+CWLAPOPT=1\r\n", "OK", 2000, 2, "Set scan options"},
        {"AT+CWLAPOPT?\r\n", "+CWLAPOPT:", 2000, 2, "Query scan options"},
        {"AT+CIPSTA_CUR?\r\n", "+CIPSTA_CUR:", 2000, 2, "Query current IP address"},
        {"AT+CIPSTAMAC_CUR?\r\n", "+CIPSTAMAC_CUR:", 2000, 2, "Query current MAC address"},
        {"AT+CIPSTO=10\r\n", "OK", 2000, 2, "Set timeout"},
        {"AT+CIPSTART=\"TCP\",\"" TCP_SERVER_IP "\"," TCP_PORT "\r\n", "OK", 5000, 3, "Connect to TCP server"},
        {NULL, NULL, 0, 0, NULL}
    };

    while (1) {
        // 处理命令队列
        WiFi_Cmd_t cmd;
        if (rtos_ops->ReceiveQueue(cmdQueue, &cmd, 0) == 1) {
            AT_Cmd_Config at_cmd = {
                .at_cmd = cmd.cmd,
                .expected_resp = "OK",
                .timeout_ms = UART_TIMEOUT,
                .retries = 3,
                .description = "Queue command"
            };
            if (WiFi_SendATCommand(device, &at_cmd) != AT_ERR_NONE) {
                Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send command: %s", cmd.cmd);
            } else {
                Log_Message(LOG_LEVEL_INFO, "[WiFi] Command sent: %s", cmd.cmd);
            }
        }

        // 执行周期性 AT 指令
        for (const AT_Cmd_Config *cmd = at_cmd_table; cmd->at_cmd != NULL; cmd++) {
            if (WiFi_SendATCommand(device, cmd) != AT_ERR_NONE) {
                Log_Message(LOG_LEVEL_ERROR, "[WiFi] Command failed: %s", cmd->description);
            }
            rtos_ops->Delay(100);
        }

        // 查询 WiFi 状态和信号强度
        WiFi_Status_t status = {0};
        if (WiFi_QueryStatus(device, &status) == AT_ERR_NONE && status.connected) {
            Log_Message(LOG_LEVEL_INFO, "[WiFi] Connected to SSID: %s, IP: %s", status.ssid, status.ip_addr);
            int8_t rssi;
            if (WiFi_QuerySignalStrength(device, &rssi) == AT_ERR_NONE) {
                Log_Message(LOG_LEVEL_INFO, "[WiFi] Signal strength: %d dBm", rssi);
            }

            // TCP 数据传输示例
            if (WiFi_ConnectTCPServer(device, TCP_SERVER_IP, TCP_PORT) == AT_ERR_NONE) {
                uint8_t data[] = "Hello, Server!";
                uint16_t data_len = strlen((char*)data);
                if (WiFi_SendTCPData(device, data, data_len) == AT_ERR_NONE) {
                    uint8_t rx_buffer[TCP_BUFFER_SIZE];
                    uint16_t rx_len = TCP_BUFFER_SIZE;
                    if (WiFi_ReceiveTCPData(device, rx_buffer, &rx_len, 5000) == AT_ERR_NONE) {
                        Log_Message(LOG_LEVEL_INFO, "[WiFi] Received TCP data: %.*s", rx_len, rx_buffer);
                    }
                }
                WiFi_DisconnectTCPServer(device);
            }
        } else {
            Log_Message(LOG_LEVEL_WARNING, "[WiFi] Not connected to any network");
        }

        rtos_ops->Delay(1000 * 60); // 每隔 1 分钟执行一次
    }
}

///**
// * @brief 默认 WiFi 操作接口
// */
//static WiFi_Ops_t default_wifi_ops = {
//    .init = default_wifi_init,
//    .send_data = default_wifi_send_data,
//    .read_data = default_wifi_read_data,
//    .disconnect_tcp = default_wifi_disconnect_tcp,
//    .query_status = default_wifi_query_status,
//    .query_signal = default_wifi_query_signal
//};

/**
 * @brief 初始化 WiFi 任务和队列
 * @param device WiFi 设备实例
 * @param serial_dev 串口设备实例
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_TaskInit(WiFi_Device_t *device, Serial_Device_t *serial_dev)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !device || !serial_dev) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid RTOS, device, or serial_dev");
        return AT_ERR_SEND_FAILED;
    }
    if (!rtos_ops->SemaphoreTake(IIC1_config.mutex, 100))
    {
       printf("[IIC%d] Failed to take mutex, timeout after 100ms\n", IIC1_config.instance_id);
       return 1;
    }

    if (WiFi_SetOps(device, &default_wifi_ops, serial_dev) != AT_ERR_NONE) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to set ops");
        return AT_ERR_SEND_FAILED;
    }

    cmdQueue = rtos_ops->CreateQueue(CMD_QUEUE_SIZE, sizeof(WiFi_Cmd_t));
    if (!cmdQueue) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to create command queue");
        return AT_ERR_SEND_FAILED;
    }

    void *task_handle = rtos_ops->TaskCreate(vWifiTask, "WiFiTask", WIFI_TASK_STACK_SIZE, device, 1);
    if (!task_handle) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to create WiFi task");
        rtos_ops->DeleteQueue(cmdQueue);
        return AT_ERR_SEND_FAILED;
    }
    rtos_ops->SemaphoreGive(IIC1_config.mutex);
    Log_Message(LOG_LEVEL_INFO, "[WiFi] Task and queue initialized");
    return AT_ERR_NONE;
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
 * 3. 初始化 WiFi 任务
 * WiFi_Device_t wifi_device = {0};
 * WiFi_TaskInit(&wifi_device, &wifi_serial);
 *
 * 4. 发送命令
 * WiFi_Cmd_t cmd = {.cmd = "AT\r\n"};
 * xQueueSend(cmdQueue, &cmd, portMAX_DELAY);
 */

