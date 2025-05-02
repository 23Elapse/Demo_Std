/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-01 20:50:17
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 00:56:15
 * @FilePath: \Demo\Middlewares\Src\api_wifi.c
 * @Description: WiFi 模块驱动实现
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "api_wifi.h"
#include "serial_driver.h"
#include "log_system.h"
#include "state_machine.h"
#include <string.h>
#include <stdio.h>

/**
 * @brief AT 指令配置表
 */
static const AT_Cmd_Config at_cmd_table[] = {
    {"AT\r\n", "OK", 1000, 3, "AT command test"},
    {"AT+GMR\r\n", "OK", 2000, 2, "Check the firmware version"},
    {"AT+CWMODE=1\r\n", "OK", 2000, 2, "Setting Station Mode"},
    {"AT+CWJAP=\"" WIFI_SSID "\",\"" WIFI_PASSWORD "\"\r\n", "OK", 10000, 3, "Connecting to a Router"},
    {"AT+CIPSTA?\r\n", "+CIPSTA:", 2000, 1, "Query IP address"},
    {"AT+CWJAP?\r\n", "OK", 2000, 2, "Query connection status"},
    {"AT+CWJAP_CUR?\r\n", "+CWJAP_CUR:", 3000, 2, "Query current WiFi"},
    {"AT+CWLAP\r\n", "+CWLAP:", 10000, 1, "Scan WiFi networks"},
    {"AT+CWLAPOPT=1\r\n", "OK", 2000, 2, "Set scan options"},
    {"AT+CWLAPOPT?\r\n", "+CWLAPOPT:", 2000, 2, "Query scan options"},
    {"AT+CIPSTA_CUR?\r\n", "+CIPSTA_CUR:", 2000, 2, "Query current IP address"},
    {"AT+CIPSTAMAC_CUR?\r\n", "+CIPSTAMAC_CUR:", 2000, 2, "Query current MAC address"},
    {"AT+CIPSTO=10\r\n", "OK", 2000, 2, "Set timeout"},
    {"AT+CIPSTART=\"TCP\",\"" TCP_SERVER_IP "\"," TCP_PORT "\r\n", "OK", 5000, 3, "Connect to TCP server"},
    {NULL, NULL, 0, 0, NULL}};

/**
 * @brief AT 指令状态机上下文
 */
typedef struct
{
    StateContext_t ctx;
    const AT_Cmd_Config *current_cmd;
    uint8_t retry_count;
    uint8_t rx_buffer[TCP_BUFFER_SIZE];
    uint16_t rx_len;
} AT_StateContext_t;

/**
 * @brief TCP 数据接收状态机上下文
 */
typedef struct
{
    StateContext_t ctx;
    uint8_t rx_buffer[TCP_BUFFER_SIZE];
    uint16_t rx_len;
    uint16_t expected_length;
} TCP_StateContext_t;

/**
 * @brief AT 指令状态机处理函数
 */
static int HandleATState_Init(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    AT_StateContext_t *at_ctx = (AT_StateContext_t *)user_data;
    at_ctx->rx_len = 0;
    at_ctx->retry_count = 0;
    ctx->current_state = STATE_START;
    return 1;
}

static int HandleATState_Start(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    AT_StateContext_t *at_ctx = (AT_StateContext_t *)user_data;
    at_ctx->rx_buffer[at_ctx->rx_len++] = byte;
    if (at_ctx->rx_len >= sizeof(at_ctx->rx_buffer) - 1)
    {
        at_ctx->rx_len = 0;
        ctx->current_state = STATE_INIT;
        return 0;
    }
    if (byte == '\n')
    {
        ctx->current_state = STATE_DATA;
    }
    return 1;
}

static int HandleATState_Data(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    AT_StateContext_t *at_ctx = (AT_StateContext_t *)user_data;
    at_ctx->rx_buffer[at_ctx->rx_len++] = byte;
    if (at_ctx->rx_len >= sizeof(at_ctx->rx_buffer) - 1)
    {
        at_ctx->rx_len = 0;
        ctx->current_state = STATE_INIT;
        return 0;
    }
    at_ctx->rx_buffer[at_ctx->rx_len] = '\0';
    if (strstr((char *)at_ctx->rx_buffer, at_ctx->current_cmd->expected_resp) != NULL)
    {
        ctx->current_state = STATE_END;
    }
    return 1;
}

static int HandleATState_End(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    AT_StateContext_t *at_ctx = (AT_StateContext_t *)user_data;
    Log_Message(LOG_LEVEL_INFO, "[WiFi] Received expected response for %s", at_ctx->current_cmd->description);
    ctx->current_state = STATE_INIT;
    return 1;
}

static StateTransition_t at_transitions[] = {
    {STATE_INIT, HandleATState_Init},
    {STATE_START, HandleATState_Start},
    {STATE_DATA, HandleATState_Data},
    {STATE_END, HandleATState_End},
    {0, NULL}};

/**
 * @brief TCP 数据接收状态机处理函数
 */
static int HandleTCPState_Init(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    TCP_StateContext_t *tcp_ctx = (TCP_StateContext_t *)user_data;
    tcp_ctx->rx_len = 0;
    ctx->current_state = STATE_START;
    return 1;
}

static int HandleTCPState_Start(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    TCP_StateContext_t *tcp_ctx = (TCP_StateContext_t *)user_data;
    if (byte == '+')
    {
        ctx->current_state = STATE_DATA;
    }
    return 1;
}

static int HandleTCPState_Data(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    TCP_StateContext_t *tcp_ctx = (TCP_StateContext_t *)user_data;
    tcp_ctx->rx_buffer[tcp_ctx->rx_len++] = byte;
    if (tcp_ctx->rx_len >= tcp_ctx->expected_length || tcp_ctx->rx_len >= sizeof(tcp_ctx->rx_buffer) - 1)
    {
        ctx->current_state = STATE_END;
    }
    return 1;
}

static int HandleTCPState_End(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    TCP_StateContext_t *tcp_ctx = (TCP_StateContext_t *)user_data;
    Log_Message(LOG_LEVEL_INFO, "[WiFi] TCP data received, length: %d", tcp_ctx->rx_len);
    ctx->current_state = STATE_INIT;
    return 1;
}

static StateTransition_t tcp_transitions[] = {
    {STATE_INIT, HandleTCPState_Init},
    {STATE_START, HandleTCPState_Start},
    {STATE_DATA, HandleTCPState_Data},
    {STATE_END, HandleTCPState_End},
    {0, NULL}};

/**
 * @brief 初始化 WiFi 模块
 * @param serial_dev 串口设备实例
 * @return AT_Error_Code 初始化状态
 */
AT_Error_Code WiFi_Init(Serial_Device_t *serial_dev)
{
    if (!serial_dev)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid serial device");
        return AT_ERR_SEND_FAILED;
    }

    if (Serial_Driver_Init(serial_dev) != SERIAL_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to initialize serial driver");
        return AT_ERR_SEND_FAILED;
    }

    Log_Message(LOG_LEVEL_INFO, "[WiFi] Initialized successfully");
    return AT_ERR_NONE;
}

/**
 * @brief 发送 AT 指令并等待响应
 * @param serial_dev 串口设备实例
 * @param cmd AT 指令配置
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_SendATCommand(Serial_Device_t *serial_dev, const AT_Cmd_Config *cmd)
{
    if (!serial_dev || !cmd || !cmd->at_cmd || !cmd->expected_resp)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for AT command");
        return AT_ERR_SEND_FAILED;
    }

    AT_StateContext_t at_ctx = {0};
    StateMachine_Init(&at_ctx.ctx, &at_ctx);
    at_ctx.current_cmd = cmd;

    uint8_t retry = 0;
    while (retry <= cmd->retries)
    {
        if (Serial_Driver_SendData(serial_dev, (uint8_t *)cmd->at_cmd, strlen(cmd->at_cmd)) != SERIAL_OK)
        {
            Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send command: %s", cmd->description);
            retry++;
            continue;
        }

        uint32_t start_tick = xTaskGetTickCount();
        while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(cmd->timeout_ms))
        {
            uint8_t byte;
            if (RingBuffer_IsAvailable(&serial_dev->rx_buffer))
            {
                RingBuffer_Read(&serial_dev->rx_buffer, &byte);
                StateMachine_Process(&at_ctx.ctx, at_transitions, byte);
                if (at_ctx.ctx.current_state == STATE_END)
                {
                    return AT_ERR_NONE;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        retry++;
        Log_Message(LOG_LEVEL_WARNING, "[WiFi] Timeout for command: %s, retry %d/%d", cmd->description, retry, cmd->retries);
    }

    Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to receive expected response for %s", cmd->description);
    return AT_ERR_TIMEOUT;
}

/**
 * @brief 连接 TCP 服务器
 * @param serial_dev 串口设备实例
 * @param ip 服务器 IP 地址
 * @param port 服务器端口
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_ConnectTCPServer(Serial_Device_t *serial_dev, const char *ip, const char *port)
{
    if (!serial_dev || !ip || !port)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for TCP connection");
        return AT_ERR_SEND_FAILED;
    }

    char cmd_str[64];
    snprintf(cmd_str, sizeof(cmd_str), "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", ip, port);
    AT_Cmd_Config tcp_cmd = {
        .at_cmd = cmd_str,
        .expected_resp = "OK",
        .timeout_ms = 5000,
        .retries = 3,
        .description = "Connect to TCP server"};

    AT_Error_Code status = WiFi_SendATCommand(serial_dev, &tcp_cmd);
    if (status != AT_ERR_NONE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to connect to TCP server");
        return AT_ERR_CONNECTION_FAILED;
    }

    Log_Message(LOG_LEVEL_INFO, "[WiFi] Connected to TCP server %s:%s", ip, port);
    return AT_ERR_NONE;
}

/**
 * @brief 发送 TCP 数据
 * @param serial_dev 串口设备实例
 * @param data 要发送的数据
 * @param length 数据长度
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_SendTCPData(Serial_Device_t *serial_dev, const uint8_t *data, uint16_t length)
{
    if (!serial_dev || !data || length == 0 || length > TCP_BUFFER_SIZE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for TCP send");
        return AT_ERR_SEND_FAILED;
    }

    char cmd_str[32];
    snprintf(cmd_str, sizeof(cmd_str), "AT+CIPSEND=%d\r\n", length);
    AT_Cmd_Config send_cmd = {
        .at_cmd = cmd_str,
        .expected_resp = ">",
        .timeout_ms = 2000,
        .retries = 2,
        .description = "Prepare to send TCP data"};

    AT_Error_Code status = WiFi_SendATCommand(serial_dev, &send_cmd);
    if (status != AT_ERR_NONE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to prepare TCP send");
        return status;
    }

    if (Serial_Driver_SendData(serial_dev, data, length) != SERIAL_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send TCP data");
        return AT_ERR_SEND_FAILED;
    }

    AT_Cmd_Config confirm_cmd = {
        .at_cmd = "",
        .expected_resp = "SEND OK",
        .timeout_ms = 2000,
        .retries = 2,
        .description = "Confirm TCP data send"};

    status = WiFi_SendATCommand(serial_dev, &confirm_cmd);
    if (status != AT_ERR_NONE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to confirm TCP send");
        return status;
    }

    Log_Message(LOG_LEVEL_INFO, "[WiFi] Sent %d bytes of TCP data", length);
    return AT_ERR_NONE;
}

/**
 * @brief 接收 TCP 数据
 * @param serial_dev 串口设备实例
 * @param buffer 数据存储缓冲区
 * @param length 期望接收的长度（输出实际接收长度）
 * @param timeout_ms 超时时间（毫秒）
 * @return AT_Error_Code 操作状态
 */
AT_Error_Code WiFi_ReceiveTCPData(Serial_Device_t *serial_dev, uint8_t *buffer, uint16_t *length, uint32_t timeout_ms)
{
    if (!serial_dev || !buffer || !length || *length == 0 || *length > TCP_BUFFER_SIZE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for TCP receive");
        return AT_ERR_SEND_FAILED;
    }

    TCP_StateContext_t tcp_ctx = {0};
    StateMachine_Init(&tcp_ctx.ctx, &tcp_ctx);
    tcp_ctx.expected_length = *length;

    uint32_t start_tick = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(timeout_ms))
    {
        uint8_t byte;
        if (RingBuffer_IsAvailable(&serial_dev->rx_buffer))
        {
            RingBuffer_Read(&serial_dev->rx_buffer, &byte);
            StateMachine_Process(&tcp_ctx.ctx, tcp_transitions, byte);
            if (tcp_ctx.ctx.current_state == STATE_END)
            {
                *length = tcp_ctx.rx_len;
                memcpy(buffer, tcp_ctx.rx_buffer, tcp_ctx.rx_len);
                Log_Message(LOG_LEVEL_INFO, "[WiFi] Received %d bytes of TCP data", tcp_ctx.rx_len);
                return AT_ERR_NONE;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    Log_Message(LOG_LEVEL_WARNING, "[WiFi] Timeout receiving TCP data");
    *length = 0;
    return AT_ERR_TIMEOUT;
}

/**
 * @brief WiFi 任务主函数
 * @param pvParameters 任务参数（串口设备实例）
 */
void vWifiTask(void *pvParameters)
{
    Serial_Device_t *serial_dev = (Serial_Device_t *)pvParameters;
    if (WiFi_Init(serial_dev) != AT_ERR_NONE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Initialization failed");
        vTaskDelete(NULL);
    }

    while (1)
    {
        for (const AT_Cmd_Config *cmd = at_cmd_table; cmd->at_cmd != NULL; cmd++)
        {
            if (WiFi_SendATCommand(serial_dev, cmd) != AT_ERR_NONE)
            {
                Log_Message(LOG_LEVEL_ERROR, "[WiFi] Command failed: %s", cmd->description);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // 示例：连接 TCP 服务器并发送/接收数据
        if (WiFi_ConnectTCPServer(serial_dev, TCP_SERVER_IP, TCP_PORT) == AT_ERR_NONE)
        {
            uint8_t data[] = "Hello, Server!";
            uint16_t data_len = strlen((char *)data);
            if (WiFi_SendTCPData(serial_dev, data, data_len) == AT_ERR_NONE)
            {
                uint8_t rx_buffer[TCP_BUFFER_SIZE];
                uint16_t rx_len = TCP_BUFFER_SIZE;
                if (WiFi_ReceiveTCPData(serial_dev, rx_buffer, &rx_len, 5000) == AT_ERR_NONE)
                {
                    Log_Message(LOG_LEVEL_INFO, "[WiFi] Received TCP data: %.*s", rx_len, rx_buffer);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000 * 60)); // 每隔 1 分钟执行一次
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
 * 3. 创建 WiFi 任务
 * xTaskCreate(vWifiTask, "WiFiTask", WIFI_TASK_STACK_SIZE, &wifi_serial, 1, NULL);
 *
 * 4. TCP 数据传输
 * WiFi_ConnectTCPServer(&wifi_serial, "192.168.2.100", "5000");
 * uint8_t data[] = "Test Data";
 * WiFi_SendTCPData(&wifi_serial, data, strlen(data));
 * uint8_t rx_buffer[256];
 * uint16_t rx_len = 256;
 * WiFi_ReceiveTCPData(&wifi_serial, rx_buffer, &rx_len, 5000);
 */