/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-01 20:50:17
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-04 17:00:00
 * @FilePath: \Demo\Middlewares\Src\api_wifi.c
 * @Description: WiFi 模块驱动实现，基于 RTOS 抽象层
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "api_wifi.h"
#include "serial_driver.h"
#include "log_system.h"
#include "state_machine.h"
#include <string.h>
#include <stdio.h>
#include "pch.h"

/**
 * @brief AT 指令配置表
 */
//static const AT_Cmd_Config at_cmd_table[] = {
//    {"AT\r\n", "OK", 1000, 3, "AT command test"},
//    {"AT+GMR\r\n", "OK", 2000, 2, "Check the firmware version"},
//    {"AT+CWMODE=1\r\n", "OK", 2000, 2, "Setting Station Mode"},
//    {"AT+CWJAP=\"" WIFI_SSID "\",\"" WIFI_PASSWORD "\"\r\n", "OK", 10000, 3, "Connecting to a Router"},
//    {"AT+CIPSTA?\r\n", "+CIPSTA:", 2000, 1, "Query IP address"},
//    {"AT+CWJAP?\r\n", "+CWJAP:", 2000, 2, "Query connection status"},
//    {"AT+CWJAP_CUR?\r\n", "+CWJAP_CUR:", 3000, 2, "Query current WiFi"},
//    {"AT+CWLAP\r\n", "+CWLAP:", 10000, 1, "Scan WiFi networks"},
//    {"AT+CWLAPOPT=1\r\n", "OK", 2000, 2, "Set scan options"},
//    {"AT+CWLAPOPT?\r\n", "+CWLAPOPT:", 2000, 2, "Query scan options"},
//    {"AT+CIPSTA_CUR?\r\n", "+CIPSTA_CUR:", 2000, 2, "Query current IP address"},
//    {"AT+CIPSTAMAC_CUR?\r\n", "+CIPSTAMAC_CUR:", 2000, 2, "Query current MAC address"},
//    {"AT+CIPSTO=10\r\n", "OK", 2000, 2, "Set timeout"},
//    {"AT+CIPSTART=\"TCP\",\"" TCP_SERVER_IP "\"," TCP_PORT "\r\n", "OK", 5000, 3, "Connect to TCP server"},
//    {NULL, NULL, 0, 0, NULL}
//};

/**
 * @brief AT 指令状态机上下文
 */
typedef struct {
    StateContext_t ctx;
    const AT_Cmd_Config *current_cmd;
    uint8_t retry_count;
    uint8_t rx_buffer[TCP_BUFFER_SIZE];
    uint16_t rx_len;
} AT_StateContext_t;

/**
 * @brief TCP 数据接收状态机上下文
 */
typedef struct {
    StateContext_t ctx;
    uint8_t rx_buffer[TCP_BUFFER_SIZE];
    uint16_t rx_len;
    uint16_t expected_length;
} TCP_StateContext_t;

/**
 * @brief 状态查询状态机上下文
 */
typedef struct {
    StateContext_t ctx;
    uint8_t rx_buffer[TCP_BUFFER_SIZE];
    uint16_t rx_len;
    WiFi_Status_t *status;
} Status_StateContext_t;

/**
 * @brief 信号强度查询状态机上下文
 */
typedef struct {
    StateContext_t ctx;
    uint8_t rx_buffer[TCP_BUFFER_SIZE];
    uint16_t rx_len;
    int8_t *rssi;
} Signal_StateContext_t;

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
    if (at_ctx->rx_len >= sizeof(at_ctx->rx_buffer) - 1) {
        at_ctx->rx_len = 0;
        ctx->current_state = STATE_INIT;
        return 0;
    }
    if (byte == '\n') {
        ctx->current_state = STATE_DATA;
    }
    return 1;
}

static int HandleATState_Data(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    AT_StateContext_t *at_ctx = (AT_StateContext_t *)user_data;
    at_ctx->rx_buffer[at_ctx->rx_len++] = byte;
    if (at_ctx->rx_len >= sizeof(at_ctx->rx_buffer) - 1) {
        at_ctx->rx_len = 0;
        ctx->current_state = STATE_INIT;
        return 0;
    }
    at_ctx->rx_buffer[at_ctx->rx_len] = '\0';
    if (strstr((char*)at_ctx->rx_buffer, at_ctx->current_cmd->expected_resp) != NULL) {
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
    {STATE_INIT, NULL}
};

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
//    TCP_StateContext_t *tcp_ctx = (TCP_StateContext_t *)user_data;
    if (byte == '+') {
        ctx->current_state = STATE_DATA;
    }
    return 1;
}

static int HandleTCPState_Data(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    TCP_StateContext_t *tcp_ctx = (TCP_StateContext_t *)user_data;
    tcp_ctx->rx_buffer[tcp_ctx->rx_len++] = byte;
    if (tcp_ctx->rx_len >= tcp_ctx->expected_length || tcp_ctx->rx_len >= sizeof(tcp_ctx->rx_buffer) - 1) {
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
    {STATE_INIT, NULL}
};

/**
 * @brief 状态查询状态机处理函数
 */
static int HandleStatusState_Init(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    Status_StateContext_t *status_ctx = (Status_StateContext_t *)user_data;
    status_ctx->rx_len = 0;
    status_ctx->status->connected = 0;
    status_ctx->status->ssid[0] = '\0';
    status_ctx->status->ip_addr[0] = '\0';
    ctx->current_state = STATE_START;
    return 1;
}

static int HandleStatusState_Start(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    Status_StateContext_t *status_ctx = (Status_StateContext_t *)user_data;
    status_ctx->rx_buffer[status_ctx->rx_len++] = byte;
    if (status_ctx->rx_len >= sizeof(status_ctx->rx_buffer) - 1) {
        status_ctx->rx_len = 0;
        ctx->current_state = STATE_INIT;
        return 0;
    }
    if (byte == '\n') {
        ctx->current_state = STATE_DATA;
    }
    return 1;
}

static int HandleStatusState_Data(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    Status_StateContext_t *status_ctx = (Status_StateContext_t *)user_data;
    status_ctx->rx_buffer[status_ctx->rx_len++] = byte;
    if (status_ctx->rx_len >= sizeof(status_ctx->rx_buffer) - 1) {
        status_ctx->rx_len = 0;
        ctx->current_state = STATE_INIT;
        return 0;
    }
    status_ctx->rx_buffer[status_ctx->rx_len] = '\0';
    if (strstr((char*)status_ctx->rx_buffer, "+CWJAP:") != NULL) {
        char *ssid_start = strstr((char*)status_ctx->rx_buffer, "\"");
        if (ssid_start) {
            ssid_start++;
            char *ssid_end = strstr(ssid_start, "\"");
            if (ssid_end) {
                size_t ssid_len = ssid_end - ssid_start;
                if (ssid_len < sizeof(status_ctx->status->ssid)) {
                    strncpy(status_ctx->status->ssid, ssid_start, ssid_len);
                    status_ctx->status->ssid[ssid_len] = '\0';
                    status_ctx->status->connected = 1;
                }
            }
        }
        ctx->current_state = STATE_END;
    } else if (strstr((char*)status_ctx->rx_buffer, "No AP") != NULL) {
        ctx->current_state = STATE_END;
    }
    return 1;
}

static int HandleStatusState_End(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    ctx->current_state = STATE_INIT;
    return 1;
}

static StateTransition_t status_transitions[] = {
    {STATE_INIT, HandleStatusState_Init},
    {STATE_START, HandleStatusState_Start},
    {STATE_DATA, HandleStatusState_Data},
    {STATE_END, HandleStatusState_End},
    {STATE_INIT, NULL}
};

/**
 * @brief 信号强度查询状态机处理函数
 */
static int HandleSignalState_Init(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    Signal_StateContext_t *signal_ctx = (Signal_StateContext_t *)user_data;
    signal_ctx->rx_len = 0;
    *signal_ctx->rssi = -100;
    ctx->current_state = STATE_START;
    return 1;
}

static int HandleSignalState_Start(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    Signal_StateContext_t *signal_ctx = (Signal_StateContext_t *)user_data;
    signal_ctx->rx_buffer[signal_ctx->rx_len++] = byte;
    if (signal_ctx->rx_len >= sizeof(signal_ctx->rx_buffer) - 1) {
        signal_ctx->rx_len = 0;
        ctx->current_state = STATE_INIT;
        return 0;
    }
    if (byte == '\n') {
        ctx->current_state = STATE_DATA;
    }
    return 1;
}

static int HandleSignalState_Data(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    Signal_StateContext_t *signal_ctx = (Signal_StateContext_t *)user_data;
    signal_ctx->rx_buffer[signal_ctx->rx_len++] = byte;
    if (signal_ctx->rx_len >= sizeof(signal_ctx->rx_buffer) - 1) {
        signal_ctx->rx_len = 0;
        ctx->current_state = STATE_INIT;
        return 0;
    }
    signal_ctx->rx_buffer[signal_ctx->rx_len] = '\0';
    if (strstr((char*)signal_ctx->rx_buffer, "+CWLAP:") != NULL) {
        char *ssid_start = strstr((char*)signal_ctx->rx_buffer, "\"" WIFI_SSID "\"");
        if (ssid_start) {
            char *rssi_start = strstr((const char*)signal_ctx->rx_buffer, ",");
            if (rssi_start) {
                rssi_start++;
                rssi_start = strstr(rssi_start, ",");
                if (rssi_start) {
                    rssi_start++;
                    int rssi_value;
                    if (sscanf(rssi_start, "%d", &rssi_value) == 1) {
                        *signal_ctx->rssi = (int8_t)rssi_value;
                        ctx->current_state = STATE_END;
                    }
                }
            }
        }
    }
    if (byte == '\n' && strstr((char*)signal_ctx->rx_buffer, "OK") != NULL) {
        ctx->current_state = STATE_END;
    }
    return 1;
}

static int HandleSignalState_End(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    ctx->current_state = STATE_INIT;
    return 1;
}

static StateTransition_t signal_transitions[] = {
    {STATE_INIT, HandleSignalState_Init},
    {STATE_START, HandleSignalState_Start},
    {STATE_DATA, HandleSignalState_Data},
    {STATE_END, HandleSignalState_End},
    {STATE_INIT, NULL}
};

/**
 * @brief 默认 WiFi 初始化回调
 */
static AT_Error_Code default_wifi_init(void *hw_context)
{
    Serial_Device_t *serial_dev = (Serial_Device_t *)hw_context;
    if (!serial_dev) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid serial device");
        return AT_ERR_SEND_FAILED;
    }

    if (Serial_Driver_Init(serial_dev) != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to initialize serial driver");
        return AT_ERR_SEND_FAILED;
    }

    Log_Message(LOG_LEVEL_INFO, "[WiFi] Initialized successfully");
    return AT_ERR_NONE;
}

/**
 * @brief 默认 WiFi 发送数据回调
 */
static AT_Error_Code default_wifi_send_data(void *hw_context, const uint8_t *data, uint16_t length)
{
    Serial_Device_t *serial_dev = (Serial_Device_t *)hw_context;
    if (!serial_dev || !data || length == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for send");
        return AT_ERR_SEND_FAILED;
    }

    if (Serial_Driver_SendData(serial_dev, data, length) != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send data");
        return AT_ERR_SEND_FAILED;
    }

    return AT_ERR_NONE;
}

/**
 * @brief 默认 WiFi 读取数据回调
 */
static int default_wifi_read_data(void *hw_context, uint8_t *buffer, uint16_t *length)
{
    Serial_Device_t *serial_dev = (Serial_Device_t *)hw_context;
    if (!serial_dev || !buffer || !length || *length == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for read");
        return 0;
    }

    if (RingBuffer_IsAvailable(&serial_dev->rx_buffer)) {
        RingBuffer_Read(&serial_dev->rx_buffer, buffer);
        *length = 1;
        return 1;
    }

    *length = 0;
    return 0;
}

/**
 * @brief 默认 WiFi 断开 TCP 连接回调
 */
static AT_Error_Code default_wifi_disconnect_tcp(void *hw_context)
{
    Serial_Device_t *serial_dev = (Serial_Device_t *)hw_context;
    if (!serial_dev) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid serial device");
        return AT_ERR_SEND_FAILED;
    }

    const char *cmd = "AT+CIPCLOSE\r\n";
    if (Serial_Driver_SendData(serial_dev, (uint8_t*)cmd, strlen(cmd)) != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send CIPCLOSE command");
        return AT_ERR_SEND_FAILED;
    }

    return AT_ERR_NONE;
}

/**
 * @brief 默认 WiFi 查询状态回调
 */
static AT_Error_Code default_wifi_query_status(void *hw_context, WiFi_Status_t *status)
{
    Serial_Device_t *serial_dev = (Serial_Device_t *)hw_context;
    if (!serial_dev || !status) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for query status");
        return AT_ERR_SEND_FAILED;
    }

    const char *cmd = "AT+CWJAP?\r\n";
    if (Serial_Driver_SendData(serial_dev, (uint8_t*)cmd, strlen(cmd)) != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send CWJAP? command");
        return AT_ERR_SEND_FAILED;
    }

    return AT_ERR_NONE;
}

/**
 * @brief 默认 WiFi 查询信号强度回调
 */
static AT_Error_Code default_wifi_query_signal(void *hw_context, int8_t *rssi)
{
    Serial_Device_t *serial_dev = (Serial_Device_t *)hw_context;
    if (!serial_dev || !rssi) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for query signal");
        return AT_ERR_SEND_FAILED;
    }

    const char *cmd = "AT+CWLAP\r\n";
    if (Serial_Driver_SendData(serial_dev, (uint8_t*)cmd, strlen(cmd)) != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send CWLAP command");
        return AT_ERR_SEND_FAILED;
    }

    return AT_ERR_NONE;
}

AT_Error_Code WiFi_SetOps(WiFi_Device_t *device, WiFi_Ops_t *ops, void *hw_context)
{
    if (!device || !ops || !ops->init || !ops->send_data || !ops->read_data ||
        !ops->disconnect_tcp || !ops->query_status || !ops->query_signal) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid device or ops");
        return AT_ERR_SEND_FAILED;
    }

    device->hw_context = hw_context;
    device->ops = ops;
    device->mutex = RTOS_GetOps()->SemaphoreCreate();
    if (!device->mutex) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to create mutex");
        return AT_ERR_SEND_FAILED;
    }

    Log_Message(LOG_LEVEL_INFO, "[WiFi] Set ops for device");
    return AT_ERR_NONE;
}

AT_Error_Code WiFi_Init(WiFi_Device_t *device)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !device || !device->ops || !device->hw_context) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid RTOS, device, or ops");
        return AT_ERR_SEND_FAILED;
    }

    if (!rtos_ops->SemaphoreTake(device->mutex, 0xFFFFFFFF)) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to take mutex");
        return AT_ERR_TIMEOUT;
    }

    AT_Error_Code status = device->ops->init(device->hw_context);
    rtos_ops->SemaphoreGive(device->mutex);
    return status;
}

AT_Error_Code WiFi_SendATCommand(WiFi_Device_t *device, const AT_Cmd_Config *cmd)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !device || !device->ops || !cmd || !cmd->at_cmd || !cmd->expected_resp) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for AT command");
        return AT_ERR_SEND_FAILED;
    }

    if (!rtos_ops->SemaphoreTake(device->mutex, 0xFFFFFFFF)) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to take mutex");
        return AT_ERR_TIMEOUT;
    }

    AT_StateContext_t at_ctx = {0};

    StateMachine_Init(&at_ctx.ctx, &at_ctx);
    at_ctx.current_cmd = cmd;

    uint8_t retry = 0;
    while (retry <= cmd->retries) {
        if (device->ops->send_data(device->hw_context, (uint8_t*)cmd->at_cmd, strlen(cmd->at_cmd)) != AT_ERR_NONE) {
            Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send command: %s", cmd->description);
            retry++;
            continue;
        }

        uint32_t start_tick = rtos_ops->GetTickCount();
        while ((rtos_ops->GetTickCount() - start_tick) < pdMS_TO_TICKS(cmd->timeout_ms)) {
            uint8_t byte;
            uint16_t len = 1;
            if (device->ops->read_data(device->hw_context, &byte, &len) && len > 0) {
                StateMachine_Process(&at_ctx.ctx, at_transitions, byte);
                if (at_ctx.ctx.current_state == STATE_END) {
                    rtos_ops->SemaphoreGive(device->mutex);
                    return AT_ERR_NONE;
                }
            }
            rtos_ops->Delay(10);
        }
        retry++;
        Log_Message(LOG_LEVEL_WARNING, "[WiFi] Timeout for command: %s, retry %d/%d", cmd->description, retry, cmd->retries);
    }

    Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to receive expected response for %s", cmd->description);
    rtos_ops->SemaphoreGive(device->mutex);
    return AT_ERR_TIMEOUT;
}

AT_Error_Code WiFi_ConnectTCPServer(WiFi_Device_t *device, const char *ip, const char *port)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !device || !device->ops || !ip || !port) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for TCP connection");
        return AT_ERR_SEND_FAILED;
    }

    if (!rtos_ops->SemaphoreTake(device->mutex, 0xFFFFFFFF)) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to take mutex");
        return AT_ERR_TIMEOUT;
    }

    char cmd_str[64];
    snprintf(cmd_str, sizeof(cmd_str), "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", ip, port);
    AT_Cmd_Config tcp_cmd = {
        .at_cmd = cmd_str,
        .expected_resp = "OK",
        .timeout_ms = 5000,
        .retries = 3,
        .description = "Connect to TCP server"
    };

    AT_Error_Code status = WiFi_SendATCommand(device, &tcp_cmd);
    rtos_ops->SemaphoreGive(device->mutex);

    if (status != AT_ERR_NONE) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to connect to TCP server");
        return AT_ERR_CONNECTION_FAILED;
    }

    Log_Message(LOG_LEVEL_INFO, "[WiFi] Connected to TCP server %s:%s", ip, port);
    return AT_ERR_NONE;
}

AT_Error_Code WiFi_DisconnectTCPServer(WiFi_Device_t *device)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !device || !device->ops) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for TCP disconnect");
        return AT_ERR_SEND_FAILED;
    }

    if (!rtos_ops->SemaphoreTake(device->mutex, 0xFFFFFFFF)) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to take mutex");
        return AT_ERR_TIMEOUT;
    }

    AT_Cmd_Config close_cmd = {
        .at_cmd = "AT+CIPCLOSE\r\n",
        .expected_resp = "OK",
        .timeout_ms = 2000,
        .retries = 2,
        .description = "Disconnect TCP server"
    };

    AT_Error_Code status = WiFi_SendATCommand(device, &close_cmd);
    rtos_ops->SemaphoreGive(device->mutex);

    if (status != AT_ERR_NONE) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to disconnect TCP server");
        return status;
    }

    Log_Message(LOG_LEVEL_INFO, "[WiFi] Disconnected from TCP server");
    return AT_ERR_NONE;
}

AT_Error_Code WiFi_QueryStatus(WiFi_Device_t *device, WiFi_Status_t *status)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !device || !device->ops || !status) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for query status");
        return AT_ERR_SEND_FAILED;
    }

    if (!rtos_ops->SemaphoreTake(device->mutex, 0xFFFFFFFF)) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to take mutex");
        return AT_ERR_TIMEOUT;
    }

    Status_StateContext_t status_ctx = {0};
    status_ctx.status = status;
    StateMachine_Init(&status_ctx.ctx, &status_ctx);

    AT_Cmd_Config status_cmd = {
        .at_cmd = "AT+CWJAP?\r\n",
        .expected_resp = "OK",
        .timeout_ms = 2000,
        .retries = 2,
        .description = "Query WiFi status"
    };

    if (device->ops->query_status(device->hw_context, status) != AT_ERR_NONE) {
        rtos_ops->SemaphoreGive(device->mutex);
        return AT_ERR_SEND_FAILED;
    }

    uint32_t start_tick = rtos_ops->GetTickCount();
    while ((rtos_ops->GetTickCount() - start_tick) < pdMS_TO_TICKS(status_cmd.timeout_ms)) {
        uint8_t byte;
        uint16_t len = 1;
        if (device->ops->read_data(device->hw_context, &byte, &len) && len > 0) {
            StateMachine_Process(&status_ctx.ctx, status_transitions, byte);
            if (status_ctx.ctx.current_state == STATE_END) {
                if (status->connected) {
                    AT_Cmd_Config ip_cmd = {
                        .at_cmd = "AT+CIPSTA?\r\n",
                        .expected_resp = "+CIPSTA:ip:",
                        .timeout_ms = 2000,
                        .retries = 2,
                        .description = "Query IP address"
                    };
                    AT_StateContext_t ip_ctx = {0};
                    StateMachine_Init(&ip_ctx.ctx, &ip_ctx);
                    ip_ctx.current_cmd = &ip_cmd;
                    if (device->ops->send_data(device->hw_context, (uint8_t*)ip_cmd.at_cmd, strlen(ip_cmd.at_cmd)) == AT_ERR_NONE) {
                        uint32_t ip_start_tick = rtos_ops->GetTickCount();
                        while ((rtos_ops->GetTickCount() - ip_start_tick) < pdMS_TO_TICKS(ip_cmd.timeout_ms)) {
                            if (device->ops->read_data(device->hw_context, &byte, &len) && len > 0) {
                                ip_ctx.rx_buffer[ip_ctx.rx_len++] = byte;
                                if (ip_ctx.rx_len >= sizeof(ip_ctx.rx_buffer) - 1) ip_ctx.rx_len = 0;
                                ip_ctx.rx_buffer[ip_ctx.rx_len] = '\0';
                                if (strstr((char*)ip_ctx.rx_buffer, "+CIPSTA:ip:\"")) {
                                    char *ip_start = strstr((char*)ip_ctx.rx_buffer, "\"");
                                    if (ip_start) {
                                        ip_start++;
                                        char *ip_end = strstr(ip_start, "\"");
                                        if (ip_end) {
                                            size_t ip_len = ip_end - ip_start;
                                            if (ip_len < sizeof(status->ip_addr)) {
                                                strncpy(status->ip_addr, ip_start, ip_len);
                                                status->ip_addr[ip_len] = '\0';
                                            }
                                        }
                                    }
                                    break;
                                }
                            }
                            rtos_ops->Delay(10);
                        }
                    }
                }
                rtos_ops->SemaphoreGive(device->mutex);
                Log_Message(LOG_LEVEL_INFO, "[WiFi] WiFi status: %s, SSID: %s, IP: %s",
                            status->connected ? "Connected" : "Not connected",
                            status->ssid, status->ip_addr);
                return AT_ERR_NONE;
            }
        }
        rtos_ops->Delay(10);
    }

    Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to query WiFi status");
    rtos_ops->SemaphoreGive(device->mutex);
    return AT_ERR_TIMEOUT;
}

AT_Error_Code WiFi_QuerySignalStrength(WiFi_Device_t *device, int8_t *rssi)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !device || !device->ops || !rssi) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for query signal");
        return AT_ERR_SEND_FAILED;
    }

    if (!rtos_ops->SemaphoreTake(device->mutex, 0xFFFFFFFF)) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to take mutex");
        return AT_ERR_TIMEOUT;
    }

    Signal_StateContext_t signal_ctx = {0};
    signal_ctx.rssi = rssi;
    StateMachine_Init(&signal_ctx.ctx, &signal_ctx);

    AT_Cmd_Config signal_cmd = {
        .at_cmd = "AT+CWLAP\r\n",
        .expected_resp = "OK",
        .timeout_ms = 10000,
        .retries = 2,
        .description = "Query signal strength"
    };

    if (device->ops->query_signal(device->hw_context, rssi) != AT_ERR_NONE) {
        rtos_ops->SemaphoreGive(device->mutex);
        return AT_ERR_SEND_FAILED;
    }

    uint32_t start_tick = rtos_ops->GetTickCount();
    while ((rtos_ops->GetTickCount() - start_tick) < pdMS_TO_TICKS(signal_cmd.timeout_ms)) {
        uint8_t byte;
        uint16_t len = 1;
        if (device->ops->read_data(device->hw_context, &byte, &len) && len > 0) {
            StateMachine_Process(&signal_ctx.ctx, signal_transitions, byte);
            if (signal_ctx.ctx.current_state == STATE_END) {
                rtos_ops->SemaphoreGive(device->mutex);
                Log_Message(LOG_LEVEL_INFO, "[WiFi] Signal strength: %d dBm", *rssi);
                return AT_ERR_NONE;
            }
        }
        rtos_ops->Delay(10);
    }

    Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to query signal strength");
    rtos_ops->SemaphoreGive(device->mutex);
    return AT_ERR_TIMEOUT;
}

AT_Error_Code WiFi_SendTCPData(WiFi_Device_t *device, const uint8_t *data, uint16_t length)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !device || !device->ops || !data || length == 0 || length > TCP_BUFFER_SIZE) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for TCP send");
        return AT_ERR_SEND_FAILED;
    }

    if (!rtos_ops->SemaphoreTake(device->mutex, 0xFFFFFFFF)) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to take mutex");
        return AT_ERR_TIMEOUT;
    }

    char cmd_str[32];
    snprintf(cmd_str, sizeof(cmd_str), "AT+CIPSEND=%d\r\n", length);
    AT_Cmd_Config send_cmd = {
        .at_cmd = cmd_str,
        .expected_resp = ">",
        .timeout_ms = 2000,
        .retries = 2,
        .description = "Prepare to send TCP data"
    };

    AT_Error_Code status = WiFi_SendATCommand(device, &send_cmd);
    if (status != AT_ERR_NONE) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to prepare TCP send");
        rtos_ops->SemaphoreGive(device->mutex);
        return status;
    }

    if (device->ops->send_data(device->hw_context, data, length) != AT_ERR_NONE) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send TCP data");
        rtos_ops->SemaphoreGive(device->mutex);
        return AT_ERR_SEND_FAILED;
    }

    AT_Cmd_Config confirm_cmd = {
        .at_cmd = "",
        .expected_resp = "SEND OK",
        .timeout_ms = 2000,
        .retries = 2,
        .description = "Confirm TCP data send"
    };

    status = WiFi_SendATCommand(device, &confirm_cmd);
    rtos_ops->SemaphoreGive(device->mutex);

    if (status != AT_ERR_NONE) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to confirm TCP send");
        return status;
    }

    Log_Message(LOG_LEVEL_INFO, "[WiFi] Sent %d bytes of TCP data", length);
    return AT_ERR_NONE;
}

AT_Error_Code WiFi_ReceiveTCPData(WiFi_Device_t *device, uint8_t *buffer, uint16_t *length, uint32_t timeout_ms)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !device || !device->ops || !buffer || !length || *length == 0 || *length > TCP_BUFFER_SIZE) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for TCP receive");
        return AT_ERR_SEND_FAILED;
    }

    if (!rtos_ops->SemaphoreTake(device->mutex, 0xFFFFFFFF)) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to take mutex");
        return AT_ERR_TIMEOUT;
    }

    TCP_StateContext_t tcp_ctx = {0};
    StateMachine_Init(&tcp_ctx.ctx, &tcp_ctx);
    tcp_ctx.expected_length = *length;

    uint32_t start_tick = rtos_ops->GetTickCount();
    while ((rtos_ops->GetTickCount() - start_tick) < pdMS_TO_TICKS(timeout_ms)) {
        uint8_t byte;
        uint16_t len = 1;
        if (device->ops->read_data(device->hw_context, &byte, &len) && len > 0) {
            StateMachine_Process(&tcp_ctx.ctx, tcp_transitions, byte);
            if (tcp_ctx.ctx.current_state == STATE_END) {
                *length = tcp_ctx.rx_len;
                memcpy(buffer, tcp_ctx.rx_buffer, tcp_ctx.rx_len);
                Log_Message(LOG_LEVEL_INFO, "[WiFi] Received %d bytes of TCP data", tcp_ctx.rx_len);
                rtos_ops->SemaphoreGive(device->mutex);
                return AT_ERR_NONE;
            }
        }
        rtos_ops->Delay(10);
    }

    Log_Message(LOG_LEVEL_WARNING, "[WiFi] Timeout receiving TCP data");
    *length = 0;
    rtos_ops->SemaphoreGive(device->mutex);
    return AT_ERR_TIMEOUT;
}

/**
 * @brief 默认 WiFi 操作接口
 */
WiFi_Ops_t default_wifi_ops = {
    .init = default_wifi_init,
    .send_data = default_wifi_send_data,
    .read_data = default_wifi_read_data,
    .disconnect_tcp = default_wifi_disconnect_tcp,
    .query_status = default_wifi_query_status,
    .query_signal = default_wifi_query_signal
};

/*
 * 示例用法：
 * 1. 设置 RTOS 抽象层
 * RTOS_SetOps(&FreeRTOS_Ops);
 *
 * 2. 初始化 WiFi 设备
 * WiFi_Device_t wifi_device = {0};
 * Serial_Device_t wifi_serial = {...};
 * WiFi_Ops_t wifi_ops = {
 *     .init = default_wifi_init,
 *     .send_data = default_wifi_send_data,
 *     .read_data = default_wifi_read_data,
 *     .disconnect_tcp = default_wifi_disconnect_tcp,
 *     .query_status = default_wifi_query_status,
 *     .query_signal = default_wifi_query_signal
 * };
 * WiFi_SetOps(&wifi_device, &wifi_ops, &wifi_serial);
 * WiFi_Init(&wifi_device);
 *
 * 3. TCP 数据传输
 * WiFi_ConnectTCPServer(&wifi_device, "192.168.2.100", "5000");
 * uint8_t data[] = "Test Data";
 * WiFi_SendTCPData(&wifi_device, data, strlen(data));
 * uint8_t rx_buffer[256];
 * uint16_t rx_len = 256;
 * WiFi_ReceiveTCPData(&wifi_device, rx_buffer, &rx_len, 5000);
 * WiFi_DisconnectTCPServer(&wifi_device);
 *
 * 4. 查询状态
 * WiFi_Status_t status = {0};
 * WiFi_QueryStatus(&wifi_device, &status);
 * int8_t rssi;
 * WiFi_QuerySignalStrength(&wifi_device, &rssi);
 */
