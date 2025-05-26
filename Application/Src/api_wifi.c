/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-01 20:50:17
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-26 20:52:45
 * @FilePath: \Demo\Application\Src\api_wifi.c
 * @Description: WiFi 和 BLE 模块驱动实现
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "api_wifi.h"
#include "serial_driver.h"
#include "log_system.h"
#include <string.h>
#include <stdio.h>
#include "pch.h"

/**
 * @brief       ATK_MB026硬件初始化
 * @param       无
 * @retval      无
 */
void atk_mb026_hw_init(void)
{
    if (Common_GPIO_Init(GPIOA, GPIO_Pin_4, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0) != COMMON_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[PCF8574] Failed to init GPIO");
        return 1;
    }
}
/* IO操作 */
#define ATK_MB026_RST(x)                  do{ x ?                                                                                     \
                                                GPIO_SetBits(GPIOA, GPIO_Pin_4) :  \
                                                GPIO_ResetBits(GPIOA, GPIO_Pin_4); \
                                            }while(0)
/**
 * @brief       ATK_MB026硬件复位
 * @param       无
 * @retval      无
 */
void atk_mb026_hw_reset(void)
{
    ATK_MB026_RST(0);
    delay_ms(100);
    ATK_MB026_RST(1);
    delay_ms(500);
}

#define AT_RX_BUFFER_SIZE 512
typedef struct {
    uint8_t buffer[AT_RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint8_t overflow;
} AT_RxBuffer_t;

static AT_RxBuffer_t wifi_rx_buffer = {0};
static AT_RxBuffer_t ble_rx_buffer = {0};

/* 中断处理函数 */
void ESP32_Serial_IRQHandler(uint8_t byte) {
    // 写入WiFi缓冲区
    uint16_t next_head = (wifi_rx_buffer.head + 1) % AT_RX_BUFFER_SIZE;
    if (next_head != wifi_rx_buffer.tail) {
        wifi_rx_buffer.buffer[wifi_rx_buffer.head] = byte;
        wifi_rx_buffer.head = next_head;
    } else {
        wifi_rx_buffer.overflow = 1;
    }
    
    // 写入BLE缓冲区
    next_head = (ble_rx_buffer.head + 1) % AT_RX_BUFFER_SIZE;
    if (next_head != ble_rx_buffer.tail) {
        ble_rx_buffer.buffer[ble_rx_buffer.head] = byte;
        ble_rx_buffer.head = next_head;
    } else {
        ble_rx_buffer.overflow = 1;
    }
}
/* 辅助函数：检查缓冲区中是否有预期响应 */
static uint8_t AT_CheckResponseInBuffer(AT_RxBuffer_t *rx_buf, const char *expected) {
    uint16_t tail = rx_buf->tail;
    uint16_t head = rx_buf->head;
    uint16_t len = strlen(expected);
    
    if (tail <= head) {
        if ((head - tail) >= len) {
            return strstr((char*)&rx_buf->buffer[tail], expected) != NULL;
        }
    } else {
        uint16_t first_part = AT_RX_BUFFER_SIZE - tail;
        if (first_part >= len) {
            return strstr((char*)&rx_buf->buffer[tail], expected) != NULL;
        } else {
            char temp_buf[AT_RX_BUFFER_SIZE];
            memcpy(temp_buf, &rx_buf->buffer[tail], first_part);
            memcpy(&temp_buf[first_part], rx_buf->buffer, len - first_part);
            return strstr(temp_buf, expected) != NULL;
        }
    }
    return 0;
}

/* 修改 WiFi_SendATCommand 函数 */
AT_Error_Code WiFi_SendATCommand(const AT_Cmd_Config *cmd) {
    WiFi_Device_t *wifi_dev = &WiFi_Device;
    if (!g_rtos_ops || !wifi_dev->serial_dev || !cmd || !cmd->at_cmd || !cmd->expected_resp) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for AT command");
        return AT_ERR_SEND_FAILED;
    }

    if (!g_rtos_ops->SemaphoreTake(wifi_dev->mutex, 0xFFFFFFFF)) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to take mutex");
        return AT_ERR_TIMEOUT;
    }

    uint8_t retry = 0;
    while (retry <= cmd->retries) {
        wifi_rx_buffer.head = wifi_rx_buffer.tail = 0; // 清空缓冲区
        
        if (Serial_Driver_SendData(wifi_dev->serial_dev, (uint8_t*)cmd->at_cmd, strlen(cmd->at_cmd)) != SERIAL_OK) {
            Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send command: %s", cmd->description);
            retry++;
            continue;
        }

        uint32_t start_tick = g_rtos_ops->GetTickCount();
        while ((g_rtos_ops->GetTickCount() - start_tick) < pdMS_TO_TICKS(cmd->timeout_ms)) {
            if (AT_CheckResponseInBuffer(&wifi_rx_buffer, cmd->expected_resp)) {
                Log_Message(LOG_LEVEL_INFO, "[WiFi] Received expected response for %s", cmd->description);
                g_rtos_ops->SemaphoreGive(wifi_dev->mutex);
                return AT_ERR_NONE;
            }
            g_rtos_ops->Delay(10);
        }
        retry++;
        Log_Message(LOG_LEVEL_WARNING, "[WiFi] Timeout for command: %s, retry %d/%d", cmd->description, retry, cmd->retries);
    }

    g_rtos_ops->SemaphoreGive(wifi_dev->mutex);
    return AT_ERR_TIMEOUT;
}

/* 类似地修改 BLE_SendATCommand 函数 */
AT_Error_Code BLE_SendATCommand(const AT_Cmd_Config *cmd) {
    BLE_Device_t *ble_dev = &BLE_Device;
    if (!g_rtos_ops || !ble_dev->serial_dev || !cmd || !cmd->at_cmd || !cmd->expected_resp) {
        Log_Message(LOG_LEVEL_ERROR, "[BLE] Invalid parameters for AT command");
        return AT_ERR_SEND_FAILED;
    }

    if (!g_rtos_ops->SemaphoreTake(ble_dev->mutex, 0xFFFFFFFF)) {
        Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to take mutex");
        return AT_ERR_TIMEOUT;
    }

    uint8_t retry = 0;
    while (retry <= cmd->retries) {
        ble_rx_buffer.head = ble_rx_buffer.tail = 0; // 清空缓冲区
        
        if (Serial_Driver_SendData(ble_dev->serial_dev, (uint8_t*)cmd->at_cmd, strlen(cmd->at_cmd)) != SERIAL_OK) {
            Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to send command: %s", cmd->description);
            retry++;
            continue;
        }

        uint32_t start_tick = g_rtos_ops->GetTickCount();
        while ((g_rtos_ops->GetTickCount() - start_tick) < pdMS_TO_TICKS(cmd->timeout_ms)) {
            if (AT_CheckResponseInBuffer(&ble_rx_buffer, cmd->expected_resp)) {
                Log_Message(LOG_LEVEL_INFO, "[BLE] Received expected response for %s", cmd->description);
                g_rtos_ops->SemaphoreGive(ble_dev->mutex);
                return AT_ERR_NONE;
            }
            g_rtos_ops->Delay(10);
        }
        retry++;
        Log_Message(LOG_LEVEL_WARNING, "[BLE] Timeout for command: %s, retry %d/%d", cmd->description, retry, cmd->retries);
    }

    g_rtos_ops->SemaphoreGive(ble_dev->mutex);
    return AT_ERR_TIMEOUT;
}

/**
 * @brief 发送 WiFi AT 指令
 */
// AT_Error_Code WiFi_SendATCommand(const AT_Cmd_Config *cmd)
// {
//     WiFi_Device_t *wifi_dev = &WiFi_Device;
//     if (!g_rtos_ops || !wifi_dev->serial_dev || !cmd || !cmd->at_cmd || !cmd->expected_resp)
//     {
//         Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for AT command");
//         return AT_ERR_SEND_FAILED;
//     }

//     if (!g_rtos_ops->SemaphoreTake(wifi_dev->mutex, 0xFFFFFFFF))
//     {
//         Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to take mutex");
//         return AT_ERR_TIMEOUT;
//     }

//     uint8_t retry = 0;
//     while (retry <= cmd->retries)
//     {
//         if (Serial_Driver_SendData(wifi_dev->serial_dev, (uint8_t*)cmd->at_cmd, strlen(cmd->at_cmd)) != SERIAL_OK)
//         {
//             Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send command: %s", cmd->description);
//             retry++;
//             continue;
//         }

//         uint8_t rx_buffer[TCP_BUFFER_SIZE] = {0};
//         uint16_t rx_len = 0;
//         uint32_t start_tick = g_rtos_ops->GetTickCount();
//         while ((g_rtos_ops->GetTickCount() - start_tick) < pdMS_TO_TICKS(cmd->timeout_ms))
//         {
//             uint8_t byte;
//             if (RingBuffer_IsAvailable(&wifi_dev->serial_dev->rx_buffer))
//             {
//                 RingBuffer_Read(&wifi_dev->serial_dev->rx_buffer, &byte);
//                 if (rx_len < TCP_BUFFER_SIZE - 1)
//                 {
//                     rx_buffer[rx_len++] = byte;
//                     rx_buffer[rx_len] = '\0';
//                 }
//                 if (strstr((char*)rx_buffer, cmd->expected_resp))
//                 {
//                     Log_Message(LOG_LEVEL_INFO, "[WiFi] Received expected response for %s", cmd->description);
//                     g_rtos_ops->SemaphoreGive(wifi_dev->mutex);
//                     return AT_ERR_NONE;
//                 }
//             }
//             g_rtos_ops->Delay(10);
//         }
//         retry++;
//         Log_Message(LOG_LEVEL_WARNING, "[WiFi] Timeout for command: %s, retry %d/%d", cmd->description, retry, cmd->retries);
//     }

// //    Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to receive expected response for %s", cmd->description);
//     g_rtos_ops->SemaphoreGive(wifi_dev->mutex);
//     return AT_ERR_TIMEOUT;
// }

/**
 * @brief 发送 WiFi TCP 数据
 */
AT_Error_Code WiFi_SendTCPData(const uint8_t *data, uint16_t length)
{
    WiFi_Device_t *wifi_dev = &WiFi_Device;
    if (!g_rtos_ops || !wifi_dev->serial_dev || !data || length == 0 || length > TCP_BUFFER_SIZE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for TCP send");
        return AT_ERR_SEND_FAILED;
    }

    if (!g_rtos_ops->SemaphoreTake(wifi_dev->mutex, 0xFFFFFFFF))
    {
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

    AT_Error_Code status = WiFi_SendATCommand(&send_cmd);
    if (status != AT_ERR_NONE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to prepare TCP send");
        g_rtos_ops->SemaphoreGive(wifi_dev->mutex);
        return status;
    }

    if (Serial_Driver_SendData(wifi_dev->serial_dev, data, length) != SERIAL_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send TCP data");
        g_rtos_ops->SemaphoreGive(wifi_dev->mutex);
        return AT_ERR_SEND_FAILED;
    }

    AT_Cmd_Config confirm_cmd = {
        .at_cmd = "",
        .expected_resp = "SEND OK",
        .timeout_ms = 2000,
        .retries = 2,
        .description = "Confirm TCP data send"
    };

    status = WiFi_SendATCommand(&confirm_cmd);
    g_rtos_ops->SemaphoreGive(wifi_dev->mutex);

    if (status != AT_ERR_NONE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to confirm TCP send");
        return status;
    }

    Log_Message(LOG_LEVEL_INFO, "[WiFi] Sent %d bytes of TCP data", length);
    return AT_ERR_NONE;
}

/**
 * @brief 接收 WiFi TCP 数据
 */
AT_Error_Code WiFi_ReceiveTCPData(uint8_t *buffer, uint16_t *length, uint32_t timeout_ms)
{
    WiFi_Device_t *wifi_dev = &WiFi_Device;
    if (!g_rtos_ops || !wifi_dev->serial_dev || !buffer || !length || *length == 0 || *length > TCP_BUFFER_SIZE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for TCP receive");
        return AT_ERR_SEND_FAILED;
    }

    if (!g_rtos_ops->SemaphoreTake(wifi_dev->mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to take mutex");
        return AT_ERR_TIMEOUT;
    }

    uint8_t rx_buffer[TCP_BUFFER_SIZE] = {0};
    uint16_t rx_len = 0;
    uint32_t start_tick = g_rtos_ops->GetTickCount();
    while ((g_rtos_ops->GetTickCount() - start_tick) < pdMS_TO_TICKS(timeout_ms))
    {
        uint8_t byte;
        if (RingBuffer_IsAvailable(&wifi_dev->serial_dev->rx_buffer))
        {
            RingBuffer_Read(&wifi_dev->serial_dev->rx_buffer, &byte);
            if (rx_len < *length && rx_len < TCP_BUFFER_SIZE - 1)
            {
                rx_buffer[rx_len++] = byte;
            }
            if (rx_len >= *length)
            {
                *length = rx_len;
                memcpy(buffer, rx_buffer, rx_len);
                Log_Message(LOG_LEVEL_INFO, "[WiFi] Received %d bytes of TCP data", rx_len);
                g_rtos_ops->SemaphoreGive(wifi_dev->mutex);
                return AT_ERR_NONE;
            }
        }
        g_rtos_ops->Delay(10);
    }

    Log_Message(LOG_LEVEL_WARNING, "[WiFi] Timeout receiving TCP data");
    *length = 0;
    g_rtos_ops->SemaphoreGive(wifi_dev->mutex);
    return AT_ERR_TIMEOUT;
}

/**
 * @brief 发送 BLE AT 指令
 */
// AT_Error_Code BLE_SendATCommand(const AT_Cmd_Config *cmd)
// {
//     BLE_Device_t *ble_dev = &BLE_Device;
//     if (!g_rtos_ops || !ble_dev->serial_dev || !cmd || !cmd->at_cmd || !cmd->expected_resp)
//     {
//         Log_Message(LOG_LEVEL_ERROR, "[BLE] Invalid parameters for AT command");
//         return AT_ERR_SEND_FAILED;
//     }

//     if (!g_rtos_ops->SemaphoreTake(ble_dev->mutex, 0xFFFFFFFF))
//     {
//         Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to take mutex");
//         return AT_ERR_TIMEOUT;
//     }

//     uint8_t retry = 0;
//     while (retry <= cmd->retries)
//     {
//         if (Serial_Driver_SendData(ble_dev->serial_dev, (uint8_t*)cmd->at_cmd, strlen(cmd->at_cmd)) != SERIAL_OK)
//         {
//             Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to send command: %s", cmd->description);
//             retry++;
//             continue;
//         }

//         uint8_t rx_buffer[TCP_BUFFER_SIZE] = {0};
//         uint16_t rx_len = 0;
//         uint32_t start_tick = g_rtos_ops->GetTickCount();
//         while ((g_rtos_ops->GetTickCount() - start_tick) < pdMS_TO_TICKS(cmd->timeout_ms))
//         {
//             uint8_t byte;
//             if (RingBuffer_IsAvailable(&ble_dev->serial_dev->rx_buffer))
//             {
//                 RingBuffer_Read(&ble_dev->serial_dev->rx_buffer, &byte);
//                 if (rx_len < TCP_BUFFER_SIZE - 1)
//                 {
//                     rx_buffer[rx_len++] = byte;
//                     rx_buffer[rx_len] = '\0';
//                 }
//                 if (strstr((char*)rx_buffer, cmd->expected_resp))
//                 {
//                     Log_Message(LOG_LEVEL_INFO, "[BLE] Received expected response for %s", cmd->description);
//                     g_rtos_ops->SemaphoreGive(ble_dev->mutex);
//                     return AT_ERR_NONE;
//                 }
//             }
//             g_rtos_ops->Delay(10);
//         }
//         retry++;
//         Log_Message(LOG_LEVEL_WARNING, "[BLE] Timeout for command: %s, retry %d/%d", cmd->description, retry, cmd->retries);
//     }

//     Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to receive expected response for %s", cmd->description);
//     g_rtos_ops->SemaphoreGive(ble_dev->mutex);
//     return AT_ERR_TIMEOUT;
// }

/**
 * @brief 发送 BLE 数据
 */
AT_Error_Code BLE_SendData(const uint8_t *data, uint16_t length)
{
    BLE_Device_t *ble_dev = &BLE_Device;
    if (!g_rtos_ops || !ble_dev->serial_dev || !data || length == 0 || length > TCP_BUFFER_SIZE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[BLE] Invalid parameters for data send");
        return AT_ERR_SEND_FAILED;
    }

    if (!g_rtos_ops->SemaphoreTake(ble_dev->mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to take mutex");
        return AT_ERR_TIMEOUT;
    }

    char cmd_str[32];
    snprintf(cmd_str, sizeof(cmd_str), "AT+BLESEND=%d\r\n", length);
    AT_Cmd_Config send_cmd = {
        .at_cmd = cmd_str,
        .expected_resp = ">",
        .timeout_ms = 2000,
        .retries = 2,
        .description = "Prepare to send BLE data"
    };

    AT_Error_Code status = BLE_SendATCommand(&send_cmd);
    if (status != AT_ERR_NONE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to prepare data send");
        g_rtos_ops->SemaphoreGive(ble_dev->mutex);
        return status;
    }

    if (Serial_Driver_SendData(ble_dev->serial_dev, data, length) != SERIAL_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to send data");
        g_rtos_ops->SemaphoreGive(ble_dev->mutex);
        return AT_ERR_SEND_FAILED;
    }

    AT_Cmd_Config confirm_cmd = {
        .at_cmd = "",
        .expected_resp = "SEND OK",
        .timeout_ms = 2000,
        .retries = 2,
        .description = "Confirm BLE data send"
    };

    status = BLE_SendATCommand(&confirm_cmd);
    g_rtos_ops->SemaphoreGive(ble_dev->mutex);

    if (status != AT_ERR_NONE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to confirm data send");
        return status;
    }

    Log_Message(LOG_LEVEL_INFO, "[BLE] Sent %d bytes of data", length);
    return AT_ERR_NONE;
}

/**
 * @brief 接收 BLE 数据
 */
AT_Error_Code BLE_ReceiveData(uint8_t *buffer, uint16_t *length, uint32_t timeout_ms)
{
    BLE_Device_t *ble_dev = &BLE_Device;
    if (!g_rtos_ops || !ble_dev->serial_dev || !buffer || !length || *length == 0 || length > TCP_BUFFER_SIZE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[BLE] Invalid parameters for data receive");
        return AT_ERR_SEND_FAILED;
    }

    if (!g_rtos_ops->SemaphoreTake(ble_dev->mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to take mutex");
        return AT_ERR_TIMEOUT;
    }

    uint8_t rx_buffer[TCP_BUFFER_SIZE] = {0};
    uint16_t rx_len = 0;
    uint32_t start_tick = g_rtos_ops->GetTickCount();
    while ((g_rtos_ops->GetTickCount() - start_tick) < pdMS_TO_TICKS(timeout_ms))
    {
        uint8_t byte;
        if (RingBuffer_IsAvailable(&ble_dev->serial_dev->rx_buffer))
        {
            RingBuffer_Read(&ble_dev->serial_dev->rx_buffer, &byte);
            if (rx_len < *length && rx_len < TCP_BUFFER_SIZE - 1)
            {
                rx_buffer[rx_len++] = byte;
            }
            if (rx_len >= *length)
            {
                *length = rx_len;
                memcpy(buffer, rx_buffer, rx_len);
                Log_Message(LOG_LEVEL_INFO, "[BLE] Received %d bytes of data", rx_len);
                g_rtos_ops->SemaphoreGive(ble_dev->mutex);
                return AT_ERR_NONE;
            }
        }
        g_rtos_ops->Delay(10);
    }

    Log_Message(LOG_LEVEL_WARNING, "[BLE] Timeout receiving data");
    *length = 0;
    g_rtos_ops->SemaphoreGive(ble_dev->mutex);
    return AT_ERR_TIMEOUT;
}
