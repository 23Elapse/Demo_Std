/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-01 20:50:17
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-26 19:00:39
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
/**
 * @brief 发送 WiFi AT 指令
 */
AT_Error_Code WiFi_SendATCommand(const AT_Cmd_Config *cmd)
{
    WiFi_Device_t *wifi_dev = &WiFi_Device;
    if (!g_rtos_ops || !wifi_dev->serial_dev || !cmd || !cmd->at_cmd || !cmd->expected_resp)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for AT command");
        return AT_ERR_SEND_FAILED;
    }

    if (!g_rtos_ops->SemaphoreTake(wifi_dev->mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to take mutex");
        return AT_ERR_TIMEOUT;
    }

    uint8_t retry = 0;
    while (retry <= cmd->retries)
    {
        if (Serial_Driver_SendData(wifi_dev->serial_dev, (uint8_t*)cmd->at_cmd, strlen(cmd->at_cmd)) != SERIAL_OK)
        {
            Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send command: %s", cmd->description);
            retry++;
            continue;
        }

        uint8_t rx_buffer[TCP_BUFFER_SIZE] = {0};
        uint16_t rx_len = 0;
        uint32_t start_tick = g_rtos_ops->GetTickCount();
        while ((g_rtos_ops->GetTickCount() - start_tick) < pdMS_TO_TICKS(cmd->timeout_ms))
        {
            uint8_t byte;
            if (RingBuffer_IsAvailable(&wifi_dev->serial_dev->rx_buffer))
            {
                RingBuffer_Read(&wifi_dev->serial_dev->rx_buffer, &byte);
                if (rx_len < TCP_BUFFER_SIZE - 1)
                {
                    rx_buffer[rx_len++] = byte;
                    rx_buffer[rx_len] = '\0';
                }
                if (strstr((char*)rx_buffer, cmd->expected_resp))
                {
                    Log_Message(LOG_LEVEL_INFO, "[WiFi] Received expected response for %s", cmd->description);
                    g_rtos_ops->SemaphoreGive(wifi_dev->mutex);
                    return AT_ERR_NONE;
                }
            }
            g_rtos_ops->Delay(10);
        }
        retry++;
        Log_Message(LOG_LEVEL_WARNING, "[WiFi] Timeout for command: %s, retry %d/%d", cmd->description, retry, cmd->retries);
    }

//    Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to receive expected response for %s", cmd->description);
    g_rtos_ops->SemaphoreGive(wifi_dev->mutex);
    return AT_ERR_TIMEOUT;
}

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
AT_Error_Code BLE_SendATCommand(const AT_Cmd_Config *cmd)
{
    BLE_Device_t *ble_dev = &BLE_Device;
    if (!g_rtos_ops || !ble_dev->serial_dev || !cmd || !cmd->at_cmd || !cmd->expected_resp)
    {
        Log_Message(LOG_LEVEL_ERROR, "[BLE] Invalid parameters for AT command");
        return AT_ERR_SEND_FAILED;
    }

    if (!g_rtos_ops->SemaphoreTake(ble_dev->mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to take mutex");
        return AT_ERR_TIMEOUT;
    }

    uint8_t retry = 0;
    while (retry <= cmd->retries)
    {
        if (Serial_Driver_SendData(ble_dev->serial_dev, (uint8_t*)cmd->at_cmd, strlen(cmd->at_cmd)) != SERIAL_OK)
        {
            Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to send command: %s", cmd->description);
            retry++;
            continue;
        }

        uint8_t rx_buffer[TCP_BUFFER_SIZE] = {0};
        uint16_t rx_len = 0;
        uint32_t start_tick = g_rtos_ops->GetTickCount();
        while ((g_rtos_ops->GetTickCount() - start_tick) < pdMS_TO_TICKS(cmd->timeout_ms))
        {
            uint8_t byte;
            if (RingBuffer_IsAvailable(&ble_dev->serial_dev->rx_buffer))
            {
                RingBuffer_Read(&ble_dev->serial_dev->rx_buffer, &byte);
                if (rx_len < TCP_BUFFER_SIZE - 1)
                {
                    rx_buffer[rx_len++] = byte;
                    rx_buffer[rx_len] = '\0';
                }
                if (strstr((char*)rx_buffer, cmd->expected_resp))
                {
                    Log_Message(LOG_LEVEL_INFO, "[BLE] Received expected response for %s", cmd->description);
                    g_rtos_ops->SemaphoreGive(ble_dev->mutex);
                    return AT_ERR_NONE;
                }
            }
            g_rtos_ops->Delay(10);
        }
        retry++;
        Log_Message(LOG_LEVEL_WARNING, "[BLE] Timeout for command: %s, retry %d/%d", cmd->description, retry, cmd->retries);
    }

    Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to receive expected response for %s", cmd->description);
    g_rtos_ops->SemaphoreGive(ble_dev->mutex);
    return AT_ERR_TIMEOUT;
}

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
