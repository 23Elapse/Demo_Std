/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-01 20:50:17
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-04 21:19:32
 * @FilePath: \Demo\Application\api_wifi.c
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

void ESP32_Device_HwInit(void) // 替换 atk_mb026_hw_init
{
    // 使用 ESP32_Device 中定义的引脚信息
    if (Common_GPIO_Init(ESP32_Device.reset_port, ESP32_Device.reset_pin, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0) != COMMON_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[ESP32] Failed to init Reset GPIO");
    }
}
void ESP32_Device_HwReset(void) // 替换 atk_mb026_hw_reset
{
    // 使用 ESP32_Device 中定义的引脚信息
    GPIO_ResetBits(ESP32_Device.reset_port, ESP32_Device.reset_pin);
    delay_ms(100); // 假设 delay_ms 是一个可用的HAL层短延时函数 (忙等待)
    GPIO_SetBits(ESP32_Device.reset_port, ESP32_Device.reset_pin);
    delay_ms(500); // 假设 delay_ms 是一个可用的HAL层短延时函数 (忙等待)
}

/**
 * @brief 发送 WiFi AT 指令
 */
AT_Error_Code WiFi_SendATCommand(const AT_Cmd_Config *cmd)
{
    // WiFi_Device_t *wifi_dev = &ESP32_Device;
    if (!g_rtos_ops || !ESP32_Device.serial_dev || !cmd || !cmd->at_cmd || !cmd->expected_resp)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for AT command");
        return AT_ERR_SEND_FAILED;
    }

    if (!g_rtos_ops->SemaphoreTake(ESP32_Device.mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to take mutex");
        return AT_ERR_TIMEOUT;
    }
    // 清空硬件接收缓冲区中可能残留的数据
    uint8_t dummy_byte;
    while (RingBuffer_IsAvailable(&ESP32_Device.serial_dev->rx_buffer)) {
        RingBuffer_Read(&ESP32_Device.serial_dev->rx_buffer, &dummy_byte);
    }
    // 同时确保本地累积缓冲区也被清空 (虽然它在栈上每次都会重新初始化，但显式清零更清晰)
    uint8_t rx_buffer[TCP_BUFFER_SIZE]; // 注意: TCP_BUFFER_SIZE 可能也需要调整，确保足够大
    memset(rx_buffer, 0, sizeof(rx_buffer));
    uint16_t rx_len = 0;

    uint8_t retry = 0;
    while (retry <= cmd->retries)
    {
        memset(rx_buffer, 0, sizeof(rx_buffer));
        rx_len = 0;
        if (Serial_Driver_SendData(ESP32_Device.serial_dev, (uint8_t*)cmd->at_cmd, strlen(cmd->at_cmd)) != SERIAL_OK)
        {
            Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send command: %s", cmd->description);
            retry++;
            g_rtos_ops->Delay(100); // 发送失败后稍作延时
            continue;
        }

        // uint8_t rx_buffer[TCP_BUFFER_SIZE] = {0};
        // uint16_t rx_len = 0;
        uint32_t start_tick = g_rtos_ops->GetTickCount();
        while ((g_rtos_ops->GetTickCount() - start_tick) < pdMS_TO_TICKS(cmd->timeout_ms))
        {
            uint8_t byte;
            if (RingBuffer_IsAvailable(&ESP32_Device.serial_dev->rx_buffer))
            {
                RingBuffer_Read(&ESP32_Device.serial_dev->rx_buffer, &byte);
                if (rx_len < TCP_BUFFER_SIZE - 1)
                {
                    rx_buffer[rx_len++] = byte;
                    rx_buffer[rx_len] = '\0';
                } else {
                    // 本地缓冲区满了，但预期响应还未找到，这可能是一个问题
                    // 可以考虑记录一个错误或警告
                    Log_Message(LOG_LEVEL_WARNING, "[WiFi] Local rx_buffer full before finding response for %s", cmd->description);
                    // 强制退出内部while，触发超时或重试逻辑
                    break;
                }
                if (strstr((char*)rx_buffer, cmd->expected_resp))
                {
                    Log_Message(LOG_LEVEL_INFO, "[WiFi RX STR] WiFi cmd description is: '%s' Expected '%s' found in '%s'", cmd->description, cmd->expected_resp, rx_buffer); // 打印找到时的累积字符串
                    g_rtos_ops->SemaphoreGive(ESP32_Device.mutex);
                    return AT_ERR_NONE;
                }
            }
            g_rtos_ops->Delay(10);
        }
        Log_Message(LOG_LEVEL_WARNING, "[WiFi RX Timeout] Final accum: %s (Expected: %s for %s)", rx_buffer, cmd->expected_resp, cmd->description); // 超时时打印最终累积的字符串
        retry++;
        Log_Message(LOG_LEVEL_WARNING, "[WiFi] Timeout for command: %s, retry %d/%d", cmd->description, retry, cmd->retries);
    }

//    Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to receive expected response for %s", cmd->description);
    g_rtos_ops->SemaphoreGive(ESP32_Device.mutex);
    return AT_ERR_TIMEOUT;
}

/**
 * @brief 发送 WiFi TCP 数据
 */
AT_Error_Code WiFi_SendTCPData(const uint8_t *data, uint16_t length)
{
    // WiFi_Device_t *wifi_dev = &WiFi_Device;
    if (!g_rtos_ops || !ESP32_Device.serial_dev || !data || length == 0 || length > TCP_BUFFER_SIZE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for TCP send");
        return AT_ERR_SEND_FAILED;
    }

    if (!g_rtos_ops->SemaphoreTake(ESP32_Device.mutex, 0xFFFFFFFF))
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
        g_rtos_ops->SemaphoreGive(ESP32_Device.mutex);
        return status;
    }

    if (Serial_Driver_SendData(ESP32_Device.serial_dev, data, length) != SERIAL_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send TCP data");
        g_rtos_ops->SemaphoreGive(ESP32_Device.mutex);
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
    g_rtos_ops->SemaphoreGive(ESP32_Device.mutex);

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
    // WiFi_Device_t *wifi_dev = &WiFi_Device;
    if (!g_rtos_ops || !ESP32_Device.serial_dev || !buffer || !length || *length == 0 || *length > TCP_BUFFER_SIZE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid parameters for TCP receive");
        return AT_ERR_SEND_FAILED;
    }

    if (!g_rtos_ops->SemaphoreTake(ESP32_Device.mutex, 0xFFFFFFFF))
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
        if (RingBuffer_IsAvailable(&ESP32_Device.serial_dev->rx_buffer))
        {
            RingBuffer_Read(&ESP32_Device.serial_dev->rx_buffer, &byte);
            if (rx_len < *length && rx_len < TCP_BUFFER_SIZE - 1)
            {
                rx_buffer[rx_len++] = byte;
            }
            if (rx_len >= *length)
            {
                *length = rx_len;
                memcpy(buffer, rx_buffer, rx_len);
                Log_Message(LOG_LEVEL_INFO, "[WiFi] Received %d bytes of TCP data", rx_len);
                g_rtos_ops->SemaphoreGive(ESP32_Device.mutex);
                return AT_ERR_NONE;
            }
        }
        g_rtos_ops->Delay(10);
    }

    Log_Message(LOG_LEVEL_WARNING, "[WiFi] Timeout receiving TCP data");
    *length = 0;
    g_rtos_ops->SemaphoreGive(ESP32_Device.mutex);
    return AT_ERR_TIMEOUT;
}

/**
 * @brief 发送 BLE AT 指令
 */
AT_Error_Code BLE_SendATCommand(const AT_Cmd_Config *cmd)
{
    // BLE_Device_t *ble_dev = &BLE_Device; // 假设 BLE_Device 是一个全局变量或通过其他方式获取
    if (!g_rtos_ops || !ESP32_Device.serial_dev || !cmd || !cmd->at_cmd || !cmd->expected_resp)
    {
        Log_Message(LOG_LEVEL_ERROR, "[BLE] Invalid parameters for AT command");
        return AT_ERR_SEND_FAILED;
    }

    if (!g_rtos_ops->SemaphoreTake(ESP32_Device.mutex, 0xFFFFFFFF))
    {
        Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to take mutex");
        return AT_ERR_TIMEOUT;
    }
    // 清空硬件接收缓冲区中可能残留的数据
    uint8_t dummy_byte;
    while (RingBuffer_IsAvailable(&ESP32_Device.serial_dev->rx_buffer)) {
        RingBuffer_Read(&ESP32_Device.serial_dev->rx_buffer, &dummy_byte);
    }
    // 同时确保本地累积缓冲区也被清空 (虽然它在栈上每次都会重新初始化，但显式清零更清晰)
    uint8_t rx_buffer[TCP_BUFFER_SIZE]; // 注意: TCP_BUFFER_SIZE 可能也需要调整，确保足够大
    memset(rx_buffer, 0, sizeof(rx_buffer));
    uint16_t rx_len = 0;

    uint8_t retry = 0;
    while (retry <= cmd->retries)
    {
        memset(rx_buffer, 0, sizeof(rx_buffer));
        rx_len = 0;
        if (Serial_Driver_SendData(ESP32_Device.serial_dev, (uint8_t*)cmd->at_cmd, strlen(cmd->at_cmd)) != SERIAL_OK)
        {
            Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to send command: %s", cmd->description);
            retry++;
            g_rtos_ops->Delay(100); // 发送失败后稍作延时
            continue;
        }

        // uint8_t rx_buffer[TCP_BUFFER_SIZE] = {0};
        // uint16_t rx_len = 0;
        uint32_t start_tick = g_rtos_ops->GetTickCount();
        while ((g_rtos_ops->GetTickCount() - start_tick) < pdMS_TO_TICKS(cmd->timeout_ms))
        {
            uint8_t byte;
            if (RingBuffer_IsAvailable(&ESP32_Device.serial_dev->rx_buffer))
            {
                RingBuffer_Read(&ESP32_Device.serial_dev->rx_buffer, &byte);
                if (rx_len < TCP_BUFFER_SIZE - 1)
                {
                    rx_buffer[rx_len++] = byte;
                    rx_buffer[rx_len] = '\0';
                } else {
                    // 本地缓冲区满了，但预期响应还未找到，这可能是一个问题
                    // 可以考虑记录一个错误或警告
                    Log_Message(LOG_LEVEL_WARNING, "[BLE] Local rx_buffer full before finding response for %s", cmd->description);
                    // 强制退出内部while，触发超时或重试逻辑
                    break;
                }
                if (strstr((char*)rx_buffer, cmd->expected_resp))
                {
                    Log_Message(LOG_LEVEL_INFO, "[BLE RX STR] BLE cmd description is: '%s' Expected '%s' found in '%s'",cmd->description, cmd->expected_resp, rx_buffer); // 打印找到时的累积字符串
                    g_rtos_ops->SemaphoreGive(ESP32_Device.mutex);
                    return AT_ERR_NONE;
                }
            }
            g_rtos_ops->Delay(10);
        }
        Log_Message(LOG_LEVEL_WARNING, "[BLE RX Timeout] Final accum: %s (Expected: %s for %s)", rx_buffer, cmd->expected_resp, cmd->description); // 超时时打印最终累积的字符串
        retry++;
        Log_Message(LOG_LEVEL_WARNING, "[BLE] Timeout for command: %s, retry %d/%d", cmd->description, retry, cmd->retries);
    }

    Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to receive expected response for %s", cmd->description);
    g_rtos_ops->SemaphoreGive(ESP32_Device.mutex);
    return AT_ERR_TIMEOUT;
}

/**
 * @brief 发送 BLE 数据
 */
AT_Error_Code BLE_SendData(const uint8_t *data, uint16_t length)
{
    // BLE_Device_t *ble_dev = &BLE_Device;
    if (!g_rtos_ops || !ESP32_Device.serial_dev || !data || length == 0 || length > TCP_BUFFER_SIZE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[BLE] Invalid parameters for data send");
        return AT_ERR_SEND_FAILED;
    }

    if (!g_rtos_ops->SemaphoreTake(ESP32_Device.mutex, 0xFFFFFFFF))
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
        g_rtos_ops->SemaphoreGive(ESP32_Device.mutex);
        return status;
    }

    if (Serial_Driver_SendData(ESP32_Device.serial_dev, data, length) != SERIAL_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to send data");
        g_rtos_ops->SemaphoreGive(ESP32_Device.mutex);
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
    g_rtos_ops->SemaphoreGive(ESP32_Device.mutex);

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
    // BLE_Device_t *ble_dev = &BLE_Device;
    if (!g_rtos_ops || !ESP32_Device.serial_dev || !buffer || !length || *length == 0 || *length > TCP_BUFFER_SIZE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[BLE] Invalid parameters for data receive");
        return AT_ERR_SEND_FAILED;
    }

    if (!g_rtos_ops->SemaphoreTake(ESP32_Device.mutex, 0xFFFFFFFF))
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
        if (RingBuffer_IsAvailable(&ESP32_Device.serial_dev->rx_buffer))
        {
            RingBuffer_Read(&ESP32_Device.serial_dev->rx_buffer, &byte);
            if (rx_len < *length && rx_len < TCP_BUFFER_SIZE - 1)
            {
                rx_buffer[rx_len++] = byte;
            }
            if (rx_len >= *length)
            {
                *length = rx_len;
                memcpy(buffer, rx_buffer, rx_len);
                Log_Message(LOG_LEVEL_INFO, "[BLE] Received %d bytes of data", rx_len);
                g_rtos_ops->SemaphoreGive(ESP32_Device.mutex);
                return AT_ERR_NONE;
            }
        }
        g_rtos_ops->Delay(10);
    }

    Log_Message(LOG_LEVEL_WARNING, "[BLE] Timeout receiving data");
    *length = 0;
    g_rtos_ops->SemaphoreGive(ESP32_Device.mutex);
    return AT_ERR_TIMEOUT;
}
