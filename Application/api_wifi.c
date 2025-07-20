/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-01 20:50:17
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-14 20:02:40
 * @FilePath: \Demo_backup\Application\api_wifi.c
 * @Description: ESP32 WiFi 和 BLE 模块统一驱动实现 (Refactored)
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "api_wifi.h"
#include "serial_driver.h" // 依赖底层串口驱动
#include "dev_config.h"    // 包含全局设备实例 g_esp32_dev
#include "log_system.h"
#include <string.h>
#include <stdio.h>   // For snprintf, atoi
#include "common_driver.h" // For delay_ms, Common_GPIO_Init (如果需要)
#include "pch.h"


/*
 * =====================================================================================
 * ESP32 硬件控制函数
 * =====================================================================================
 */

/**
 * @brief ESP32硬件相关初始化 (例如复位引脚)
 * @param dev 指向ESP32共享设备实例
 */
void ESP32_Hw_Init(ESP32_Shared_Device_t *dev)
{
    if (!dev || !dev->reset_port) {
        Log_Message(LOG_LEVEL_ERROR, "[ESP32 Hw] Init: Invalid device or reset port.");
        return;
    }
    // 初始化复位引脚为输出模式
    Common_GPIO_Init(dev->reset_port, dev->reset_pin, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz, 0);
    
    if (dev->mutex == NULL) {
        dev->mutex = g_rtos_ops->SemaphoreCreate();
        if (dev->mutex == NULL) {
            Log_Message(LOG_LEVEL_ERROR, "[ESP32] Device Init: Failed to create mutex.");
            return; // 使用更具体的错误码
        }
    }
    // 确保复位引脚处于非复位状态 (高电平)
    GPIO_SetBits(dev->reset_port, dev->reset_pin);
    Log_Message(LOG_LEVEL_INFO, "[ESP32 Hw] Reset pin initialized.");
}

/**
 * @brief ESP32硬件复位
 * @param dev 指向ESP32共享设备实例
 */
void ESP32_Hw_Reset(ESP32_Shared_Device_t *dev)
{
    if (!dev || !dev->reset_port) {
        Log_Message(LOG_LEVEL_ERROR, "[ESP32 Hw] Reset: Invalid device or reset port.");
        return;
    }
    Log_Message(LOG_LEVEL_INFO, "[ESP32 Hw] Performing hardware reset...");
    GPIO_ResetBits(dev->reset_port, dev->reset_pin); // 拉低复位引脚
    delay_ms(100);                                  // 保持低电平一段时间
    GPIO_SetBits(dev->reset_port, dev->reset_pin);  // 拉高复位引脚
    delay_ms(500);                                  // 等待ESP32启动
    Log_Message(LOG_LEVEL_INFO, "[ESP32 Hw] Hardware reset completed.");
}

/*
 * =====================================================================================
 * 内部辅助函数：发送AT指令并等待响应
 * 注意：此函数假定调用者已经获取了 g_esp32_dev.mutex 互斥锁。
 * =====================================================================================
 */
/**
 * @brief 内部辅助函数：发送AT指令并等待响应 (已持有互斥锁时调用)
 * @param dev 指向ESP32共享设备实例
 * @param cmd AT指令配置
 * @param log_prefix 日志前缀 (例如 "WiFi", "BLE")
 * @return AT_Status_t 操作状态
 */
static AT_Status_t _ESP32_SendATCommand_Internal(ESP32_Shared_Device_t *dev, const AT_Cmd_Config_t* cmd, const char* log_prefix)
{
    if (!dev || !dev->serial_dev || !cmd || !cmd->at_cmd || !cmd->expected_resp) {
        Log_Message(LOG_LEVEL_ERROR, "[%s AT] SendATCommand_Internal: Invalid parameters.", log_prefix);
        return AT_ERR_PARAM;
    }

    // 清空接收缓冲区，避免旧数据干扰
    RingBuffer_Clear(&dev->serial_dev->rx_buffer);

    // 发送指令
    if (Serial_Driver_SendData(dev->serial_dev, (uint8_t*)cmd->at_cmd, strlen(cmd->at_cmd)) != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[%s AT] SendATCommand_Internal: Failed to send cmd: %s", log_prefix, cmd->description);
        return AT_ERR_SEND_FAILED;
    }

    // 等待响应
    uint8_t local_rx_buffer[TCP_BUFFER_SIZE + 50] = {0}; // 增加一些额外空间用于响应头尾
    uint16_t rx_len = 0;
    uint32_t start_tick = g_rtos_ops->GetTickCount();
    uint32_t current_tick;
    const char* expected_resp = cmd->expected_resp; // 期望响应

    // 持续从串口读取数据，直到超时或收到期望的响应
    while (true) {
        current_tick = g_rtos_ops->GetTickCount();
        if ((current_tick - start_tick) >= cmd->timeout_ms) {
            Log_Message(LOG_LEVEL_WARNING, "[%s AT] Timeout for cmd: %s. Current buffer: '%s'",
                        log_prefix, cmd->description, local_rx_buffer);
            return AT_ERR_TIMEOUT;
        }

        uint8_t byte;
        // 使用非阻塞读取或短超时等待单个字节，避免长时间阻塞
        if (Serial_Driver_ReadByte(dev->serial_dev, &byte) == SERIAL_OK) {
            if (rx_len < sizeof(local_rx_buffer) - 1) {
                local_rx_buffer[rx_len++] = byte;
                local_rx_buffer[rx_len] = '\0'; // 确保字符串以null结尾
            } else {
                Log_Message(LOG_LEVEL_WARNING, "[%s AT] Receive buffer full for cmd: %s. Data truncated.",
                            log_prefix, cmd->description);
                // 缓冲区满也尝试继续检查已接收部分是否包含期望响应
            }

            // 检查是否包含期望的响应
            if (strstr((char*)local_rx_buffer, expected_resp)) {
                Log_Message(LOG_LEVEL_INFO, "[%s AT] Cmd: %s. Response OK: '%s'",
                            log_prefix, cmd->description, local_rx_buffer);
                return AT_OK;
            }
        }
        // 短暂延时，让出CPU
        g_rtos_ops->Delay(1);
    }
}

/*
 * =====================================================================================
 * ESP32 AT 命令操作接口实现
 * =====================================================================================
 */

/**
 * @brief 发送 ESP32 AT 指令 (通用接口，处理互斥锁和重试)
 * @param dev 指向ESP32共享设备实例
 * @param cmd AT 指令配置
 * @param type 通信类型 (WiFi/BLE)
 * @return AT_Status_t 操作状态
 */
static AT_Status_t ESP32_AT_SendATCommand(ESP32_Shared_Device_t *dev, const AT_Cmd_Config_t* cmd, ESP32_Comm_Type_t type)
{
    if (!dev || !dev->mutex || !cmd) {
        Log_Message(LOG_LEVEL_ERROR, "[ESP32 AT] SendATCommand: Invalid device, mutex, or cmd.");
        return AT_ERR_PARAM;
    }

    const char* log_prefix = (type == ESP32_COMM_TYPE_WIFI) ? "WiFi" : "BLE";

    // 获取互斥锁
    if (g_rtos_ops->SemaphoreTake(dev->mutex, 1000) != pdTRUE) { // 1秒超时
        Log_Message(LOG_LEVEL_ERROR, "[%s AT] SendATCommand: Failed to take ESP32 mutex (timeout).", log_prefix);
        return AT_ERR_MUTEX_TIMEOUT;
    }

    AT_Status_t status = AT_ERR_TIMEOUT; // 初始状态为超时
    for (uint8_t retry = 0; retry <= cmd->retries; retry++) {
        status = _ESP32_SendATCommand_Internal(dev, cmd, log_prefix);
        if (status == AT_OK) {
            break; // 成功则退出重试循环
        }
        if (retry < cmd->retries) {
            Log_Message(LOG_LEVEL_WARNING, "[%s AT] Cmd: %s failed (%d). Retrying %u/%u...",
                        log_prefix, cmd->description, status, retry + 1, cmd->retries);
            g_rtos_ops->Delay(200); // 重试前短暂延时
        }
    }

    g_rtos_ops->SemaphoreGive(dev->mutex); // 释放互斥锁
    return status;
}

/**
 * @brief 发送数据 (TCP/BLE透传数据)
 * @param dev 指向ESP32共享设备实例
 * @param data 要发送的数据缓冲区
 * @param length 数据长度
 * @param type 通信类型 (WiFi/BLE)
 * @return AT_Status_t 操作状态
 */
static AT_Status_t ESP32_AT_SendData(ESP32_Shared_Device_t *dev, const uint8_t *data, uint16_t length, ESP32_Comm_Type_t type)
{
    if (!dev || !dev->serial_dev || !dev->mutex || !data || length == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[ESP32 Data Send] Invalid parameters.");
        return AT_ERR_PARAM;
    }
    if (length > TCP_BUFFER_SIZE) { // ESP32 AT指令通常对单次CIPSEND/BLESEND长度有限制
        Log_Message(LOG_LEVEL_ERROR, "[ESP32 Data Send] Data length %u exceeds max buffer %u.", length, TCP_BUFFER_SIZE);
        return AT_ERR_PARAM; // 超过允许的最大长度
    }

    const char* log_prefix = (type == ESP32_COMM_TYPE_WIFI) ? "WiFi TCP" : "BLE";
    char cmd_str[32];
    AT_Status_t status = AT_ERR_SEND_FAILED;

    // 获取互斥锁
    if (g_rtos_ops->SemaphoreTake(dev->mutex, 0xFFFFFFFF) != pdTRUE) { // 永远等待
        Log_Message(LOG_LEVEL_ERROR, "[%s Send] Failed to take ESP32 mutex.", log_prefix);
        return AT_ERR_MUTEX_TIMEOUT;
    }

    // 1. 发送准备指令 (例如 AT+CIPSEND=length 或 AT+BLESEND=length)
    if (type == ESP32_COMM_TYPE_WIFI) {
        snprintf(cmd_str, sizeof(cmd_str), "AT+CIPSEND=%d\r\n", length);
    } else { // ESP32_COMM_TYPE_BLE
        // 假设 BLE 发送指令为 AT+BLUESPPDAT=<length> (或类似，需查阅具体AT手册)
        // 这里使用一个通用的 BLESEND 占位
        snprintf(cmd_str, sizeof(cmd_str), "AT+BLESEND=%d\r\n", length);
    }

    AT_Cmd_Config_t prep_cmd = {cmd_str, ">", 2000, 0, "Data Send Prep"}; // 期待 '>' 提示

    status = _ESP32_SendATCommand_Internal(dev, &prep_cmd, log_prefix);
    if (status != AT_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[%s Send] Failed to get '>' prompt. Status: %d.", log_prefix, status);
        g_rtos_ops->SemaphoreGive(dev->mutex);
        return status;
    }

    // 2. 发送实际数据载荷
    if (Serial_Driver_SendData(dev->serial_dev, data, length) != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[%s Send] Failed to send data payload.", log_prefix);
        g_rtos_ops->SemaphoreGive(dev->mutex);
        return AT_ERR_SEND_FAILED;
    }

    // 3. 等待 "SEND OK" 响应
    // 注意：AT+CIPSEND的"SEND OK"响应时间可能较长，且依赖网络。
    // AT_Cmd_Config_t confirm_cmd = {"", "SEND OK", 5000, 0, "Data Send Confirm"}; // at_cmd为空，只等待响应
    // status = _ESP32_SendATCommand_Internal(dev, &confirm_cmd, log_prefix);
    //
    // if (status != AT_OK) {
    //     Log_Message(LOG_LEVEL_WARNING, "[%s Send] Did not receive 'SEND OK'. Status: %d. Data might have been sent.", log_prefix, status);
    //     // 根据应用需求，即使没有SEND OK，数据也可能已发送。
    // } else {
    //     Log_Message(LOG_LEVEL_INFO, "[%s Send] Sent %u bytes successfully.", log_prefix, length);
    // }

    // 由于某些固件可能在发送数据后立即返回 SEND OK，或者存在其他延迟，
    // 这里采取一个更灵活的等待方式：等待一段时间，看看串口是否空闲。
    // 实际应用中，更健壮的可能是通过额外的AT指令查询发送状态或等待服务器响应。
    // 为了简化和通用性，我们直接假设发送成功，并等待一小段时间让模块处理。
    g_rtos_ops->Delay(100); // 等待模块处理发送
    Log_Message(LOG_LEVEL_INFO, "[%s Send] Sent %u bytes (assuming success).", log_prefix, length);
    status = AT_OK; // 假定发送命令本身成功，具体数据传输结果由更高层逻辑判断

    g_rtos_ops->SemaphoreGive(dev->mutex); // 释放互斥锁
    return status;
}

/**
 * @brief 接收数据 (TCP/BLE透传数据)
 * @param dev 指向ESP32共享设备实例
 * @param buffer 数据存储缓冲区
 * @param length 期望接收的长度（输入），实际接收的长度（输出）
 * @param timeout_ms 超时时间（毫秒）
 * @param type 通信类型 (WiFi/BLE)
 * @return AT_Status_t 操作状态
 */
static AT_Status_t ESP32_AT_ReceiveData(ESP32_Shared_Device_t *dev, uint8_t *buffer, uint16_t *length, uint32_t timeout_ms, ESP32_Comm_Type_t type)
{
    if (!dev || !dev->serial_dev || !dev->mutex || !buffer || !length || *length == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[ESP32 Data Recv] Invalid parameters.");
        return AT_ERR_PARAM;
    }

    const char* log_prefix = (type == ESP32_COMM_TYPE_WIFI) ? "WiFi TCP" : "BLE";
    RingBuffer_t* rb = &dev->serial_dev->rx_buffer;
    uint16_t max_len = *length;
    uint16_t received_len = 0;
    uint32_t start_tick = g_rtos_ops->GetTickCount();
    *length = 0; // 重置实际接收长度

    // 获取互斥锁
    if (g_rtos_ops->SemaphoreTake(dev->mutex, 1000) != pdTRUE) { // 1秒超时
        Log_Message(LOG_LEVEL_ERROR, "[%s Recv] Failed to take ESP32 mutex (timeout).", log_prefix);
        return AT_ERR_MUTEX_TIMEOUT;
    }

    // --- 数据解析状态机 (主要针对 +IPD 格式的数据) ---
    // 对于BLE或其他格式，可能需要调整或跳过此解析
    enum {
        PARSE_STATE_INIT,
        PARSE_STATE_WAIT_FOR_PREFIX, // 等待 "+IPD," 或其他数据前缀
        PARSE_STATE_READ_LEN,        // 读取数据长度
        PARSE_STATE_READ_COLON,      // 等待冒号分隔符
        PARSE_STATE_READ_DATA        // 读取实际数据
    } parse_state = PARSE_STATE_INIT;

    char len_str[6] = {0}; // 用于存储长度字符串 (最大99999字节)
    uint8_t len_idx = 0;
    uint16_t expected_data_len = 0;
    uint8_t current_prefix_idx = 0;
    const char* ipd_prefix = "+IPD,"; // WiFi TCP数据通常有此前缀

    Log_Message(LOG_LEVEL_DEBUG, "[%s Recv] Starting data reception. Max %u bytes, timeout %lu ms.", log_prefix, max_len, timeout_ms);

    while (true) {
        uint32_t current_tick = g_rtos_ops->GetTickCount();
        if ((current_tick - start_tick) >= timeout_ms) {
            Log_Message(LOG_LEVEL_WARNING, "[%s Recv] Timeout waiting for data or full packet.", log_prefix);
            goto recv_end; // 跳出循环并释放锁
        }

        uint8_t byte;
        // 尝试从环形缓冲区读取一个字节，使用剩余超时时间
        uint32_t remaining_timeout = timeout_ms - (current_tick - start_tick);
        if (remaining_timeout == 0) remaining_timeout = 1; // 至少等待1ms，避免死循环

        if (g_rtos_ops->SemaphoreTake(rb->sem, remaining_timeout) == pdTRUE) {
            if (RingBuffer_Read(rb, &byte) != RB_OK) {
                // RingBuffer_Read 内部已经获取信号量，这里再次读取失败不应该发生
                Log_Message(LOG_LEVEL_ERROR, "[%s Recv] Ring buffer read error after semaphore took.", log_prefix);
                continue;
            }

            switch (parse_state) {
                case PARSE_STATE_INIT:
                    // 初始状态，等待第一个字节，或者直接进入等待前缀
                    // 对于 +IPD 格式，直接进入等待前缀状态
                    parse_state = PARSE_STATE_WAIT_FOR_PREFIX;
                    // fallthrough to PARSE_STATE_WAIT_FOR_PREFIX
                case PARSE_STATE_WAIT_FOR_PREFIX:
                    // 尝试匹配 "+IPD," 前缀
                    if (byte == ipd_prefix[current_prefix_idx]) {
                        current_prefix_idx++;
                        if (current_prefix_idx == strlen(ipd_prefix)) {
                            parse_state = PARSE_STATE_READ_LEN; // 匹配到前缀，进入读取长度状态
                            len_idx = 0;
                            memset(len_str, 0, sizeof(len_str)); // 清空长度字符串缓冲区
                            current_prefix_idx = 0; // 重置前缀索引
                        }
                    } else {
                        // 不匹配，重置状态。如果当前字节是 '+'，则可能是新前缀的开始
                        current_prefix_idx = 0;
                        if (byte == '+') { // 可能是新的 +IPD 开始
                             current_prefix_idx = 1;
                        }
                        // 否则，忽略当前字节，继续等待
                    }
                    break;

                case PARSE_STATE_READ_LEN:
                    if (byte >= '0' && byte <= '9') {
                        if (len_idx < sizeof(len_str) - 1) {
                            len_str[len_idx++] = byte;
                        } else {
                            Log_Message(LOG_LEVEL_WARNING, "[%s Recv] IPD length string too long.", log_prefix);
                            parse_state = PARSE_STATE_INIT; // 错误，重置
                        }
                    } else if (byte == ':') {
                        len_str[len_idx] = '\0'; // 终止字符串
                        expected_data_len = atoi(len_str);
                        if (expected_data_len > 0 && expected_data_len <= max_len) {
                            parse_state = PARSE_STATE_READ_DATA; // 长度有效，进入读取数据状态
                            received_len = 0; // 重置已接收数据计数
                            Log_Message(LOG_LEVEL_DEBUG, "[%s Recv] IPD: Expecting %u bytes.", log_prefix, expected_data_len);
                        } else {
                            Log_Message(LOG_LEVEL_WARNING, "[%s Recv] IPD: Invalid data length %u (max %u).", log_prefix, expected_data_len, max_len);
                            parse_state = PARSE_STATE_INIT; // 长度无效，重置
                        }
                    } else {
                        Log_Message(LOG_LEVEL_WARNING, "[%s Recv] IPD: Unexpected char '%c' (0x%02X) in length field.", log_prefix, byte, byte);
                        parse_state = PARSE_STATE_INIT; // 错误，重置
                    }
                    break;

                case PARSE_STATE_READ_DATA:
                    if (received_len < expected_data_len) {
                        buffer[received_len++] = byte;
                    }

                    if (received_len == expected_data_len) {
                        *length = received_len;
                        Log_Message(LOG_LEVEL_INFO, "[%s Recv] Received %u bytes successfully.", log_prefix, received_len);
                        g_rtos_ops->SemaphoreGive(dev->mutex); // 释放互斥锁
                        return AT_OK; // 成功接收整个包
                    } else if (received_len >= max_len) {
                         // 用户提供的缓冲区已满，即使未收到完整IPD包也停止
                         Log_Message(LOG_LEVEL_WARNING, "[%s Recv] User buffer full (%u bytes), but expected %u. Data truncated.", log_prefix, max_len, expected_data_len);
                         *length = received_len;
                         g_rtos_ops->SemaphoreGive(dev->mutex);
                         return AT_OK; // 返回部分数据
                    }
                    break;
            }
        }
    }

recv_end:
    g_rtos_ops->SemaphoreGive(dev->mutex); // 释放互斥锁
    if (received_len > 0) {
        *length = received_len; // 返回已接收的部分数据
        Log_Message(LOG_LEVEL_INFO, "[%s Recv] Partial data received %u bytes before timeout.", log_prefix, received_len);
        return AT_OK; // 即使超时，如果收到数据也算成功（根据需求决定）
    }
    return AT_ERR_TIMEOUT;
}


// 定义全局唯一的ESP32 AT操作接口实例
const ESP32_AT_Ops_t g_esp32_at_ops = {
    .SendATCommand = ESP32_AT_SendATCommand,
    .SendData = ESP32_AT_SendData,
    .ReceiveData = ESP32_AT_ReceiveData,
};
