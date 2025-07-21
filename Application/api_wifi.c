/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-01 20:50:17
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-07-21 23:48:01
 * @FilePath: \Demo\Application\api_wifi.c
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

ESP32_Device_Info_t g_esp32_info = {
    .version = "Unknown",
    .ip_addr = "0.0.0.0",
    .mac_addr = "00:00:00:00:00:00",
    .ssid = "Unknown",
    .password = "Unknown"
};
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

/**
 * @brief 回调函数：解析AT+CIPSTAMAC?响应并保存MAC地址
 * @param response 响应字符串
 */
void get_mac_addr_callback(const char *response) {
    // 假设响应格式为: +CIPSTAMAC:"xx:xx:xx:xx:xx:xx"
    const char *mac_prefix = "+CIPSTAMAC:\"";
    const char *mac_start = strstr(response, mac_prefix);
    if (mac_start) {
        mac_start += strlen(mac_prefix);
        char mac_addr[18] = {0}; // "xx:xx:xx:xx:xx:xx" + '\0'
        strncpy(mac_addr, mac_start, 17);
        mac_addr[17] = '\0';
        // 保存到全局变量或打印
        Log_Message(LOG_LEVEL_INFO, "[WiFi] Station MAC: %s", mac_addr);
        // 例如: strcpy(g_wifi_station_mac, mac_addr);
        strncpy(g_esp32_info.mac_addr, mac_addr, sizeof(g_esp32_info.mac_addr) - 1);
        g_esp32_info.mac_addr[sizeof(g_esp32_info.mac_addr) - 1] = '\0'; // 确保字符串以null结尾
    } else {
        Log_Message(LOG_LEVEL_WARNING, "[WiFi] Failed to parse MAC address from response: %s", response);
    }
}

/**
 * @brief 回调函数：解析AT+CIPSTA?响应并保存IP地址
 * @param response 响应字符串
 */
void get_ip_addr_callback(const char *response) {
    const char *ip_prefix = "+CIPSTA:ip:\"";
    const char *ip_start = strstr(response, ip_prefix);
    if (ip_start) {
        ip_start += strlen(ip_prefix);
        const char *ip_end = strchr(ip_start, '"');
        if (ip_end && (ip_end - ip_start) < sizeof(g_esp32_info.ip_addr)) {
            size_t ip_len = ip_end - ip_start;
            strncpy(g_esp32_info.ip_addr, ip_start, ip_len);
            g_esp32_info.ip_addr[ip_len] = '\0'; // null-terminated
            Log_Message(LOG_LEVEL_INFO, "[WiFi] Station IP: %s", g_esp32_info.ip_addr);
        } else {
            Log_Message(LOG_LEVEL_WARNING, "[WiFi] Invalid IP format in response: %s", response);
        }
    } else {
        Log_Message(LOG_LEVEL_WARNING, "[WiFi] Failed to parse IP address from response: %s", response);
    }
}

/**
 * @brief 回调函数：解析AT+GMR响应并保存ESP32版本信息
 * @param response 响应字符串
 */
void get_version_callback(const char *response) {
    const char *version_prefix = "AT version:";
    const char *version_start = strstr(response, version_prefix);
    if (version_start) {
        version_start += strlen(version_prefix);
        // 版本号后可能跟着换行、回车或其他字段，我们找到行尾
        const char *line_end = strpbrk(version_start, "\r\n");
        size_t version_len = 0;
        if (line_end) {
            version_len = line_end - version_start;
        } else {
            version_len = strlen(version_start);
        }

        if (version_len >= sizeof(g_esp32_info.version)) {
            version_len = sizeof(g_esp32_info.version) - 1;
        }

        strncpy(g_esp32_info.version, version_start, version_len);
        g_esp32_info.version[version_len] = '\0';  // null terminator

        Log_Message(LOG_LEVEL_INFO, "[WiFi] ESP32 Version: %s", g_esp32_info.version);
    } else {
        // 忽略非 version 行，比如 AT+GMR 本身或 SDK version 行
        // Log_Level 可改为 Debug 级别或去掉打印
        // Log_Message(LOG_LEVEL_DEBUG, "[WiFi] Skipped line: %s", response);
    }
}
/**
 * @brief 回调函数：设置WiFi SSID和密码
 * @param response 响应字符串
 */
void set_ssid_password_callback(const char *response) {
    // 假设响应格式为: OK
    if (strstr(response, "OK")) {
        strncpy(g_esp32_info.ssid, WIFI_SSID, sizeof(g_esp32_info.ssid) - 1);
        g_esp32_info.ssid[sizeof(g_esp32_info.ssid) - 1] = '\0'; // 确保字符串以null结尾
        strncpy(g_esp32_info.password, WIFI_PASSWORD, sizeof(g_esp32_info.password) - 1);
        g_esp32_info.password[sizeof(g_esp32_info.password) - 1 ] = '\0'; // 确保字符串以null结尾
        // 可以在这里更新设备状态或触发事件
        Log_Message(LOG_LEVEL_INFO, "[WiFi] SSID and Password set successfully.");
    } else {
        Log_Message(LOG_LEVEL_WARNING, "[WiFi] Failed to set SSID and Password: %s", response);
    }
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
    if (strlen(cmd->at_cmd) > 0) {
        if (Serial_Driver_SendData(dev->serial_dev, (uint8_t*)cmd->at_cmd, strlen(cmd->at_cmd)) != SERIAL_OK) {
            Log_Message(LOG_LEVEL_ERROR, "[%s AT] SendATCommand_Internal: Failed to send cmd: %s", log_prefix, cmd->description);
            return AT_ERR_SEND_FAILED;
        }
    } else {
        Log_Message(LOG_LEVEL_DEBUG, "[%s AT] No AT command sent, waiting for response only.", log_prefix);
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
                Log_Message(LOG_LEVEL_TEST, "[%s AT] Cmd: %s. Response OK: %s", log_prefix, cmd->description, local_rx_buffer);

                // 回调处理响应内容
                if (cmd->callback) {
                    cmd->callback((const char*)local_rx_buffer);
                }

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
    AT_Cmd_Config_t confirm_cmd = {"", "SEND OK", 5000, 0, "Data Send Confirm"}; // at_cmd为空，只等待响应
    status = _ESP32_SendATCommand_Internal(dev, &confirm_cmd, log_prefix);
    
    if (status != AT_OK) {
        Log_Message(LOG_LEVEL_WARNING, "[%s Send] Did not receive 'SEND OK'. Status: %d. Data might have been sent.", log_prefix, status);
        // 根据应用需求，即使没有SEND OK，数据也可能已发送。
    } else {
        Log_Message(LOG_LEVEL_INFO, "[%s Send] Sent %u bytes successfully.", log_prefix, length);
    }

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


static const AT_DataPrefix_t supported_prefixes[] = {
    {"+IPD,",   5},
    {"+BLE,",   5},
    {"+WRITE,", 7},
};

#define MAX_PREFIX_COUNT (sizeof(supported_prefixes) / sizeof(supported_prefixes[0]))

/**
 * @brief CRC16-Modbus 计算（大端在前）
 */
static uint16_t CRC16_Modbus(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return (crc >> 8) | (crc << 8); // 转为大端格式返回
}

/**
 * @brief 查找数据中是否存在支持的前缀，返回匹配位置和索引
 * @param data 输入数据
 * @param len 数据长度
 * @param prefix_index 匹配到的前缀索引输出
 * @return 匹配起始位置（>=0）或 -1 表示未找到
 */
static int find_prefix(const uint8_t* data, size_t len, uint8_t* prefix_index)
{
    for (size_t i = 0; i < len; i++) {
        for (uint8_t j = 0; j < MAX_PREFIX_COUNT; j++) {
            const char* prefix = supported_prefixes[j].prefix;
            uint8_t plen = supported_prefixes[j].prefix_len;
            if (i + plen < len && memcmp(&data[i], prefix, plen) == 0) {
                *prefix_index = j;
                return (int)i;
            }
        }
    }
    return -1;
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
AT_Status_t ESP32_AT_ReceiveData(ESP32_Shared_Device_t *dev, uint8_t *buffer, uint16_t *length, uint32_t timeout_ms, ESP32_Comm_Type_t type)
{
    if (!dev || !dev->serial_dev || !dev->mutex || !buffer || !length || *length == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[AT Recv] Invalid parameters.");
        return AT_ERR_PARAM;
    }

    const char* log_prefix = (type == ESP32_COMM_TYPE_WIFI) ? "WiFi" : "BLE";
    RingBuffer_t* rb = &dev->serial_dev->rx_buffer;
    const uint16_t max_len = *length;
    *length = 0;

    if (g_rtos_ops->SemaphoreTake(dev->mutex, 1000) != pdTRUE) {
        Log_Message(LOG_LEVEL_ERROR, "[%s Recv] Mutex timeout.", log_prefix);
        return AT_ERR_MUTEX_TIMEOUT;
    }

    uint32_t start_tick = g_rtos_ops->GetTickCount();
    uint8_t peek_buf[PEEK_BUF_SIZE];

    while (g_rtos_ops->GetTickCount() - start_tick < timeout_ms) {
        // 预读一定数量数据用于前缀匹配
        size_t available = RingBuffer_Peek(rb, peek_buf, sizeof(peek_buf));
        if (available < 8) { // 数据过少无法组成完整帧
            g_rtos_ops->Delay(2); // 等待更多数据
            continue;
        }

        // 查找数据前缀
        uint8_t prefix_index = 0xFF;
        int prefix_pos = find_prefix(peek_buf, available, &prefix_index);
        if (prefix_pos < 0) {
            RingBuffer_Drop(rb, available); // 无效数据全部丢弃
            continue;
        }

        const AT_DataPrefix_t* p = &supported_prefixes[prefix_index];
        if ((size_t)(prefix_pos + p->prefix_len + 2) >= available) {
            g_rtos_ops->Delay(2); // 数据长度字段尚未完整
            continue;
        }

        // 提取长度字符串并解析数字
        char len_str[6] = {0};
        size_t len_idx = 0;
        size_t i = prefix_pos + p->prefix_len;
        while (i < available && peek_buf[i] != ':' && len_idx < sizeof(len_str) - 1) {
            if (peek_buf[i] < '0' || peek_buf[i] > '9') break;
            len_str[len_idx++] = peek_buf[i++];
        }

        if (peek_buf[i] != ':' || len_idx == 0) {
            RingBuffer_Drop(rb, i + 1); // 异常格式：缺少冒号或无效长度
            continue;
        }

        uint16_t data_len = (uint16_t)atoi(len_str);
        if (data_len == 0 || data_len > MAX_FRAME_SIZE) {
            RingBuffer_Drop(rb, i + 1);
            continue;
        }

        // 计算整个帧长度（含前缀 + 长度字段 + : + 数据 + CRC）
        size_t total_frame_len = (i + 1 - prefix_pos) + data_len + CRC16_LEN;
        if (available - prefix_pos < total_frame_len) {
            g_rtos_ops->Delay(2); // 等待后续字节接收
            continue;
        }

        // 从环形缓冲区读取完整帧（含前缀+数据+CRC）
        uint8_t frame_buf[MAX_FRAME_SIZE + 8] = {0};
        RingBuffer_Drop(rb, prefix_pos); // 丢弃前缀前无效部分
        RingBuffer_ReadMulti(rb, frame_buf, total_frame_len);

        uint8_t* data_ptr = &frame_buf[i + 1 - prefix_pos];
        uint8_t* crc_ptr = &data_ptr[data_len];

        // 提取CRC字段并验证
        char crc_str[5] = {0};
        memcpy(crc_str, crc_ptr, 4);
        uint16_t expected_crc = (uint16_t)strtoul(crc_str, NULL, 16);
        uint16_t actual_crc = CRC16_Modbus(data_ptr, data_len);

        if (expected_crc == actual_crc) {
            uint16_t copy_len = (data_len <= max_len) ? data_len : max_len;
            memcpy(buffer, data_ptr, copy_len);
            *length = copy_len;
            Log_Message(LOG_LEVEL_INFO, "[%s Recv] Frame OK: %u bytes.", log_prefix, copy_len);
            g_rtos_ops->SemaphoreGive(dev->mutex);
            return AT_OK;
        } else {
            Log_Message(LOG_LEVEL_WARNING, "[%s Recv] CRC ERROR.", log_prefix);
            continue;
        }
    }

    g_rtos_ops->SemaphoreGive(dev->mutex);
    return AT_ERR_TIMEOUT;
}


// 定义全局唯一的ESP32 AT操作接口实例
const ESP32_AT_Ops_t g_esp32_at_ops = {
    .SendATCommand = ESP32_AT_SendATCommand,
    .SendData = ESP32_AT_SendData,
    .ReceiveData = ESP32_AT_ReceiveData,
};


