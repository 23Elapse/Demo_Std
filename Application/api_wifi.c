/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-01 20:50:17
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-09 21:29:00
 * @FilePath: \Demo_backup\Application\api_wifi.c
 * @Description: ESP32 WiFi 和 BLE 模块统一驱动实现
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "api_wifi.h"
#include "serial_driver.h"
#include "dev_config.h" 
#include "log_system.h"
#include <string.h>
#include <stdio.h>
#include "serial_interface.h"
#include "pch.h" // 假设包含 Common_GPIO_Init, delay_ms


/**
 * @brief ESP32硬件相关初始化 (例如复位引脚)
 */
void ESP32_Device_HwInit(void)
{
    // 注意：这里的初始化逻辑最好由 App_Driver_Init 统一管理，此处保留是为了兼容旧调用
    // 在理想的架构中，此函数可以为空，所有GPIO初始化都在一个地方完成。
    if (g_esp32_dev.reset_port) {
        Common_GPIO_Init(g_esp32_dev.reset_port, g_esp32_dev.reset_pin, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz, 0);
    }
}
/**
 * @brief ESP32硬件复位
 */
void ESP32_Device_HwReset(void)
{
    if (g_esp32_dev.reset_port) {
        GPIO_ResetBits(g_esp32_dev.reset_port, g_esp32_dev.reset_pin);
        delay_ms(100);
        GPIO_SetBits(g_esp32_dev.reset_port, g_esp32_dev.reset_pin);
        delay_ms(500);
    }
}

/**
 * @brief 内部辅助函数：发送AT指令并等待响应 (已持有互斥锁时调用)
 */
static AT_Error_Code _ESP32_SendATCommand_nolock(const AT_Cmd_Config* cmd, const char* log_prefix)
{
    if (!g_esp32_dev.serial_dev || !cmd || !cmd->at_cmd || !cmd->expected_resp) {
        return AT_ERR_PARAM;
    }

    // 清空接收缓冲区
    RingBuffer_Clear(&g_esp32_dev.serial_dev->rx_buffer);

    // 发送指令
    if (Serial_Operations.SendData(g_esp32_dev.serial_dev, (uint8_t*)cmd->at_cmd, strlen(cmd->at_cmd)) != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[%s AT] Failed to send cmd: %s", log_prefix, cmd->description);
        return AT_ERR_SEND_FAILED;
    }

    // 等待响应
    uint8_t local_rx_buffer[TCP_BUFFER_SIZE] = {0};
    uint16_t rx_len = 0;
    uint32_t start_tick = g_rtos_ops->GetTickCount();

    while ((g_rtos_ops->GetTickCount() - start_tick) < cmd->timeout_ms) {
        uint8_t byte;
        // 注意：这里不应该使用SemaphoreTake，因为这个操作是在一个更大的锁（g_esp32_dev.mutex）内部
        if (RingBuffer_Read(&g_esp32_dev.serial_dev->rx_buffer, &byte) == RB_OK) {
            if (rx_len < sizeof(local_rx_buffer) - 1) {
                local_rx_buffer[rx_len++] = byte;
            }
            if (strstr((char*)local_rx_buffer, cmd->expected_resp)) {
                Log_Message(LOG_LEVEL_INFO, "[%s AT] Send AT cmd:\r\n %s Response:\r\n %s", log_prefix, cmd->at_cmd, local_rx_buffer);
                return AT_ERR_NONE;
            }
        }
        g_rtos_ops->Delay(10); // 短暂让出CPU
    }

    Log_Message(LOG_LEVEL_WARNING, "[%s AT] Timeout for cmd:\r\n %s Response:\r\n %s", log_prefix, cmd->at_cmd, local_rx_buffer);
    return AT_ERR_TIMEOUT;
}


/**
 * @brief 发送 WiFi/BLE AT 指令 (公共接口，处理互斥锁和重试)
 */
AT_Error_Code ESP32_SendATCommand(const AT_Cmd_Config* cmd, const char* log_prefix)
{
    if (!g_rtos_ops || !g_esp32_dev.mutex || !cmd) {
        return AT_ERR_PARAM;
    }

    if (g_rtos_ops->SemaphoreTake(g_esp32_dev.mutex, 1000) != pdTRUE) {
        Log_Message(LOG_LEVEL_ERROR, "[%s AT] Failed to take ESP32 mutex", log_prefix);
        return AT_ERR_TIMEOUT;
    }

    AT_Error_Code status = AT_ERR_TIMEOUT;
    for (uint8_t retry = 0; retry <= cmd->retries; retry++) {
        status = _ESP32_SendATCommand_nolock(cmd, log_prefix);
        if (status == AT_ERR_NONE) {
            break;
        }
        if (retry < cmd->retries) {
            g_rtos_ops->Delay(200); // 重试前延时
        }
    }

    g_rtos_ops->SemaphoreGive(g_esp32_dev.mutex);
    return status;
}

// 包装函数，保持旧API兼容性
AT_Error_Code WiFi_SendATCommand(const AT_Cmd_Config *cmd) {
    return ESP32_SendATCommand(cmd, "WiFi");
}
AT_Error_Code BLE_SendATCommand(const AT_Cmd_Config *cmd) {
    return ESP32_SendATCommand(cmd, "BLE");
}



/**
 * @brief 发送 WiFi TCP 数据
 */
AT_Error_Code WiFi_SendTCPData(const uint8_t *data, uint16_t length)
{
    if (!g_rtos_ops || !g_esp32_dev.serial_dev || !g_esp32_dev.mutex || !data || length == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi TCP Send] Invalid parameters");
        return AT_ERR_SEND_FAILED;
    }
    if (length > TCP_BUFFER_SIZE) { // ESP32 AT指令通常对单次CIPSEND长度有限制
        Log_Message(LOG_LEVEL_ERROR, "[WiFi TCP Send] Data length %d exceeds max buffer %d", length, TCP_BUFFER_SIZE);
        return AT_ERR_SEND_FAILED;
    }

    if (!g_rtos_ops->SemaphoreTake(g_esp32_dev.mutex, 0xFFFFFFFF)) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi TCP Send] Failed to take ESP32 mutex");
        return AT_ERR_TIMEOUT;
    }

    AT_Error_Code status = AT_ERR_SEND_FAILED;
    char cmd_str[32];
    snprintf(cmd_str, sizeof(cmd_str), "AT+CIPSEND=%d\r\n", length);

    AT_Cmd_Config prep_cmd = {cmd_str, ">", 2000, 0, "TCP Send Prep"}; // Prep cmd, 0 retries internally

    // 1. 发送CIPSEND=length指令，期望">"
    status = _ESP32_SendATCommand_nolock(&prep_cmd, "WiFi TCP");
    if (status != AT_ERR_NONE) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi TCP Send] Failed to get '>' prompt. Status: %d", status);
        g_rtos_ops->SemaphoreGive(g_esp32_dev.mutex);
        return status;
    }

    // 2. 发送实际数据
    if (Serial_Driver_SendData(g_esp32_dev.serial_dev, data, length) != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi TCP Send] Failed to send data payload.");
        g_rtos_ops->SemaphoreGive(g_esp32_dev.mutex);
        return AT_ERR_SEND_FAILED;
    }

    // 3. 等待 "SEND OK"
    //    注意：AT+CIPSEND的"SEND OK"响应时间可能较长，且依赖网络。
    //    有些固件可能在数据发送后不立即返回 "SEND OK"，或者其格式可能变化。
    //    此处使用一个虚拟的AT_Cmd_Config来复用 _ESP32_SendATCommand_nolock 的接收逻辑。
    //    at_cmd为空字符串，因为我们不发送额外命令，只是等待响应。
    AT_Cmd_Config confirm_cmd = {"", "SEND OK", 5000, 0, "TCP Send Confirm"}; // 5s timeout for SEND OK
    status = _ESP32_SendATCommand_nolock(&confirm_cmd, "WiFi TCP");

    if (status != AT_ERR_NONE) {
        Log_Message(LOG_LEVEL_WARNING, "[WiFi TCP Send] Did not receive 'SEND OK'. Status: %d. Data might have been sent.", status);
        // 根据应用需求，即使没有SEND OK，数据也可能已发送。
        // 有些应用可能认为没有SEND OK就是失败。
    } else {
        Log_Message(LOG_LEVEL_INFO, "[WiFi TCP Send] Sent %d bytes successfully.", length);
    }

    g_rtos_ops->SemaphoreGive(g_esp32_dev.mutex);
    return status; // 返回SEND OK的最终状态
}

/**
 * @brief 接收 WiFi TCP 数据 (已优化，并恢复了健壮的+IPD头解析)
 * @note  本函数使用信号量阻塞等待，仅在有数据时才开始解析，兼具效率与可靠性。
 */
AT_Error_Code WiFi_ReceiveTCPData(uint8_t *buffer, uint16_t *length, uint32_t timeout_ms)
{
    if (!g_rtos_ops || !g_esp32_dev.serial_dev || !g_esp32_dev.mutex || !buffer || !length || *length == 0) {
        return AT_ERR_PARAM;
    }

    if (g_rtos_ops->SemaphoreTake(g_esp32_dev.mutex, 1000) != pdTRUE) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi TCP Recv] Failed to take ESP32 mutex");
        return AT_ERR_TIMEOUT;
    }

    RingBuffer_t* rb = &g_esp32_dev.serial_dev->rx_buffer;
    uint16_t max_len = *length;
    uint16_t received_len = 0;
    uint32_t start_tick = g_rtos_ops->GetTickCount();
    *length = 0;

    // --- IPD 解析状态机 ---
    enum {
        PARSING_STATE_WAIT_FOR_PLUS,
        PARSING_STATE_WAIT_FOR_IPD,
        PARSING_STATE_READ_LEN,
        PARSING_STATE_READ_COLON,
        PARSING_STATE_READ_DATA
    } state = PARSING_STATE_WAIT_FOR_PLUS;

    char len_str[6] = {0};
    uint8_t len_idx = 0;
    uint16_t data_to_read = 0;

    while ((g_rtos_ops->GetTickCount() - start_tick) < timeout_ms) {
        uint32_t remaining_timeout = timeout_ms - (g_rtos_ops->GetTickCount() - start_tick);
        if (remaining_timeout > timeout_ms) remaining_timeout = 0;

        // 阻塞等待新字节的到来
        if (g_rtos_ops->SemaphoreTake(rb->sem, remaining_timeout) == pdTRUE) {
            uint8_t byte;
            if (RingBuffer_Read(rb, &byte) != RB_OK) continue; // 不应该发生

            switch (state) {
                case PARSING_STATE_WAIT_FOR_PLUS:
                    if (byte == '+') {
                        state = PARSING_STATE_WAIT_FOR_IPD;
                    }
                    break;
                
                case PARSING_STATE_WAIT_FOR_IPD:
                    if (strstr((const char[]){byte, 0}, "IPD,")) { // 这是一个简化的匹配，实际应顺序匹配"IPD,"
                        state = PARSING_STATE_READ_LEN;
                        len_idx = 0;
                        memset(len_str, 0, sizeof(len_str));
                    } else {
                        state = PARSING_STATE_WAIT_FOR_PLUS; // 不匹配，重新等待'+'
                    }
                    break;
                    
                case PARSING_STATE_READ_LEN:
                    if (byte >= '0' && byte <= '9' && len_idx < sizeof(len_str) - 1) {
                        len_str[len_idx++] = byte;
                    } else if (byte == ':') {
                        data_to_read = atoi(len_str);
                        if (data_to_read > 0 && data_to_read <= max_len) {
                            state = PARSING_STATE_READ_DATA;
                        } else {
                            Log_Message(LOG_LEVEL_WARNING, "[WiFi TCP Recv] Invalid IPD len: %u", data_to_read);
                            state = PARSING_STATE_WAIT_FOR_PLUS;
                        }
                    } else {
                        state = PARSING_STATE_WAIT_FOR_PLUS; // 解析长度出错
                    }
                    break;

                case PARSING_STATE_READ_DATA:
                    buffer[received_len++] = byte;
                    if (received_len == data_to_read) {
                        // 成功接收完一个IPD包
                        *length = received_len;
                        g_rtos_ops->SemaphoreGive(g_esp32_dev.mutex);
                        return AT_ERR_NONE;
                    }
                    break;

                default: // Should not happen
                    state = PARSING_STATE_WAIT_FOR_PLUS;
                    break;
            }
        } else {
            // SemaphoreTake 超时
            Log_Message(LOG_LEVEL_INFO, "[WiFi TCP Recv] Timeout waiting for data.");
            goto recv_end;
        }
    }

recv_end:
    g_rtos_ops->SemaphoreGive(g_esp32_dev.mutex);
    if (received_len > 0) { // 如果超时但已收到部分数据
        *length = received_len;
        return AT_ERR_NONE;
    }
    return AT_ERR_TIMEOUT;
}

/**
 * @brief 接收 WiFi TCP 数据
 */
// AT_Error_Code WiFi_ReceiveTCPData(uint8_t *buffer, uint16_t *length, uint32_t timeout_ms)
// {
//     if (!g_rtos_ops || !g_esp32_dev.serial_dev || !g_esp32_dev.mutex || !buffer || !length || *length == 0) {
//         Log_Message(LOG_LEVEL_ERROR, "[WiFi TCP Recv] Invalid parameters");
//         return AT_ERR_SEND_FAILED; // Or a more specific error
//     }

//     if (!g_rtos_ops->SemaphoreTake(g_esp32_dev.mutex, 0xFFFFFFFF)) {
//         Log_Message(LOG_LEVEL_ERROR, "[WiFi TCP Recv] Failed to take ESP32 mutex");
//         return AT_ERR_TIMEOUT;
//     }

//     // TCP数据通常以 "+IPD,<len>:<data>" 的形式到达。
//     // 简化的实现：直接读取串口数据直到超时或缓冲区满。
//     // 健壮的实现需要解析 "+IPD"头来确定数据长度。

//     uint16_t received_len = 0;
//     uint16_t max_len = *length; // 用户期望的最大长度
//     uint32_t start_tick = g_rtos_ops->GetTickCount();
//     char ipd_prefix[15]; // For "+IPD,"
//     uint16_t ipd_data_len = 0;
//     uint8_t parsing_ipd_header = 1; // 状态：1=等待+IPD, 2=解析长度, 0=接收数据
//     uint16_t ipd_header_idx = 0;


//     // 先尝试清空一次非IPD数据，例如OK, ERROR等残留
//     uint32_t pre_clean_timeout = 100; // 短暂超时清理
//     uint32_t pre_clean_start = g_rtos_ops->GetTickCount();
//     while((g_rtos_ops->GetTickCount() - pre_clean_start) < pdMS_TO_TICKS(pre_clean_timeout) && RingBuffer_IsAvailable(&g_esp32_dev.serial_dev->rx_buffer)){
//         uint8_t temp_byte;
//         RingBuffer_Read(&g_esp32_dev.serial_dev->rx_buffer, &temp_byte);
//     }


//     while ((g_rtos_ops->GetTickCount() - start_tick) < pdMS_TO_TICKS(timeout_ms)) {
//         uint8_t byte;
//         if (RingBuffer_IsAvailable(&g_esp32_dev.serial_dev->rx_buffer)) {
//             RingBuffer_Read(&g_esp32_dev.serial_dev->rx_buffer, &byte);

//             if (parsing_ipd_header == 1) { // 等待 '+IPD,'
//                 if (ipd_header_idx < sizeof(ipd_prefix) - 1) {
//                     ipd_prefix[ipd_header_idx++] = byte;
//                     ipd_prefix[ipd_header_idx] = '\0';
//                     if (strstr(ipd_prefix, "+IPD,") == ipd_prefix) {
//                         parsing_ipd_header = 2; // 开始解析长度
//                         ipd_header_idx = 0; // 重置索引用于长度字符串
//                         memset(ipd_prefix, 0, sizeof(ipd_prefix)); // 清空用于存长度
//                     } else if (ipd_header_idx > 5 && strstr(ipd_prefix, "+IPD,") == NULL) { // 不是+IPD的开头，重置
//                         ipd_header_idx = 0;
//                         memset(ipd_prefix, 0, sizeof(ipd_prefix));
//                          if (byte == '+') { // 可能是新的+IPD开头
//                             ipd_prefix[ipd_header_idx++] = byte;
//                          }
//                     }
//                 } else { // +IPD, 头太长或未匹配，重置
//                     ipd_header_idx = 0;
//                     memset(ipd_prefix, 0, sizeof(ipd_prefix));
//                 }
//             } else if (parsing_ipd_header == 2) { // 解析长度直到 ':'
//                 if (byte == ':') {
//                     ipd_data_len = atoi(ipd_prefix);
//                     if (ipd_data_len > 0 && ipd_data_len <= max_len) {
//                         parsing_ipd_header = 0; // 开始接收数据
//                         received_len = 0; // 重置已接收数据长度计数器
//                         Log_Message(LOG_LEVEL_DEBUG, "[WiFi TCP Recv] IPD: expect %d bytes", ipd_data_len);
//                     } else { // 长度无效或超出缓冲区
//                         Log_Message(LOG_LEVEL_WARNING, "[WiFi TCP Recv] IPD: invalid len %d (max %d)", ipd_data_len, max_len);
//                         parsing_ipd_header = 1; // 回到等待+IPD状态
//                         ipd_header_idx = 0;
//                         memset(ipd_prefix, 0, sizeof(ipd_prefix));
//                         ipd_data_len = 0;
//                     }
//                 } else if (byte >= '0' && byte <= '9' && ipd_header_idx < 6) { // 长度数字部分
//                     ipd_prefix[ipd_header_idx++] = byte;
//                     ipd_prefix[ipd_header_idx] = '\0';
//                 } else { // 长度解析中遇到非数字或冒号，或长度字符串过长
//                     Log_Message(LOG_LEVEL_WARNING, "[WiFi TCP Recv] IPD: error parsing len near '%s', char '%c'", ipd_prefix, byte);
//                     parsing_ipd_header = 1; // 回到等待+IPD状态
//                     ipd_header_idx = 0;
//                     memset(ipd_prefix, 0, sizeof(ipd_prefix));
//                     ipd_data_len = 0;
//                 }
//             } else { // parsing_ipd_header == 0, 接收实际数据
//                 if (received_len < ipd_data_len && received_len < max_len) {
//                     buffer[received_len++] = byte;
//                     if (received_len == ipd_data_len) { // 当前IPD包接收完毕
//                         *length = received_len;
//                         Log_Message(LOG_LEVEL_INFO, "[WiFi TCP Recv] Received %d bytes from IPD.", received_len);
//                         g_rtos_ops->SemaphoreGive(g_esp32_dev.mutex);
//                         return AT_ERR_NONE;
//                     }
//                 }
//             }
//         }
//         if (g_rtos_ops->Delay) g_rtos_ops->Delay(1);
//     }

//     *length = received_len; // 返回部分接收的数据（如果有）
//     g_rtos_ops->SemaphoreGive(g_esp32_dev.mutex);

//     if (received_len > 0 && parsing_ipd_header == 0 && received_len < ipd_data_len) {
//          Log_Message(LOG_LEVEL_WARNING, "[WiFi TCP Recv] Timeout, but received %d/%d bytes of IPD.", received_len, ipd_data_len);
//         return AT_ERR_NONE; // 返回部分数据
//     } else if (parsing_ipd_header != 0) {
//         Log_Message(LOG_LEVEL_WARNING, "[WiFi TCP Recv] Timeout waiting for or parsing +IPD header. State: %d, Buffer: '%s'", parsing_ipd_header, ipd_prefix);
//         return AT_ERR_TIMEOUT;
//     }
//     Log_Message(LOG_LEVEL_WARNING, "[WiFi TCP Recv] Timeout, no complete IPD data received.");
//     return AT_ERR_TIMEOUT;
// }


/**
 * @brief 发送 BLE 数据
 */
AT_Error_Code BLE_SendData(const uint8_t *data, uint16_t length)
{
    // 与 WiFi_SendTCPData 类似，也需要先发送准备指令如 "AT+BLESEND=<length>\r\n"
    // 然后等待 ">"，再发送数据，最后等待 "SEND OK"
    // 具体指令需要查阅ESP32 BLE AT指令手册

    if (!g_rtos_ops || !g_esp32_dev.serial_dev || !g_esp32_dev.mutex || !data || length == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[BLE Send] Invalid parameters");
        return AT_ERR_SEND_FAILED;
    }
     if (length > TCP_BUFFER_SIZE) {
        Log_Message(LOG_LEVEL_ERROR, "[BLE Send] Data length %d exceeds max buffer %d", length, TCP_BUFFER_SIZE);
        return AT_ERR_SEND_FAILED;
    }

    if (!g_rtos_ops->SemaphoreTake(g_esp32_dev.mutex, 0xFFFFFFFF)) {
        Log_Message(LOG_LEVEL_ERROR, "[BLE Send] Failed to take ESP32 mutex");
        return AT_ERR_TIMEOUT;
    }

    AT_Error_Code status = AT_ERR_SEND_FAILED;
    char cmd_str[32];
    // 假设BLE发送指令为 AT+BLUESPPDAT=<length> (或类似，需确认)
    // 或者 AT+BLEGATTSNTFY (GATT Server Notify)
    // 示例使用一个通用的 BLESEND
    snprintf(cmd_str, sizeof(cmd_str), "AT+BLESEND=%d\r\n", length);

    AT_Cmd_Config prep_cmd = {cmd_str, ">", 2000, 0, "BLE Send Prep"};

    status = _ESP32_SendATCommand_nolock(&prep_cmd, "BLE");
    if (status != AT_ERR_NONE) {
        Log_Message(LOG_LEVEL_ERROR, "[BLE Send] Failed to get '>' prompt. Status: %d", status);
        g_rtos_ops->SemaphoreGive(g_esp32_dev.mutex);
        return status;
    }

    if (Serial_Driver_SendData(g_esp32_dev.serial_dev, data, length) != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[BLE Send] Failed to send data payload.");
        g_rtos_ops->SemaphoreGive(g_esp32_dev.mutex);
        return AT_ERR_SEND_FAILED;
    }

    AT_Cmd_Config confirm_cmd = {"", "SEND OK", 5000, 0, "BLE Send Confirm"};
    status = _ESP32_SendATCommand_nolock(&confirm_cmd, "BLE");

    if (status != AT_ERR_NONE) {
        Log_Message(LOG_LEVEL_WARNING, "[BLE Send] Did not receive 'SEND OK'. Status: %d.", status);
    } else {
        Log_Message(LOG_LEVEL_INFO, "[BLE Send] Sent %d bytes successfully.", length);
    }

    g_rtos_ops->SemaphoreGive(g_esp32_dev.mutex);
    return status;
}

/**
 * @brief 接收 BLE 数据
 */
AT_Error_Code BLE_ReceiveData(uint8_t *buffer, uint16_t *length, uint32_t timeout_ms)
{
    // BLE数据接收可能通过 "+BLERECV=<len>:<data>" 或类似AT主动上报，或GATT读响应
    // 此处简化，类似WiFi TCP接收，但BLE通常没有明确的+IPD，可能需要不同的解析逻辑
    // 或依赖于透传模式。

    if (!g_rtos_ops || !g_esp32_dev.serial_dev || !g_esp32_dev.mutex || !buffer || !length || *length == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[BLE Recv] Invalid parameters");
        return AT_ERR_SEND_FAILED;
    }

    if (!g_rtos_ops->SemaphoreTake(g_esp32_dev.mutex, 0xFFFFFFFF)) {
        Log_Message(LOG_LEVEL_ERROR, "[BLE Recv] Failed to take ESP32 mutex");
        return AT_ERR_TIMEOUT;
    }

    uint16_t received_len = 0;
    uint16_t max_len = *length;
    uint32_t start_tick = g_rtos_ops->GetTickCount();

    // 简单实现：读取直到超时或缓冲区满
    while ((g_rtos_ops->GetTickCount() - start_tick) < pdMS_TO_TICKS(timeout_ms)) {
        uint8_t byte;
        if (RingBuffer_IsAvailable(&g_esp32_dev.serial_dev->rx_buffer)) {
            RingBuffer_Read(&g_esp32_dev.serial_dev->rx_buffer, &byte);
            if (received_len < max_len) {
                buffer[received_len++] = byte;
            } else {
                Log_Message(LOG_LEVEL_WARNING, "[BLE Recv] Buffer full (%d bytes).", max_len);
                break;
            }
        }
        if (received_len == max_len) break; // 已达到用户期望长度
        if (g_rtos_ops->Delay) g_rtos_ops->Delay(1);
    }

    *length = received_len;
    g_rtos_ops->SemaphoreGive(g_esp32_dev.mutex);

    if (received_len > 0) {
        Log_Message(LOG_LEVEL_INFO, "[BLE Recv] Received %d bytes.", received_len);
        return AT_ERR_NONE;
    }
    Log_Message(LOG_LEVEL_WARNING, "[BLE Recv] Timeout.");
    return AT_ERR_TIMEOUT;
}
