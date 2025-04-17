/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-01 20:50:17
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-04-16 00:25:55
 * @FilePath: \Demo\Middlewares\Src\api_wifi.c
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
#include "api_wifi.h"

// AT 指令配置表
static const AT_Cmd_Config at_cmd_table[] = {
    {"AT\r\n", "OK", 1000, 3, "AT command test"},          // 测试连接
    {"AT+GMR\r\n", "OK", 2000, 2, "Check the firmware version"},     // 查询固件版本

    {"AT+CWMODE=1\r\n", "OK", 2000, 2, "Setting Station Mode"}, // 设置 Station 模式
    {"AT+CWJAP=\"WIFI_SSID\",\"WIFI_PASSWORD\"\r\n", "OK", 10000, 3, "Connecting to a Router"}, // 连接 Wi-Fi
    {"AT+CIPSTA?\r\n", "+CIPSTA:", 2000, 1, "Query IP address"}, // 查询 IP 地址
    {"AT+CWJAP?\r\n", "OK", 2000, 2, "Query connection status"}, // 查询连接状态
    {"AT+CWJAP_CUR?\r\n", "+CWJAP_CUR:", 3000, 2, "Query current WiFi"}, // 查询当前 Wi-Fi 信息
    {"AT+CWLAP\r\n", "+CWLAP:", 10000, 1, "Scan WiFi networks"}, // 扫描 Wi-Fi 网络
    {"AT+CWLAPOPT=1\r\n", "OK", 2000, 2, "Set scan options"}, // 设置扫描选项
    {"AT+CWLAPOPT?\r\n", "+CWLAPOPT:", 2000, 2, "Query scan options"}, // 查询扫描选项
    {"AT+CIPSTA_CUR?\r\n", "+CIPSTA_CUR:", 2000, 2, "Query current IP address"}, // 查询当前 IP 地址
    {"AT+CIPSTAMAC_CUR?\r\n", "+CIPSTAMAC_CUR:", 2000, 2, "Query current MAC address"}, // 查询当前 MAC 地址
    {"AT+CIPSTO=10\r\n", "OK", 2000, 2, "Set timeout"}, // 设置超时时间

    {"AT+CIPSTART=\"TCP\",\"api.example.com\",80\r\n", "CONNECT", 5000, 3, "建立TCP连接"}, // 建立 TCP 连接
    {"AT+CIPSEND\r\n", ">", 2000, 2, "准备发送数据"},   // 准备发送数据

    {"AT+CIPCLOSE\r\n", "CLOSED", 3000, 2, "关闭TCP连接"}, // 关闭 TCP 连接
    {"AT+CWQAP\r\n", "OK", 2000, 2, "断开WiFi"},       // 断开 Wi-Fi
    {"AT+RST\r\n", "ready", 5000, 1, "重启模块"},     // 重启模块

    {NULL, NULL, 0, 0} // 结束标记
};

// 发送 AT 指令并等待响应
static BaseType_t send_at_command(const AT_Cmd_Config* cmd) {
    uint8_t retry = 0;
    uint8_t rx_buffer[256];
    uint16_t rx_len = 0;

    while (retry <= cmd->retries) {
        // 发送指令
        SendData(&usart_instances[2], (uint8_t*)cmd->at_cmd, strlen(cmd->at_cmd));
        
        // 等待响应
        uint32_t start_tick = xTaskGetTickCount();
        while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(cmd->timeout_ms)) {
            // 读取接收缓冲区
            while (usart_instances[2].rx_ring.tail != usart_instances[2].rx_ring.head) {
                rx_buffer[rx_len++] = usart_instances[2].rx_ring.buffer[usart_instances[2].rx_ring.tail];
                usart_instances[2].rx_ring.tail = (usart_instances[2].rx_ring.tail + 1) % RX_BUFFER_SIZE;
                if (rx_len >= sizeof(rx_buffer)-1) break;
            }
            rx_buffer[rx_len] = '\0';

            // 检查是否包含预期响应
            if (strstr((char*)rx_buffer, cmd->expected_resp) != NULL) {
                printf("Received expected response: %s\r\n", cmd->expected_resp);
                printf("Response: %s\r\n", rx_buffer);
                // 清空接收缓冲区
                usart_instances[2].rx_ring.head = usart_instances[2].rx_ring.tail = 0;
                rx_len = 0; // 重置接收长度

                return pdPASS;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        retry++;
    }
    // 超时或未收到预期响应
    printf("Failed to receive expected response: %s\r\n", cmd->expected_resp);
    // 清空接收缓冲区
    usart_instances[2].rx_ring.head = usart_instances[2].rx_ring.tail = 0;
    rx_len = 0; // 重置接收长度
    // 处理错误（如重试或报告）
    // 这里可以添加错误处理逻辑，例如重试次数超过限制后进行重连或报告错误

    return pdFAIL;
}

// Wi-Fi 任务主函数
void vWifiTask(void const *argument) {

    while (1)
    {
        for (const AT_Cmd_Config* cmd = at_cmd_table; cmd->at_cmd != NULL; cmd++) {
            if (send_at_command(cmd) == pdFAIL) {
                printf("Failed to send command: %s\r\n", cmd->at_cmd);
                // 处理错误（如重试或报告）
                // 这里可以添加错误处理逻辑，例如重试次数超过限制后进行重连或报告错误
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(1000 * 60)); // 每隔 1 分钟执行一次
    }

}

