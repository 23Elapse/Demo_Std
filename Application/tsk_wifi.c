#include "tsk_wifi.h"

static uint8_t wifi_app_initialized = 0; // WiFi应用层初始化标志
static uint8_t ble_app_initialized = 0;  // BLE应用层初始化标志
static uint8_t tcp_app_initialized = 0;  // TCP应用层初始化标志
static uint32_t msg_counter = 0;
static uint8_t wifi_send_buf[5000]; // 共享缓冲区

// 初始化相关状态变量
static uint8_t wifi_stage_idx = 0;
static uint8_t wifi_init_done = 0;

// 定时发送控制变量（单位ms）
static uint32_t realtime_data_last_send = 0;
static uint32_t offline_data_last_send = 0;

// 发送周期（单位ms）
#define REALTIME_DATA_INTERVAL_MS  10000
#define OFFLINE_DATA_INTERVAL_MS   2000

// 定义阶段表（设备树、属性树、时间同步）
const InitStageDynamic_t wifi_init_stages[] = {
    { INIT_STAGE_DEVICE_TREE, "[ACK] DEVICE_TREE_OK" },
    { INIT_STAGE_PROP_TREE,   "[ACK] PROP_TREE_OK" },
    { INIT_STAGE_TIME_SYNC,   "[ACK] TIME_SYNC_OK" },
    { 0xFF, NULL } // 结束标记
};

// 获取系统毫秒时间（请根据实际平台修改）
extern uint32_t GetSysTickMs(void);


void Process_Command(Comm_Channel_t channel, const char *cmd_str)
{
    const char *channel_tag = (channel == COMM_TCP) ? "TCP" : "BLE";
    ESP32_Comm_Type_t esp32_comm_type = (channel == COMM_TCP) ? ESP32_COMM_TYPE_WIFI : ESP32_COMM_TYPE_BLE;

    if (strncmp(cmd_str, "CMD:", 4) != 0) {
        Log_Message(LOG_LEVEL_WARNING, "[CMD/%s] Invalid prefix: %s", channel_tag, cmd_str);
        const char *ack = "[ACK] ERR: Invalid Prefix\r\n";
        g_esp32_at_ops.SendData(&g_esp32_dev, (uint8_t*)ack, strlen(ack), esp32_comm_type);
        return;
    }

    const char *cmd = cmd_str + 4;

    // 执行命令并生成 ACK
    char ack_buf[64] = {0};

    if (strcmp(cmd, "LED_ON") == 0) {
        Log_Message(LOG_LEVEL_INFO, "[CMD/%s] LED ON triggered.", channel_tag);
        // TODO: 实际控制 LED
        snprintf(ack_buf, sizeof(ack_buf), "[%s_ACK] OK: LED ON\r\n", channel_tag);
    }
    else if (strcmp(cmd, "LED_OFF") == 0) {
        Log_Message(LOG_LEVEL_INFO, "[CMD/%s] LED OFF triggered.", channel_tag);
        // TODO: 实际控制 LED
        snprintf(ack_buf, sizeof(ack_buf), "[%s_ACK] OK: LED OFF\r\n", channel_tag);
    }
    else if (strcmp(cmd, "GET_STATUS") == 0) {
        // 模拟状态返回
        Log_Message(LOG_LEVEL_INFO, "[CMD/%s] Status requested.", channel_tag);
        snprintf(ack_buf, sizeof(ack_buf), "[%s_ACK] OK: STATUS=NORMAL\r\n", channel_tag);
    }
    else {
        Log_Message(LOG_LEVEL_WARNING, "[CMD/%s] Unknown command: %s", channel_tag, cmd);
        snprintf(ack_buf, sizeof(ack_buf), "[%s_ACK] ERR: Unknown Command\r\n", channel_tag);
    }

    // 发送 ACK 回通道
    g_esp32_at_ops.SendData(&g_esp32_dev, (uint8_t*)ack_buf, strlen(ack_buf), esp32_comm_type);
}


void Handle_Comm_Channel(Comm_Channel_t channel, uint32_t interval_ms)
{
    static uint32_t last_send_tick_tcp = 0;
    static uint32_t last_send_tick_ble = 0;

    const char *tag = (channel == COMM_TCP) ? "TCP" : "BLE";
    ESP32_Comm_Type_t esp32_comm_type = (channel == COMM_TCP) ? ESP32_COMM_TYPE_WIFI : ESP32_COMM_TYPE_BLE;
    uint8_t is_initialized = (channel == COMM_TCP) ? (wifi_app_initialized && tcp_app_initialized) : ble_app_initialized;

    if (!is_initialized) return;

    // --- 接收 ---
    uint8_t rx_buffer[TCP_BUFFER_SIZE];
    uint16_t rx_len = sizeof(rx_buffer);
    if (g_esp32_at_ops.ReceiveData(&g_esp32_dev, rx_buffer, &rx_len, 500, esp32_comm_type) == AT_OK && rx_len > 0) {
        rx_buffer[rx_len] = '\0';
        Log_Message(LOG_LEVEL_INFO, "[%s] Received: %s", tag, rx_buffer);
        Process_Command(channel, (char*)rx_buffer);
    }

    // --- 发送 ---
    uint32_t *last_tick = (channel == COMM_TCP) ? &last_send_tick_tcp : &last_send_tick_ble;
    if (g_rtos_ops->GetTickCount() - *last_tick > interval_ms) {
        char tx_msg[64];
        snprintf(tx_msg, sizeof(tx_msg), "Hello %s #%lu from STM32!", tag, msg_counter++);

        if (g_esp32_at_ops.SendData(&g_esp32_dev, (uint8_t*)tx_msg, strlen(tx_msg), esp32_comm_type) == AT_OK) {
            Log_Message(LOG_LEVEL_INFO, "[%s] Sent: %s", tag, tx_msg);
        } else {
            Log_Message(LOG_LEVEL_ERROR, "[%s] Send failed.", tag);
        }
        *last_tick = g_rtos_ops->GetTickCount();
    }
}
// 准备实时数据发送内容
static uint16_t PrepareRealtimeData(uint8_t *buf, uint16_t max_len)
{
    // 这里做实时数据格式拼接，比如温度，湿度等
    return snprintf((char*)buf, max_len, "{ \"temp\": 25, \"hum\": 50 }");
}

// 准备离线数据发送内容
static uint16_t PrepareOfflineData(uint8_t *buf, uint16_t max_len)
{
    // 离线数据通常批量或历史数据
    return snprintf((char*)buf, max_len, "{ \"offline\": [ {\"temp\":24}, {\"temp\":23} ] }");
}

void Wifi_PeriodicSendHandler(ESP32_Comm_Type_t comm_type)
{
    uint32_t now = g_rtos_ops->GetTickCount();

    // 仅当初始化完成后才发送实时和离线数据
    if (!wifi_init_done) return;

    // 实时数据定时发送
    if (now - realtime_data_last_send >= REALTIME_DATA_INTERVAL_MS) {
        uint16_t len = PrepareRealtimeData(wifi_send_buf, sizeof(wifi_send_buf));
        if (g_esp32_at_ops.SendData(&g_esp32_dev, wifi_send_buf, len, comm_type) == 0 /*AT_OK*/) {
            Log_Message(LOG_LEVEL_INFO, "[WIFI] Realtime data sent.");
            realtime_data_last_send = now;
        } else {
            Log_Message(LOG_LEVEL_WARNING, "[WIFI] Failed to send realtime data.");
        }
    }

    // 离线数据定时发送
    if (now - offline_data_last_send >= OFFLINE_DATA_INTERVAL_MS) {
        uint16_t len = PrepareOfflineData(wifi_send_buf, sizeof(wifi_send_buf));
        if (g_esp32_at_ops.SendData(&g_esp32_dev, wifi_send_buf, len, comm_type) == 0 /*AT_OK*/) {
            Log_Message(LOG_LEVEL_INFO, "[WIFI] Offline data sent.");
            offline_data_last_send = now;
        } else {
            Log_Message(LOG_LEVEL_WARNING, "[WIFI] Failed to send offline data.");
        }
    }
}

uint16_t Prepare_Init_Stage_Data(uint8_t stage_index, uint8_t *out_buf, uint16_t max_len)
{
    switch ((InitStageIndex_t)stage_index) {
        case INIT_STAGE_DEVICE_TREE:
            return snprintf((char*)out_buf, max_len, 
                "{ \"device\": \"STM32\", \"module\": \"ESP32\" }");
        case INIT_STAGE_PROP_TREE:
            return snprintf((char*)out_buf, max_len, 
                "{ \"temp\": \"int\", \"hum\": \"int\" }");
        case INIT_STAGE_TIME_SYNC:
            return snprintf((char*)out_buf, max_len, 
                "{ \"sync_time\": true }");
        default:
            return 0;
    }
}

void ServerCommProcess(uint8_t *buf, uint16_t buf_size,
                       const InitStageDynamic_t *stages,
                       uint8_t *current_stage,
                       uint8_t *is_done,
                       ESP32_Comm_Type_t comm_type,
                       const char *tag)
{
    if (*is_done) return;

    const InitStageDynamic_t *stage = &stages[*current_stage];
    if (stage->ack_keyword == NULL || stage->stage_index == 0xFF) {
        *is_done = 1;
        Log_Message(LOG_LEVEL_INFO, "[%s] Multi-stage init DONE.", tag);
        return;
    }

    uint16_t data_len = Prepare_Init_Stage_Data(stage->stage_index, buf, buf_size);
    if (data_len == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[%s] Failed to prepare data for stage %d.", tag, *current_stage);
        return;
    }

    if (g_esp32_at_ops.SendData(&g_esp32_dev, buf, data_len, comm_type) == 0 /*AT_OK*/) {
        Log_Message(LOG_LEVEL_INFO, "[%s] Sent stage %d (%u bytes), waiting ACK: %s", 
                    tag, *current_stage, data_len, stage->ack_keyword);
    } else {
        Log_Message(LOG_LEVEL_WARNING, "[%s] Failed to send stage %d", tag, *current_stage);
    }
}

void ServerCommProcess_HandleAck(const InitStageDynamic_t *stages,
                                uint8_t *current_stage,
                                uint8_t *is_done,
                                const char *recv_str,
                                const char *tag)
{
    if (*is_done) return;

    const InitStageDynamic_t *stage = &stages[*current_stage];
    if (strstr(recv_str, stage->ack_keyword)) {
        Log_Message(LOG_LEVEL_INFO, "[%s] ACK matched for stage %d: %s", tag, *current_stage, stage->ack_keyword);
        (*current_stage)++;
    }
}

void WifiTaskLoop(void)
{
    // 先做多阶段初始化发送
    ServerCommProcess(wifi_send_buf, sizeof(wifi_send_buf),
                      wifi_init_stages,
                      &wifi_stage_idx,
                      &wifi_init_done,
                      ESP32_COMM_TYPE_WIFI,
                      "WIFI");

    // 处理接收ACK，调用时机由接收数据事件触发
    /*
    if (recv_data_available) {
        ServerCommProcess_HandleAck(wifi_init_stages, &wifi_stage_idx, &wifi_init_done, (char*)recv_buffer, "WIFI");
    }
    */
    // 初始化完成后执行周期性发送
    Wifi_PeriodicSendHandler(ESP32_COMM_TYPE_WIFI);
}

/**
 * @brief WiFi/BLE 统一管理任务
 * @param pvParameters 未使用
 */
void App_WifiBLETask(void *pvParameters)
{
    (void)pvParameters; // 避免编译器警告
    Log_Message(LOG_LEVEL_TEST, "[WiFi/BLE Task] Started.");

    uint8_t esp32_ready = 0;
    uint8_t ready_retry_count = 0;
    const uint8_t max_ready_retries = 5;

    // --- 1. ESP32 模块就绪检查 ---
    while (!esp32_ready && ready_retry_count < max_ready_retries) {
        ESP32_Hw_Reset(&g_esp32_dev); // 硬件复位ESP32
        Log_Message(LOG_LEVEL_TEST, "[WiFi/BLE] ESP32 Reset, waiting for boot up (attempt %u/%u)...", ready_retry_count + 1, max_ready_retries);
        g_rtos_ops->Delay(300); // 等待ESP32启动

        // 发送基础AT指令测试模块是否响应
        AT_Cmd_Config_t at_test_cmd = {"AT\r\n", "OK", 2000, 2, "ESP32 Ready Test"};
        if (g_esp32_at_ops.SendATCommand(&g_esp32_dev, &at_test_cmd, ESP32_COMM_TYPE_WIFI) == AT_OK) {
            Log_Message(LOG_LEVEL_TEST, "[WiFi/BLE] ESP32 is ready.");
            esp32_ready = 1;
        } else {
            Log_Message(LOG_LEVEL_WARNING, "[WiFi/BLE] ESP32 not ready. Retrying...");
            ready_retry_count++;
            if (ready_retry_count < max_ready_retries) g_rtos_ops->Delay(2000); // 重试前额外延时
        }
    }

    if (!esp32_ready) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi/BLE] ESP32 failed to become ready after %u attempts. Suspending task.", max_ready_retries);
        if (g_rtos_ops->Task_Suspend) g_rtos_ops->Task_Suspend(NULL); else for(;;); // 挂起任务
        return;
    }


    uint8_t app_init_retry_count = 0;
    uint8_t tcp_init_retry_count = 0;
    const uint8_t max_app_init_retries = 3;

    while (1) {
        // --- 2. WiFi 应用层初始化 ---
        if (!wifi_app_initialized) {
            Log_Message(LOG_LEVEL_TEST, "[WiFi] Attempting application layer initialization...");
            AT_Cmd_Config_t init_cmds[] = {
                {"ATE1\r\n", "OK", 2000, 2, "Echo Test",NULL},
                {"AT+GMR\r\n", "OK", 2000, 2, "Get Version",get_version_callback}, // 获取版本信息
                {"AT+CWMODE=1\r\n", "OK", 2000, 2, "Set Station Mode",NULL},
                {"AT+CWJAP=\"" WIFI_SSID "\",\"" WIFI_PASSWORD "\"\r\n", "OK", 15000, 3, "Connect to WiFi AP",set_ssid_password_callback},
                {"AT+CIPSTAMAC?\r\n", "OK", 2000, 2, "Get Station MAC",get_mac_addr_callback},
                {"AT+CIPSTA?\r\n", "OK", 2000, 2, "Get Station IP",get_ip_addr_callback},
                // Add more WiFi specific init commands here if needed
                {NULL, NULL, 0, 0, NULL} // 哨兵值
            };
            uint8_t init_success = 1;
            for (const AT_Cmd_Config_t *cmd = init_cmds; cmd->at_cmd != NULL; cmd++) {
                if (g_esp32_at_ops.SendATCommand(&g_esp32_dev, cmd, ESP32_COMM_TYPE_WIFI) != AT_OK) {
                    init_success = 0;
                    break;
                }
                g_rtos_ops->Delay(200); // 命令间延时
            }

            if (init_success) {
                wifi_app_initialized = 1;
                app_init_retry_count = 0;
                Log_Message(LOG_LEVEL_INFO, "[WiFi] Application Layer Initialization successful.");
            } else {
                app_init_retry_count++;
                Log_Message(LOG_LEVEL_WARNING, "[WiFi] App Layer Init failed, retry %u/%u.", app_init_retry_count, max_app_init_retries);
                if (app_init_retry_count >= max_app_init_retries) {
                    Log_Message(LOG_LEVEL_ERROR, "[WiFi] Max App Layer Init retries. Triggering ESP32 full reset cycle.");
                    esp32_ready = 0; // 标记ESP32需要重新检查就绪状态
                    ready_retry_count = 0; // 重置ESP32就绪检查计数
                    wifi_app_initialized = 0; // 确保下次循环重新初始化应用层
                    g_rtos_ops->Delay(5000); // 长延时后从头开始
                    continue; // 返回到外层while，重新检查esp32_ready
                }
                g_rtos_ops->Delay(3000); // 应用初始化失败后的短延时重试
            }
        }
        // --- 3. BLE 应用层初始化 ---
        if (!ble_app_initialized) {
             Log_Message(LOG_LEVEL_TEST, "[BLE] Attempting application layer initialization...");
             AT_Cmd_Config_t ble_init_cmds[] = {
                {"AT+BLEINIT=2\r\n", "OK", 2000, 2, "Initialize BLE Peripheral"}, // 示例指令
                {"AT+BLEADVSTART\r\n", "OK", 2000, 3, "Start BLE Advertising"}, // 示例指令
                {NULL, NULL, 0, 0, NULL}
            };
            uint8_t ble_init_success = 1;
            for (const AT_Cmd_Config_t *cmd = ble_init_cmds; cmd->at_cmd != NULL; cmd++) {
                if (g_esp32_at_ops.SendATCommand(&g_esp32_dev, cmd, ESP32_COMM_TYPE_BLE) != AT_OK) {
                    ble_init_success = 0;
                    break;
                }
                g_rtos_ops->Delay(200);
            }

            if (ble_init_success) {
                ble_app_initialized = 1;
                Log_Message(LOG_LEVEL_TEST, "[BLE] Application Layer Initialization successful.");
            } else {
                Log_Message(LOG_LEVEL_WARNING, "[BLE] App Layer Init failed. Retrying in next cycle.");
                // 对于BLE，失败后暂时不强制ESP32重置，而是等待下一次循环尝试
            }
        }
        // --- 4. TCP 应用层初始化 ---
        if (!tcp_app_initialized) {
            Log_Message(LOG_LEVEL_TEST, "[WiFi] TCP Application Layer not initialized yet. Initializing...");
            // 这里可以添加TCP应用层初始化逻辑，例如连接到服务器等
            static char cmd_str[64];
            // snprintf(cmd_str, sizeof(cmd_str), "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", TCP_SERVER_IP, TCP_PORT);
            snprintf(cmd_str, sizeof(cmd_str), "AT+ATKCLDSTA=\"%s\",\"%s\"\r\n", AKT_DEV_ID, AKT_DEV_SECRET);
            AT_Cmd_Config_t tcp_init_cmds[] = {
                {cmd_str, "CLOUD CONNECTED", 10000, 1, "Connect to TCP Server",NULL},
                {NULL, NULL, 0, 0, NULL,NULL} // 哨兵值
            };
            uint8_t tcp_init_success = 1;
            for (const AT_Cmd_Config_t *cmd = tcp_init_cmds; cmd->at_cmd != NULL; cmd++) {
                if (g_esp32_at_ops.SendATCommand(&g_esp32_dev, cmd, ESP32_COMM_TYPE_WIFI) != AT_OK) {
                    tcp_init_success = 0;
                    break;
                }
                g_rtos_ops->Delay(200); // 命令间延时
            }

            if (tcp_init_success) {
                tcp_app_initialized = 1;
                tcp_init_retry_count = 0;
                Log_Message(LOG_LEVEL_TEST, "[WiFi] TCP Application Layer Initialization successful.");
            } else {
                tcp_init_retry_count++;
                Log_Message(LOG_LEVEL_WARNING, "[WiFi] TCP App Layer Init failed, retry %u/%u.", tcp_init_retry_count, max_app_init_retries);
                if (tcp_init_retry_count >= max_app_init_retries) {
                    Log_Message(LOG_LEVEL_ERROR, "[WiFi] Max TCP App Layer Init retries. Triggering ESP32 full reset cycle.");
                    tcp_app_initialized = 0; // 确保下次循环重新初始化应用层
                    g_rtos_ops->Delay(5000); // 长延时后从头开始
                    continue; // 返回到外层while，重新检查esp32_ready
                }
                g_rtos_ops->Delay(3000); // 应用初始化失败后的短延时重试
            }
        }
        
        WifiTaskLoop(); 
        // --- 5. WiFi/BLE 定期功能测试 ---
        if (wifi_app_initialized && tcp_app_initialized) {
            // char cmd_str[64];
            // snprintf(cmd_str, sizeof(cmd_str), "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", TCP_SERVER_IP, TCP_PORT);
            // AT_Cmd_Config_t tcp_connect_cmd = {cmd_str, "OK", 10000, 1, "Connect to TCP Server"};

            // if (g_esp32_at_ops.SendATCommand(&g_esp32_dev, &tcp_connect_cmd, ESP32_COMM_TYPE_WIFI) == AT_OK) {
            {
                uint8_t tx_data[] = "Hello Server from STM32 WiFi!";
                uint8_t rx_buffer[TCP_BUFFER_SIZE];
                uint16_t rx_len = sizeof(rx_buffer); // 输入期望最大长度

                // 发送TCP数据
                if (g_esp32_at_ops.SendData(&g_esp32_dev, tx_data, strlen((char*)tx_data), ESP32_COMM_TYPE_WIFI) == AT_OK) {
                    // 接收TCP数据
                    if (g_esp32_at_ops.ReceiveData(&g_esp32_dev, rx_buffer, &rx_len, 5000, ESP32_COMM_TYPE_WIFI) == AT_OK && rx_len > 0) {
                        rx_buffer[rx_len] = '\0'; // 确保字符串终止
                        Log_Message(LOG_LEVEL_TEST, "[WiFi] TCP Recv: %s", rx_buffer);
                    } else {
                        Log_Message(LOG_LEVEL_WARNING, "[WiFi] No TCP data received or receive error.");
                    }
                } else {
                    Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send TCP data.");
                }

                // AT_Cmd_Config_t close_cmd = {"AT+CIPCLOSE\r\n", "OK", 2000, 0, "Disconnect TCP"};
                // g_esp32_at_ops.SendATCommand(&g_esp32_dev, &close_cmd, ESP32_COMM_TYPE_WIFI);
            } 
            // else {
            //     Log_Message(LOG_LEVEL_WARNING, "[WiFi] Failed to connect TCP. Checking AP connection...");
            //     AT_Cmd_Config_t check_ap_cmd = {"AT+CWJAP?\r\n", WIFI_SSID, 3000, 0, "Check AP"};
            //     if (g_esp32_at_ops.SendATCommand(&g_esp32_dev, &check_ap_cmd, ESP32_COMM_TYPE_WIFI) != AT_OK) {
            //         Log_Message(LOG_LEVEL_WARNING, "[WiFi] AP connection lost. Triggering app layer re-init.");
            //         wifi_app_initialized = 0; // 触发应用层重初始化
            //     }
            // }
        }

        if (ble_app_initialized) {
            uint8_t ble_tx_data[] = "Hello via BLE from STM32!";
            uint8_t ble_rx_buffer[TCP_BUFFER_SIZE];
            uint16_t ble_rx_len = sizeof(ble_rx_buffer);

            if (g_esp32_at_ops.SendData(&g_esp32_dev, ble_tx_data, strlen((char*)ble_tx_data), ESP32_COMM_TYPE_BLE) == AT_OK) {
                if (g_esp32_at_ops.ReceiveData(&g_esp32_dev, ble_rx_buffer, &ble_rx_len, 5000, ESP32_COMM_TYPE_BLE) == AT_OK && ble_rx_len > 0) {
                    ble_rx_buffer[ble_rx_len] = '\0';
                    Log_Message(LOG_LEVEL_TEST, "[BLE] Recv: %s", ble_rx_buffer);
                } else {
                    Log_Message(LOG_LEVEL_WARNING, "[BLE] No BLE data received or receive error.");
                }
            } else {
                Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to send BLE data.");
                // ble_app_initialized = 0; // 可选：发送失败则重新初始化应用
            }
        }

        g_rtos_ops->Delay(10000); // 主循环延时 (10秒)
    }

}
