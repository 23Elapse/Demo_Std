#include "tsk_wifi.h"

InitPhaseControl_t init_ctrl;
PeriodicSendControl_t periodic_ctrl;
ESP32Status_t esp32_status = {0};
static AppInitFlags_t app_init_flags = {.all = 0};

// 初始化函数
static inline void InitPhaseControl_Init(InitPhaseControl_t *ctrl, InitStageMask_t mask)
{
    ctrl->mask = mask;
    ctrl->finished_mask = 0;
    ctrl->current_stage = 0;
    ctrl->completed = 0; // 修改：当前阶段未完成
    ctrl->trigger_mask = 0;
}

// 判断当前阶段是否需要发送
static inline int InitPhaseControl_NeedSend(InitPhaseControl_t *ctrl)
{
    uint8_t stage_bit = (1 << ctrl->current_stage);
    // 当前阶段启用且未完成，或者触发了重发
    if ((ctrl->mask & stage_bit) && (!(ctrl->finished_mask & stage_bit) || (ctrl->trigger_mask & stage_bit)))
    {
        return 1;
    }
    return 0;
}

// 收到当前阶段成功响应，**不自动推进到下一阶段**，只标记完成并清除触发标志
static inline void InitPhaseControl_OnResponse(InitPhaseControl_t *ctrl)
{
    InitStageMask_t stage_bit = (1 << ctrl->current_stage);
    ctrl->finished_mask |= stage_bit; // 标记当前阶段完成
    ctrl->trigger_mask &= ~stage_bit; // 清除触发标志

    // 找下一个需要发送的阶段
    for (int i = ctrl->current_stage + 1; i < INIT_INDEX_COUNT; i++)
    {
        uint8_t bit = (1 << i);
        if ((ctrl->mask & bit) && !(ctrl->finished_mask & bit))
        {
            ctrl->current_stage = i;
            ctrl->completed = 0;
            return;
        }
    }

    // 如果没有下一个未完成的启用阶段，则全部完成
    ctrl->completed = ((ctrl->finished_mask & ctrl->mask) == ctrl->mask);
}

// 外部调用，设置触发标志（某阶段需要重新发送），切换当前阶段索引
static inline void InitPhaseControl_TriggerStage(InitPhaseControl_t *ctrl, InitStageMask_t stage)
{
    if ((ctrl->mask & stage))
    {
        ctrl->finished_mask &= ~stage; // 标记该阶段为未完成
        ctrl->trigger_mask |= stage;   // 设置触发标志
        // 切换到触发阶段索引
        for (int i = 0; i < INIT_INDEX_COUNT; i++)
        {
            if ((1 << i) == stage)
            {
                ctrl->current_stage = i;
                ctrl->completed = 0; // 当前阶段未完成
                break;
            }
        }
    }
}

// 初始化周期发送控制
static inline void PeriodicSendControl_Init(PeriodicSendControl_t *ctrl, uint32_t rt_interval_ms, uint32_t hist_interval_ms)
{
    ctrl->last_realtime_tick = 0;
    ctrl->last_history_tick = 0;
    ctrl->realtime_interval_ms = rt_interval_ms;
    ctrl->history_interval_ms = hist_interval_ms;
}

// 发送接口声明（需外部实现）
void SendDeviceTree(void)
{
    // 发送设备树数据的具体实现
    Log_Message(LOG_LEVEL_INFO, "[WIFI] Sending device tree data.");
}
void SendPropTree(void)
{
    // 发送属性树数据的具体实现
    Log_Message(LOG_LEVEL_INFO, "[WIFI] Sending property tree data.");
}
void SendOfflineTree(void)
{
    // 发送离线树数据的具体实现
    Log_Message(LOG_LEVEL_INFO, "[WIFI] Sending offline tree data.");
}
void SendTimeSyncCommand(void)
{
    // 发送时间同步命令的具体实现
    Log_Message(LOG_LEVEL_INFO, "[WIFI] Sending time sync command.");
}
void SendHeartbeatCommand(void)
{
    // 发送心跳命令的具体实现
    Log_Message(LOG_LEVEL_INFO, "[WIFI] Sending heartbeat command.");
}
void SendRealtimeData(void)
{
    // 发送实时数据的具体实现
    Log_Message(LOG_LEVEL_INFO, "[WIFI] Sending realtime data.");
}
void SendHistoryData(void)
{
    // 发送历史数据的具体实现
    Log_Message(LOG_LEVEL_INFO, "[WIFI] Sending history data.");
}

// 模拟接收成功响应（这里直接返回true，模拟成功）
int ReceiveStageResponse(uint8_t stage)
{
    printf("Received response for stage %d\n", stage);
    return 1; // 模拟总是成功
}
// 周期发送检查和执行
static inline void PeriodicSendControl_CheckAndSend(PeriodicSendControl_t *ctrl)
{
    uint32_t now = g_rtos_ops->GetTickCount();
    if (now - ctrl->last_realtime_tick >= ctrl->realtime_interval_ms)
    {
        SendRealtimeData();
        ctrl->last_realtime_tick = now;
    }
    if (now - ctrl->last_history_tick >= ctrl->history_interval_ms)
    {
        SendHistoryData();
        ctrl->last_history_tick = now;
    }
}

// 其他任务调用此函数触发某阶段重发
void ExternalTrigger_ReSendStage(InitStageMask_t stage)
{
    printf("[External Task] Trigger resend stage mask: 0x%02X\n", stage);
    InitPhaseControl_TriggerStage(&init_ctrl, stage);
}

static const InitStageEntry_t init_stage_table[] = {
    {INIT_STAGE_MASK_DEVICE_TREE, INIT_INDEX_DEVICE_TREE, SendDeviceTree},
    {INIT_STAGE_MASK_PROP_TREE, INIT_INDEX_PROP_TREE, SendPropTree},
    {INIT_STAGE_MASK_OFFLINE_TREE, INIT_INDEX_OFFLINE_TREE, SendOfflineTree},
    {INIT_STAGE_MASK_TIME_SYNC, INIT_INDEX_TIME_SYNC, SendTimeSyncCommand},
    {INIT_STAGE_MASK_HEARTBEAT, INIT_INDEX_HEARTBEAT, SendHeartbeatCommand},
};

// 初始化阶段处理入口（查表方式）
void InitPhaseControl_Process(InitPhaseControl_t *ctrl)
{
    for (int i = 0; i < sizeof(init_stage_table) / sizeof(init_stage_table[0]); ++i)
    {
        const InitStageEntry_t *entry = &init_stage_table[i];
        if (entry->stage_index == ctrl->current_stage && InitPhaseControl_NeedSend(ctrl))
        {
            entry->send_func();
            InitPhaseControl_OnResponse(ctrl);
            break; // 每次只处理一个阶段
        }
    }
}

// 主任务循环示例（结合阶段顺序和周期发送）
void WifiTaskLoop(void)
{
    static uint8_t initialized = 0;
    if (!initialized)
    {
        InitPhaseControl_Init(&init_ctrl, INIT_STAGE_MASK_ALL);
        PeriodicSendControl_Init(&periodic_ctrl, REALTIME_DATA_INTERVAL_MS, OFFLINE_DATA_INTERVAL_MS); // 10s实时数据, 2s历史数据
        initialized = 1;
    }
    if (!init_ctrl.completed)
    {
        InitPhaseControl_Process(&init_ctrl);
    }
    else
    {
        // 所有初始化阶段完成后，执行周期性数据发送
        PeriodicSendControl_CheckAndSend(&periodic_ctrl);
    }
}

/**
 * @brief 检查ESP32是否准备就绪
 * @param status ESP32状态结构体指针
 * @param max_retries 最大重试次数
 * @param delay_ms 每次重试间隔（毫秒）
 * @return 0表示就绪，-1表示未就绪
 */
int ESP32_CheckReady(ESP32Status_t *status, int max_retries, uint32_t delay_ms)
{
    while (!status->ready && status->ready_retry_count < max_retries)
    {
        ESP32_Hw_Reset(&g_esp32_dev);
        Log_Message(LOG_LEVEL_TEST, "[WiFi/BLE] ESP32 Reset, waiting for boot up (attempt %u/%u)...",
                    status->ready_retry_count + 1, max_retries);
        g_rtos_ops->Delay(300);

        AT_Cmd_Config_t at_test_cmd = {"AT\r\n", "OK", 2000, 2, "ESP32 Ready Test"};
        if (g_esp32_at_ops.SendATCommand(&g_esp32_dev, &at_test_cmd, ESP32_COMM_TYPE_WIFI) == AT_OK)
        {
            Log_Message(LOG_LEVEL_TEST, "[WiFi/BLE] ESP32 is ready.");
            status->ready = 1;
        }
        else
        {
            Log_Message(LOG_LEVEL_WARNING, "[WiFi/BLE] ESP32 not ready. Retrying...");
            status->ready_retry_count++;
            if (status->ready_retry_count < max_retries)
            {
                g_rtos_ops->Delay(delay_ms);
            }
        }
    }

    if (!status->ready)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi/BLE] ESP32 failed to become ready after %u attempts.", max_retries);
        return -1;
    }
    return 0;
}

/**
 * @brief 重置ESP32并清除状态
 * @param status ESP32状态结构体指针
 */
void ESP32_ResetAndClearStatus(ESP32Status_t *status)
{
    ESP32_Hw_Reset(&g_esp32_dev);
    Log_Message(LOG_LEVEL_INFO, "[ESP32] Hardware reset performed, clearing status.");
    status->ready = 0;
    status->ready_retry_count = 0;
    status->wifi_init_retry_count = 0;
    status->ble_init_retry_count = 0;
    status->tcp_init_retry_count = 0;
    app_init_flags.all = 0; // 清除应用初始化标志
    g_rtos_ops->Delay(5000);
}

/**
 * @brief BLE 应用层初始化
 * @param flags 应用初始化标志结构体指针
 * @note 该函数会尝试初始化BLE并在失败时进行重试
 */
void ble_init(AppInitFlags_t *flags)
{
    if (!flags->bits.ble_app_init)
    {
        Log_Message(LOG_LEVEL_TEST, "[BLE] Attempting application layer initialization...");
        AT_Cmd_Config_t ble_init_cmds[] = {
            {"AT+BLEINIT=2\r\n", "OK", 2000, 2, "Initialize BLE Peripheral"}, // 示例指令
            {"AT+BLEADVSTART\r\n", "OK", 2000, 3, "Start BLE Advertising"},   // 示例指令
            {NULL, NULL, 0, 0, NULL}};
        uint8_t ble_init_success = 1;
        for (const AT_Cmd_Config_t *cmd = ble_init_cmds; cmd->at_cmd != NULL; cmd++)
        {
            if (g_esp32_at_ops.SendATCommand(&g_esp32_dev, cmd, ESP32_COMM_TYPE_BLE) != AT_OK)
            {
                ble_init_success = 0;
                break;
            }
            g_rtos_ops->Delay(200);
        }

        if (ble_init_success)
        {
            flags->bits.ble_app_init = 1;
            Log_Message(LOG_LEVEL_TEST, "[BLE] Application Layer Initialization successful.");
        }
        else
        {
            Log_Message(LOG_LEVEL_WARNING, "[BLE] App Layer Init failed. Retrying in next cycle.");
            // 对于BLE，失败后暂时不强制ESP32重置，而是等待下一次循环尝试
        }
    }
}

/**
 * @brief WiFi TCP 应用层初始化
 * @param flags 应用初始化标志结构体指针
 * @param max_retries 最大重试次数
 * @note 该函数会尝试连接到TCP服务器，并在失败时进行重试
 */
void wifi_tcp_init(AppInitFlags_t *flags, uint8_t max_retries)
{
    if (!flags->bits.wifi_app_init)
    {
        Log_Message(LOG_LEVEL_TEST, "[WiFi] Attempting application layer initialization...");
        AT_Cmd_Config_t wifi_init_cmds[] = {
            {"ATE1\r\n", "OK", 2000, 2, "Echo Test"},
            {"AT+GMR\r\n", "OK", 2000, 2, "Get Version", get_version_callback},
            {"AT+CWMODE=1\r\n", "OK", 2000, 2, "Set Station Mode"},
            {"AT+CWJAP=\"" WIFI_SSID "\",\"" WIFI_PASSWORD "\"\r\n", "OK", 15000, 3, "Connect to WiFi AP", set_ssid_password_callback},
            {"AT+CIPSTAMAC?\r\n", "OK", 2000, 2, "Get Station MAC", get_mac_addr_callback},
            {"AT+CIPSTA?\r\n", "OK", 2000, 2, "Get Station IP", get_ip_addr_callback},
            {NULL, NULL, 0, 0, NULL} // 哨兵值
        };
        uint8_t init_success = 1;
        for (const AT_Cmd_Config_t *cmd = wifi_init_cmds; cmd->at_cmd != NULL; cmd++)
        {
            if (g_esp32_at_ops.SendATCommand(&g_esp32_dev, cmd, ESP32_COMM_TYPE_WIFI) != AT_OK)
            {
                init_success = 0;
                break;
            }
            g_rtos_ops->Delay(200); // 命令间延时
        }

        if (init_success)
        {
            flags->bits.wifi_app_init = 1;
            esp32_status.wifi_init_retry_count = 0;
            Log_Message(LOG_LEVEL_INFO, "[WiFi] Application Layer Initialization successful.");
        }
        else
        {
            esp32_status.wifi_init_retry_count++;
            Log_Message(LOG_LEVEL_WARNING, "[WiFi] App Layer Init failed, retry %u/%u.", esp32_status.wifi_init_retry_count, max_retries);
            if (esp32_status.wifi_init_retry_count >= max_retries)
            {
                Log_Message(LOG_LEVEL_ERROR, "[WiFi] Max App Layer Init retries. Triggering ESP32 full reset cycle.");
                ESP32_ResetAndClearStatus(&esp32_status);
            }
        }
    }
    // --- 4. TCP 应用层初始化 ---
    if (!flags->bits.tcp_app_init)
    {
        Log_Message(LOG_LEVEL_TEST, "[WiFi] TCP Application Layer not initialized yet. Initializing...");
        // 这里可以添加TCP应用层初始化逻辑，例如连接到服务器等
        static char cmd_str[64];
        // snprintf(cmd_str, sizeof(cmd_str), "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", TCP_SERVER_IP, TCP_PORT);
        snprintf(cmd_str, sizeof(cmd_str), "AT+ATKCLDSTA=\"%s\",\"%s\"\r\n", AKT_DEV_ID, AKT_DEV_SECRET);
        AT_Cmd_Config_t tcp_init_cmds[] = {
            {cmd_str, "CLOUD CONNECTED", 10000, 1, "Connect to TCP Server", NULL},
            {NULL, NULL, 0, 0, NULL, NULL} // 哨兵值
        };
        uint8_t tcp_init_success = 1;
        for (const AT_Cmd_Config_t *cmd = tcp_init_cmds; cmd->at_cmd != NULL; cmd++)
        {
            if (g_esp32_at_ops.SendATCommand(&g_esp32_dev, cmd, ESP32_COMM_TYPE_WIFI) != AT_OK)
            {
                tcp_init_success = 0;
                break;
            }
            g_rtos_ops->Delay(200); // 命令间延时
        }

        if (tcp_init_success)
        {
            flags->bits.tcp_app_init = 1;
            esp32_status.tcp_init_retry_count = 0;
            Log_Message(LOG_LEVEL_TEST, "[WiFi] TCP Application Layer Initialization successful.");
        }
        else
        {
            esp32_status.tcp_init_retry_count++;
            Log_Message(LOG_LEVEL_WARNING, "[WiFi] TCP App Layer Init failed, retry %u/%u.", esp32_status.tcp_init_retry_count, max_retries);
            if (esp32_status.tcp_init_retry_count >= max_retries)
            {
                Log_Message(LOG_LEVEL_ERROR, "[WiFi] Max TCP App Layer Init retries. Triggering ESP32 full reset cycle.");
                flags->bits.tcp_app_init = 0; // 确保下次循环重新初始化应用层
                g_rtos_ops->Delay(5000);      // 长延时后从头开始
            }
            g_rtos_ops->Delay(3000); // 应用初始化失败后的短延时重试
        }
    }
}
/**
 * @brief WiFi/BLE 统一管理任务
 * @param pvParameters 未使用
 */
void App_WifiBLETask(void *pvParameters)
{
    (void)pvParameters;
    Log_Message(LOG_LEVEL_TEST, "[WiFi/BLE Task] Started.");

    const uint8_t max_ready_retries = 5;

    if (ESP32_CheckReady(&esp32_status, max_ready_retries, 2000) != 0)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi/BLE] ESP32 failed to become ready after max retries. Suspending task.");
        if (g_rtos_ops->Task_Suspend)
            g_rtos_ops->Task_Suspend(NULL);
        else
            for (;;)
                ;
        return;
    }

    while (1)
    {
        // --- 1. BLE 应用层初始化 ---
        ble_init(&app_init_flags);
        // --- 2. WiFi TCP 应用层初始化 ---
        wifi_tcp_init(&app_init_flags, 3); // 最大重试次数为3
        //
        WifiTaskLoop();
        // --- 5. WiFi/BLE 定期功能测试 ---
        if (app_init_flags.bits.wifi_app_init && app_init_flags.bits.tcp_app_init)
        {
            {
                uint8_t tx_data[] = "Hello Server from STM32 WiFi!";
                uint8_t rx_buffer[TCP_BUFFER_SIZE];
                uint16_t rx_len = sizeof(rx_buffer); // 输入期望最大长度
                // 发送TCP数据
                if (g_esp32_at_ops.SendData(&g_esp32_dev, tx_data, strlen((char *)tx_data), ESP32_COMM_TYPE_WIFI) == AT_OK)
                {
                    // 接收TCP数据
                    if (g_esp32_at_ops.ReceiveData(&g_esp32_dev, rx_buffer, &rx_len, 5000, ESP32_COMM_TYPE_WIFI) == AT_OK && rx_len > 0)
                    {
                        rx_buffer[rx_len] = '\0'; // 确保字符串终止
                        Log_Message(LOG_LEVEL_TEST, "[WiFi] TCP Recv: %s", rx_buffer);
                    }
                    else
                    {
                        Log_Message(LOG_LEVEL_WARNING, "[WiFi] No TCP data received or receive error.");
                    }
                }
                else
                {
                    Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send TCP data.");
                }
            }
        }

        if (app_init_flags.bits.ble_app_init)
        {
            uint8_t ble_tx_data[] = "Hello via BLE from STM32!";
            uint8_t ble_rx_buffer[TCP_BUFFER_SIZE];
            uint16_t ble_rx_len = sizeof(ble_rx_buffer);

            if (g_esp32_at_ops.SendData(&g_esp32_dev, ble_tx_data, strlen((char *)ble_tx_data), ESP32_COMM_TYPE_BLE) == AT_OK)
            {
                if (g_esp32_at_ops.ReceiveData(&g_esp32_dev, ble_rx_buffer, &ble_rx_len, 5000, ESP32_COMM_TYPE_BLE) == AT_OK && ble_rx_len > 0)
                {
                    ble_rx_buffer[ble_rx_len] = '\0';
                    Log_Message(LOG_LEVEL_TEST, "[BLE] Recv: %s", ble_rx_buffer);
                }
                else
                {
                    Log_Message(LOG_LEVEL_WARNING, "[BLE] No BLE data received or receive error.");
                }
            }
            else
            {
                Log_Message(LOG_LEVEL_ERROR, "[BLE] Failed to send BLE data.");
            }
        }

        g_rtos_ops->Delay(10000); // 主循环延时 (10秒)
    }
}
