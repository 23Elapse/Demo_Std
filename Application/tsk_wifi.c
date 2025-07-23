#include "tsk_wifi.h"


typedef union {
    struct {
        uint8_t wifi_app_init   : 1;
        uint8_t ble_app_init    : 1;
        uint8_t tcp_app_init    : 1;
        uint8_t wifi_init_done  : 1;
        uint8_t reserved        : 4;
    } bits;
    uint8_t all;
} AppInitFlags_t;

// Encapsulate stage index for multi-stage init
typedef struct {
    uint8_t current;    // current stage index
    uint8_t count;      // total stages
} StageIndex_t;

// Aggregated status context
typedef struct {
    AppInitFlags_t    init_flags;
    StageIndex_t      wifi_stage;
    // future: other channels stage index (e.g. ble_stage)
} AppStatus_t;

// Initialize status context
static inline void AppStatus_Reset(AppStatus_t *status) {
    status->init_flags.all = 0;
    status->wifi_stage.current = 0;
    status->wifi_stage.count = INIT_STAGE_COUNT;
}

// Helpers
static inline uint8_t AppStatus_IsWifiReady(const AppStatus_t *status) {
    return status->init_flags.bits.wifi_app_init && status->init_flags.bits.wifi_init_done;
}

static inline void AppStatus_SetWifiAppInit(AppStatus_t *status) {
    status->init_flags.bits.wifi_app_init = 1;
}

static inline void AppStatus_SetWifiInitDone(AppStatus_t *status) {
    status->init_flags.bits.wifi_init_done = 1;
}

static AppInitFlags_t app_init_flags = { .all = 0 };


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
    uint8_t is_initialized = (channel == COMM_TCP) ? (app_init_flags.bits.wifi_app_init && app_init_flags.bits.tcp_app_init) : app_init_flags.bits.ble_app_init;

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

// void WifiTaskLoop(void)
// {
//     // 先做多阶段初始化发送
//     ServerCommProcess(wifi_send_buf, sizeof(wifi_send_buf),
//                       wifi_init_stages,
//                       &wifi_stage_idx,
//                       &wifi_init_done,
//                       ESP32_COMM_TYPE_WIFI,
//                       "WIFI");

//     // 处理接收ACK，调用时机由接收数据事件触发
//     /*
//     if (recv_data_available) {
//         ServerCommProcess_HandleAck(wifi_init_stages, &wifi_stage_idx, &wifi_init_done, (char*)recv_buffer, "WIFI");
//     }
//     */
//     // 初始化完成后执行周期性发送
//     Wifi_PeriodicSendHandler(ESP32_COMM_TYPE_WIFI);
// }

// ============================== 初始化阶段顺序控制 ==============================

// 阶段掩码定义（启用的阶段）
typedef enum {
    INIT_STAGE_DEVICE_TREE   = (1 << 0),
    INIT_STAGE_PROP_TREE     = (1 << 1),
    INIT_STAGE_OFFLINE_TREE  = (1 << 2),
    INIT_STAGE_TIME_SYNC     = (1 << 3),
    INIT_STAGE_HEARTBEAT     = (1 << 4),

    INIT_STAGE_ALL           = INIT_STAGE_DEVICE_TREE | INIT_STAGE_PROP_TREE | INIT_STAGE_OFFLINE_TREE | INIT_STAGE_TIME_SYNC | INIT_STAGE_HEARTBEAT,
} InitStageMask_t;

// 阶段索引（顺序）
typedef enum {
    INIT_INDEX_DEVICE_TREE = 0,
    INIT_INDEX_PROP_TREE,
    INIT_INDEX_OFFLINE_TREE,
    INIT_INDEX_TIME_SYNC,
    INIT_INDEX_HEARTBEAT,
    INIT_INDEX_COUNT
} InitStageIndex_t;

// 初始化控制结构
typedef struct {
    uint8_t mask;          // 启用的阶段掩码
    uint8_t finished_mask; // 已完成的阶段掩码
    uint8_t current_stage; // 当前阶段索引
    uint8_t completed;     // 当前阶段是否已完成（修改：只表示当前阶段完成）

    uint8_t trigger_mask;  // 触发重新发送的阶段掩码（可动态置位触发）
} InitPhaseControl_t;

// 定时发送控制结构体
typedef struct {
    uint32_t last_realtime_tick;     // 上次发送实时数据的时间戳（单位ms）
    uint32_t last_history_tick;      // 上次发送历史数据的时间戳（单位ms）
    uint32_t realtime_interval_ms;   // 实时数据发送间隔
    uint32_t history_interval_ms;    // 历史数据发送间隔
} PeriodicSendControl_t;

// 初始化函数
static inline void InitPhaseControl_Init(InitPhaseControl_t *ctrl, uint8_t mask) {
    ctrl->mask = mask;
    ctrl->finished_mask = 0;
    ctrl->current_stage = 0;
    ctrl->completed = 0;      // 修改：当前阶段未完成
    ctrl->trigger_mask = 0;
}

// 判断当前阶段是否需要发送
static inline int InitPhaseControl_NeedSend(InitPhaseControl_t *ctrl) {
    uint8_t stage_bit = (1 << ctrl->current_stage);
    // 当前阶段启用且未完成，或者触发了重发
    if ((ctrl->mask & stage_bit) && (!(ctrl->finished_mask & stage_bit) || (ctrl->trigger_mask & stage_bit))) {
        return 1;
    }
    return 0;
}

// 收到当前阶段成功响应，**不自动推进到下一阶段**，只标记完成并清除触发标志
static inline void InitPhaseControl_OnResponse(InitPhaseControl_t *ctrl) {
    uint8_t stage_bit = (1 << ctrl->current_stage);
    ctrl->finished_mask |= stage_bit;       // 标记当前阶段完成
    ctrl->trigger_mask &= ~stage_bit;       // 清除触发标志

    // 找下一个需要发送的阶段
    for (int i = ctrl->current_stage + 1; i < INIT_INDEX_COUNT; i++) {
        uint8_t bit = (1 << i);
        if ((ctrl->mask & bit) && !(ctrl->finished_mask & bit)) {
            ctrl->current_stage = i;
            ctrl->completed = 0;
            return;
        }
    }

    // 如果没有下一个未完成的启用阶段，则全部完成
    ctrl->completed = ((ctrl->finished_mask & ctrl->mask) == ctrl->mask);
}

// 外部调用，设置触发标志（某阶段需要重新发送），切换当前阶段索引
static inline void InitPhaseControl_TriggerStage(InitPhaseControl_t *ctrl, InitStageMask_t stage) {
    if ((ctrl->mask & stage)) {
        ctrl->finished_mask &= ~stage;      // 标记该阶段为未完成
        ctrl->trigger_mask |= stage;        // 设置触发标志
        // 切换到触发阶段索引
        for (int i = 0; i < INIT_INDEX_COUNT; i++) {
            if ((1 << i) == stage) {
                ctrl->current_stage = i;
                ctrl->completed = 0;         // 当前阶段未完成
                break;
            }
        }
    }
}

// 初始化周期发送控制
static inline void PeriodicSendControl_Init(PeriodicSendControl_t *ctrl, uint32_t rt_interval_ms, uint32_t hist_interval_ms) {
    ctrl->last_realtime_tick = 0;
    ctrl->last_history_tick = 0;
    ctrl->realtime_interval_ms = rt_interval_ms;
    ctrl->history_interval_ms = hist_interval_ms;
}

// 获取系统当前时间（单位ms）外部实现
uint32_t GetTickMs(void);

// 发送接口声明（需外部实现）
void SendDeviceTree(void);
void SendPropTree(void);
void SendOfflineTree(void);
void SendTimeSyncCommand(void);
void SendHeartbeatCommand(void);
void SendRealtimeData(void);
void SendHistoryData(void);

// 模拟接收成功响应（这里直接返回true，模拟成功）
int ReceiveStageResponse(uint8_t stage) {
    printf("Received response for stage %d\n", stage);
    return 1; // 模拟总是成功
}
// 周期发送检查和执行
static inline void PeriodicSendControl_CheckAndSend(PeriodicSendControl_t *ctrl) {
    uint32_t now = GetTickMs();
    if (now - ctrl->last_realtime_tick >= ctrl->realtime_interval_ms) {
        SendRealtimeData();
        ctrl->last_realtime_tick = now;
    }
    if (now - ctrl->last_history_tick >= ctrl->history_interval_ms) {
        SendHistoryData();
        ctrl->last_history_tick = now;
    }
}

// 假设此结构体定义在共享头文件中，并由主任务循环持有
extern InitPhaseControl_t init_ctrl; // 声明全局控制结构体

// 其他任务调用此函数触发某阶段重发
void ExternalTrigger_ReSendStage(InitStageMask_t stage) {
    printf("[External Task] Trigger resend stage mask: 0x%02X\n", stage);
    InitPhaseControl_TriggerStage(&init_ctrl, stage);
}

// 主循环任务中定义控制结构体（简化示例）
InitPhaseControl_t init_ctrl;


// 主任务循环示例（结合阶段顺序和周期发送）
void WifiTaskLoop(void) {
    // static InitPhaseControl_t init_ctrl;
    static PeriodicSendControl_t periodic_ctrl;
    static uint8_t initialized = 0;

    if (!initialized) {
        InitPhaseControl_Init(&init_ctrl, INIT_STAGE_ALL);
        PeriodicSendControl_Init(&periodic_ctrl, 20000, 10000); // 20s实时数据, 10s历史数据
        initialized = 1;
    }

    // 初始化阶段处理
    if (!init_ctrl.completed) {
        if (InitPhaseControl_NeedSend(&init_ctrl)) {
            switch (init_ctrl.current_stage) {
                case INIT_INDEX_DEVICE_TREE:   SendDeviceTree(); break;
                case INIT_INDEX_PROP_TREE:     SendPropTree(); break;
                case INIT_INDEX_OFFLINE_TREE:  SendOfflineTree(); break;
                case INIT_INDEX_TIME_SYNC:     SendTimeSyncCommand(); break;
                case INIT_INDEX_HEARTBEAT:     SendHeartbeatCommand(); break;
                default: break;
            }
        }
        if (ReceiveStageResponse(init_ctrl.current_stage)) {
            Log_Message(LOG_LEVEL_INFO, "[WIFI] Stage %d response received.", init_ctrl.current_stage);
            // 收到当前阶段响应，标记完成
            InitPhaseControl_OnResponse(&init_ctrl);
        }
    } else {
        // 所有初始化阶段完成后，执行周期性数据发送
        PeriodicSendControl_CheckAndSend(&periodic_ctrl);
    }
}


ESP32Status_t esp32_status = {0};
/**
 * @brief 检查ESP32是否准备就绪
 * @param status ESP32状态结构体指针
 * @param max_retries 最大重试次数
 * @param delay_ms 每次重试间隔（毫秒）
 * @return 0表示就绪，-1表示未就绪
 */
int ESP32_CheckReady(ESP32Status_t *status, int max_retries, uint32_t delay_ms)
{
    while (!status->ready && status->ready_retry_count < max_retries) {
        ESP32_Hw_Reset(&g_esp32_dev);
        Log_Message(LOG_LEVEL_TEST, "[WiFi/BLE] ESP32 Reset, waiting for boot up (attempt %u/%u)...",
                    status->ready_retry_count + 1, max_retries);
        g_rtos_ops->Delay(300);

        AT_Cmd_Config_t at_test_cmd = {"AT\r\n", "OK", 2000, 2, "ESP32 Ready Test"};
        if (g_esp32_at_ops.SendATCommand(&g_esp32_dev, &at_test_cmd, ESP32_COMM_TYPE_WIFI) == AT_OK) {
            Log_Message(LOG_LEVEL_TEST, "[WiFi/BLE] ESP32 is ready.");
            status->ready = 1;
        } else {
            Log_Message(LOG_LEVEL_WARNING, "[WiFi/BLE] ESP32 not ready. Retrying...");
            status->ready_retry_count++;
            if (status->ready_retry_count < max_retries) {
                g_rtos_ops->Delay(delay_ms);
            }
        }
    }

    if (!status->ready) {
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
    wifi_stage_idx = 0; // 重置阶段索引
    g_rtos_ops->Delay(5000);
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
    const uint8_t max_app_init_retries = 3;

    if (ESP32_CheckReady(&esp32_status, max_ready_retries, 2000) != 0) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi/BLE] ESP32 failed to become ready after max retries. Suspending task.");
        if (g_rtos_ops->Task_Suspend) g_rtos_ops->Task_Suspend(NULL);
        else for(;;);
        return;
    }

    while (1) {
        // --- 2. WiFi 应用层初始化 ---
        if (!app_init_flags.bits.wifi_app_init) {
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
                app_init_flags.bits.wifi_app_init = 1;
                esp32_status.wifi_init_retry_count = 0;
                Log_Message(LOG_LEVEL_INFO, "[WiFi] Application Layer Initialization successful.");
            } else {
                esp32_status.wifi_init_retry_count++;
                Log_Message(LOG_LEVEL_WARNING, "[WiFi] App Layer Init failed, retry %u/%u.", esp32_status.wifi_init_retry_count, max_app_init_retries);
                if (esp32_status.wifi_init_retry_count >= max_app_init_retries) {
                    Log_Message(LOG_LEVEL_ERROR, "[WiFi] Max App Layer Init retries. Triggering ESP32 full reset cycle.");
                    ESP32_ResetAndClearStatus(&esp32_status);
                    app_init_flags.bits.wifi_app_init = 0; // 确保下次循环重新初始化应用层
                    continue; // 重置状态并重新开始
                }
                g_rtos_ops->Delay(3000); // 应用初始化失败后的短延时重试
            }
        }
        // --- 3. BLE 应用层初始化 ---
        if (!app_init_flags.bits.ble_app_init) {
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
                app_init_flags.bits.ble_app_init = 1;
                Log_Message(LOG_LEVEL_TEST, "[BLE] Application Layer Initialization successful.");
            } else {
                Log_Message(LOG_LEVEL_WARNING, "[BLE] App Layer Init failed. Retrying in next cycle.");
                // 对于BLE，失败后暂时不强制ESP32重置，而是等待下一次循环尝试
            }
        }
        // --- 4. TCP 应用层初始化 ---
        if (!app_init_flags.bits.tcp_app_init) {
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
                app_init_flags.bits.tcp_app_init = 1;
                esp32_status.tcp_init_retry_count = 0;
                Log_Message(LOG_LEVEL_TEST, "[WiFi] TCP Application Layer Initialization successful.");
            } else {
                esp32_status.tcp_init_retry_count++;
                Log_Message(LOG_LEVEL_WARNING, "[WiFi] TCP App Layer Init failed, retry %u/%u.", esp32_status.tcp_init_retry_count, max_app_init_retries);
                if (esp32_status.tcp_init_retry_count >= max_app_init_retries) {
                    Log_Message(LOG_LEVEL_ERROR, "[WiFi] Max TCP App Layer Init retries. Triggering ESP32 full reset cycle.");
                    app_init_flags.bits.tcp_app_init = 0; // 确保下次循环重新初始化应用层
                    g_rtos_ops->Delay(5000); // 长延时后从头开始
                    continue; // 重置状态并重新开始
                }
                g_rtos_ops->Delay(3000); // 应用初始化失败后的短延时重试
            }
        }
        
        WifiTaskLoop(); 
        // --- 5. WiFi/BLE 定期功能测试 ---
        if (app_init_flags.bits.wifi_app_init && app_init_flags.bits.tcp_app_init) {
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

        if (app_init_flags.bits.ble_app_init) {
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
