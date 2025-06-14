/**
 * =====================================================================================
 * @file        app_tasks.c
 * @brief       应用层任务创建与系统初始化协调中心
 * @author      23Elapse & Gemini
 * @version     2.0 (Refactored)
 * @date        2025-06-08
 * @note        这是系统启动的总入口，负责按顺序初始化所有模块并创建任务。
 * =====================================================================================
 */
#include "app_tasks.h"
#include "dev_config.h"         // 关键：包含所有设备实例声明
#include "device_manager.h"     // 关键：包含设备管理器接口
#include "rtos_abstraction.h"
#include "log_system.h"
#include "serial_interface.h"   // 需要 Serial_Operations
#include "can_driver.h"         // 需要 CAN_Operations
#include "i2c_driver.h"         // 需要 g_i2c_bus_ops
#include "spi_flash.h"
#include "tsk_eeprom.h"         // 需要 Tsk_Eeprom_Init
#include "FreeRTOS.h"
#include "task.h"

/*
 * =====================================================================================
 * 宏定义 - 任务参数
 * =====================================================================================
 */
#define TASK_RS485_POLL_STK_SIZE    256
#define TASK_RS485_POLL_PRIO        2

#define TASK_SERIAL_RX_STK_SIZE     256
#define TASK_SERIAL_RX_PRIO         3 // 接收任务优先级可以高一些

#define TASK_ERROR_LOG_STK_SIZE     128
#define TASK_ERROR_LOG_PRIO         1

#define TASK_WIFI_STK_SIZE          512
#define TASK_WIFI_PRIO              2

#define TASK_BLE_STK_SIZE           512
#define TASK_BLE_PRIO               2

#define TASK_CAN_STK_SIZE           256
#define TASK_CAN_PRIO               3

#define TASK_SPI_FLASH_STK_SIZE     256
#define TASK_SPI_FLASH_PRIO         2

/*
 * =====================================================================================
 * 模块私有变量
 * =====================================================================================
 */
#define MAX_DEVICES 10
static Device_Handle_t g_device_array[MAX_DEVICES];
static Device_Manager_t g_device_mgr;


/*
 * =====================================================================================
 * 内部函数声明
 * =====================================================================================
 */
static void App_RTOS_Init(void);
static void App_Device_Register(void);
static void App_Driver_Init(void);
static void App_Tasks_Create(void);

// 各个任务的函数原型声明
void App_RS485_PollTask(void* pvParameters);
void App_SerialRxTask(void* pvParameters);
void App_ErrorLogTask(void* pvParameters);
void App_WifiTask(void* pvParameters);
void App_BLETask(void* pvParameters);
void App_CANTask(void* pvParameters);
void App_SPIFlashTask(void* pvParameters);


/*
 * =====================================================================================
 * 主初始化函数 (系统入口)
 * =====================================================================================
 */
void App_Init(void) {
    App_RTOS_Init();
    App_Device_Register();
    App_Driver_Init();
    App_Tasks_Create();

    Log_Message(LOG_LEVEL_INFO, "[App] Starting RTOS scheduler...");
    g_rtos_ops->TaskStartScheduler();

    Log_Message(LOG_LEVEL_ERROR, "FATAL: Scheduler exited unexpectedly!");
    for (;;);
}


/*
 * =====================================================================================
 * 初始化辅助函数
 * =====================================================================================
 */
static void App_RTOS_Init(void) {
    g_rtos_ops = &FreeRTOS_Ops;
    if (!g_rtos_ops || !g_rtos_ops->SemaphoreCreate || !g_rtos_ops->TaskCreate) {
        for (;;);
    }
    // ESP32的Mutex在其驱动内部或dev_config中处理更佳，此处简化
}

static void App_Device_Register(void) {
    DeviceManager_Init(&g_device_mgr, g_device_array, MAX_DEVICES);
    Log_Message(LOG_LEVEL_INFO, "[App] Registering devices to manager...");

    DeviceManager_Register(&g_device_mgr, &g_rs485_serial, DEVICE_TYPE_SERIAL, 1);
    DeviceManager_Register(&g_device_mgr, &g_uart_dev,     DEVICE_TYPE_SERIAL, 2);
    DeviceManager_Register(&g_device_mgr, &g_esp32_serial,  DEVICE_TYPE_SERIAL, 3);
    DeviceManager_Register(&g_device_mgr, &g_can1_dev,      DEVICE_TYPE_CAN_BUS, 1);
    DeviceManager_Register(&g_device_mgr, &g_i2c1_bus,      DEVICE_TYPE_I2C_BUS, 1);
    DeviceManager_Register(&g_device_mgr, &g_spi_flash_dev, DEVICE_TYPE_SPI_FLASH, 1);
    DeviceManager_Register(&g_device_mgr, &g_esp32_dev,     DEVICE_TYPE_ESP32, 1);
    // 为ESP32设备创建共享互斥锁
    g_esp32_dev.mutex = g_rtos_ops->SemaphoreCreate();
    if (!g_esp32_dev.mutex) {
        Log_Message(LOG_LEVEL_ERROR, "[App] create ESP32 mutex failed"); // 日志系统此时可能未就绪
        for(;;); // 停机
    }
}

static void App_Driver_Init(void) {
    Log_Message(LOG_LEVEL_INFO, "[App] Initializing hardware drivers...");

    // 初始化底层物理驱动
    Serial_Operations.Init(&g_rs485_serial);
    Serial_Operations.Init(&g_uart_dev);
    Serial_Operations.Init(&g_esp32_serial);
    CAN_Operations.Init(&g_can1_dev);
    g_i2c_bus_ops.Init(&g_i2c1_bus);
    SPI_Flash_Init(g_spi_flash_dev.config);

    // 初始化高层服务/模块
    // Tsk_Eeprom_Init(); // 此函数会初始化参数并创建自己的后台任务
}

static void App_Tasks_Create(void) {
    Log_Message(LOG_LEVEL_INFO, "[App] Creating application tasks...");

    g_rtos_ops->TaskCreate(App_RS485_PollTask, "RS485_Poll", TASK_RS485_POLL_STK_SIZE, &g_rs485_serial, TASK_RS485_POLL_PRIO);
    g_rtos_ops->TaskCreate(App_SerialRxTask, "Serial_Rx", TASK_SERIAL_RX_STK_SIZE, &g_rs485_serial, TASK_SERIAL_RX_PRIO);
    g_rtos_ops->TaskCreate(App_ErrorLogTask, "Error_Log", TASK_ERROR_LOG_STK_SIZE, NULL, TASK_ERROR_LOG_PRIO);
    g_rtos_ops->TaskCreate(App_WifiTask, "WiFi", TASK_WIFI_STK_SIZE, NULL, TASK_WIFI_PRIO);
    g_rtos_ops->TaskCreate(App_BLETask, "BLE", TASK_BLE_STK_SIZE, NULL, TASK_BLE_PRIO);
    g_rtos_ops->TaskCreate(App_CANTask, "CAN", TASK_CAN_STK_SIZE, &g_can1_dev, TASK_CAN_PRIO);
    g_rtos_ops->TaskCreate(App_SPIFlashTask, "SPI_Flash", TASK_SPI_FLASH_STK_SIZE, &g_spi_flash_dev, TASK_SPI_FLASH_PRIO);
    
    // 注意：EepromMonitorTask 由 Tsk_Eeprom_Init() 内部创建，此处无需再创建
}

/*
 * =====================================================================================
 * 任务函数实现 (此处仅为框架，具体实现需保留)
 * =====================================================================================
 */
// void App_RS485_PollTask(void *pvParameters) {
//     Serial_Device_t *dev = (Serial_Device_t *)pvParameters;
//     for(;;) {
//         Serial_Operations.PollSendRS485(dev);
//         g_rtos_ops->Delay(10); // 轮询间隔
//     }
// }

/**
 * @brief RS485 轮询任务
 */
void App_RS485_PollTask(void *pvParameters) {
    Serial_Device_t *dev = (Serial_Device_t *)pvParameters;
    if (!dev) {
        Log_Message(LOG_LEVEL_ERROR, "[RS485 Poll] Invalid device parameter. Task suspending.");
        if(g_rtos_ops && g_rtos_ops->Task_Suspend) g_rtos_ops->Task_Suspend(NULL); else for(;;);
    }
    Log_Message(LOG_LEVEL_INFO, "[RS485 Poll] Task started.");
    while (1) {
        if (dev->mode == RS485_MODE) { // 确保是RS485设备
             Serial_Operations.PollSendRS485(dev);
        }
        if (g_rtos_ops && g_rtos_ops->Delay) g_rtos_ops->Delay(100);
    }
}

/**
 * @brief 串口接收任务 (示例，处理RS485数据)
 */
void App_SerialRxTask(void *pvParameters) {
    Serial_Device_t *dev = (Serial_Device_t *)pvParameters;
     if (!dev) {
        Log_Message(LOG_LEVEL_ERROR, "[Serial Rx] Invalid device parameter. Task suspending.");
        if(g_rtos_ops && g_rtos_ops->Task_Suspend) g_rtos_ops->Task_Suspend(NULL); else for(;;);
    }
    Log_Message(LOG_LEVEL_INFO, "[Serial Rx] Task started for USART %p.", dev->instance);

    Protocol_Data_t local_rx_data;
    uint32_t local_rx_index = 0;
    uint8_t local_state = 0;
    uint8_t local_expected_length = 0;
    memset(&local_rx_data, 0, sizeof(Protocol_Data_t));


    while (1) {
        uint8_t byte;
        // 确保dev和环形缓冲区有效
        if (dev && dev->rx_buffer.buffer && dev->rx_buffer.sem && RingBuffer_IsAvailable(&dev->rx_buffer)) {
            // 使用带超时的信号量等待，避免完全阻塞或忙等
            // if (g_rtos_ops->SemaphoreTake(dev->rx_buffer.sem, pdMS_TO_TICKS(10))) { // 等待10ms
                if (RingBuffer_Read(&dev->rx_buffer, &byte) == RB_OK) {
                    // g_rtos_ops->SemaphoreGive(dev->rx_buffer.sem); // 如果RingBuffer_Read不处理信号量，则需要

                    if (Protocol_ProcessByte(dev, &local_rx_data, byte, &local_rx_index, &local_state, &local_expected_length, NULL)) {
                        if (local_rx_data.is_rs485) {
                            RS485_Frame_t *frame = &local_rx_data.rs485_frame;
                            Log_Message(LOG_LEVEL_INFO, "[RS485 Rx] Frame: addr1=0x%02X, cmd=0x%02X, len=%d",
                                        frame->addr1, frame->cmd, frame->info_len);
                        } else {
                            Log_Message(LOG_LEVEL_INFO, "[UART Rx] Data on USART %p (non-RS485)", dev->instance);
                        }
                        // 重置状态以便处理下一帧
                        local_rx_index = 0;
                        local_state = 0;
                        local_expected_length = 0;
                        memset(&local_rx_data, 0, sizeof(Protocol_Data_t));
                    }
                } // else { g_rtos_ops->SemaphoreGive(dev->rx_buffer.sem); } // 读取失败也要释放
            // }
        } else {
             if (g_rtos_ops && g_rtos_ops->Delay) g_rtos_ops->Delay(10); // 如果没数据，短暂延时
        }
    }
}

/**
 * @brief 错误日志任务
 */
void App_ErrorLogTask(void *pvParameters) {
    Log_Message(LOG_LEVEL_INFO, "[Error Log] Task started.");
    Serial_ErrorLog_t log_entry; // 重命名以避免与全局 log 冲突
    while (1) {
        if (Serial_Operations.GetErrorLog(&log_entry, 0xFFFFFFFF) == SERIAL_OK) { // 阻塞等待
            Log_Message(LOG_LEVEL_WARNING, "[ErrorLog] Type=%d, Inst=%p, Timestamp=%u",
                        log_entry.type, log_entry.instance, (unsigned int)log_entry.timestamp);
        }
    }
}

/**
 * @brief WiFi 管理任务
 */
void App_WifiTask(void *pvParameters)
{
    Log_Message(LOG_LEVEL_INFO, "[WiFi Task] Started.");
    uint8_t esp32_ready = 0;
    uint8_t ready_retry_count = 0;
    const uint8_t max_ready_retries = 5;

    // 循环检查ESP32模块是否就绪
    while(!esp32_ready && ready_retry_count < max_ready_retries) {
        ESP32_Device_HwReset(); // 硬件复位ESP32
        Log_Message(LOG_LEVEL_INFO, "[WiFi] ESP32 Reset, waiting for boot up (attempt %d/%d)...", ready_retry_count + 1, max_ready_retries);
        if (g_rtos_ops->Delay) g_rtos_ops->Delay(300); // 等待300ms

        // 发送基础AT指令测试模块是否响应
        AT_Cmd_Config at_test_cmd = {"AT\r\n", "OK", 2000, 2, "ESP32 Ready Test"};
        if (WiFi_SendATCommand(&at_test_cmd) == AT_ERR_NONE) {
            Log_Message(LOG_LEVEL_INFO, "[WiFi] ESP32 is ready.");
            esp32_ready = 1;
        } else {
            Log_Message(LOG_LEVEL_WARNING, "[WiFi] ESP32 not ready. Retrying...");
            ready_retry_count++;
            if (g_rtos_ops->Delay && ready_retry_count < max_ready_retries) g_rtos_ops->Delay(2000); // 重试前额外延时
        }
    }

    if (!esp32_ready) {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] ESP32 failed to become ready after %d attempts. Suspending task.", max_ready_retries);
        if (g_rtos_ops->Task_Suspend) g_rtos_ops->Task_Suspend(NULL); else for(;;); // 挂起任务
        return; // 理论上不应执行到此
    }

    static uint8_t wifi_app_initialized = 0; // WiFi应用层初始化标志
    uint8_t app_init_retry_count = 0;
    const uint8_t max_app_init_retries = 3;

    while (1) {
        if (!wifi_app_initialized) {
            AT_Cmd_Config init_cmds[] = {
                {"AT+CWMODE=1\r\n", "OK", 2000, 2, "Set Station Mode"},
                {"AT+CWJAP=\"" WIFI_SSID "\",\"" WIFI_PASSWORD "\"\r\n", "OK", 15000, 3, "Connect to WiFi AP"},
                {NULL, NULL, 0, 0, NULL}
            };
            uint8_t init_success = 1;
            for (const AT_Cmd_Config *cmd = init_cmds; cmd->at_cmd != NULL; cmd++) {
                if (WiFi_SendATCommand(cmd) != AT_ERR_NONE) {
                    init_success = 0;
                    break;
                }
                if (g_rtos_ops->Delay) g_rtos_ops->Delay(200);
            }

            if (init_success) {
                wifi_app_initialized = 1;
                app_init_retry_count = 0;
                Log_Message(LOG_LEVEL_INFO, "[WiFi] Application Layer Initialization successful.");
            } else {
                app_init_retry_count++;
                Log_Message(LOG_LEVEL_WARNING, "[WiFi] App Layer Init failed, retry %d/%d.", app_init_retry_count, max_app_init_retries);
                if (app_init_retry_count >= max_app_init_retries) {
                    Log_Message(LOG_LEVEL_ERROR, "[WiFi] Max App Layer Init retries. ESP32 may need full reset cycle.");
                    esp32_ready = 0; // 标记ESP32需要重新检查就绪状态
                    ready_retry_count = 0; // 重置ESP32就绪检查计数
                    wifi_app_initialized = 0; // 确保下次循环重新初始化应用层
                    if (g_rtos_ops->Delay) g_rtos_ops->Delay(5000); // 长延时后从头开始
                    continue; // 返回到外层while，重新检查esp32_ready
                }
                if (g_rtos_ops->Delay) g_rtos_ops->Delay(3000); // 应用初始化失败后的短延时重试
            }
        }

        if (wifi_app_initialized) { // 只有在应用初始化成功后才执行TCP操作
            char cmd_str[64];
            snprintf(cmd_str, sizeof(cmd_str), "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", TCP_SERVER_IP, TCP_PORT);
            AT_Cmd_Config tcp_cmd = {cmd_str, "OK", 10000, 1, "Connect to TCP Server"};

            if (WiFi_SendATCommand(&tcp_cmd) == AT_ERR_NONE) {
                Log_Message(LOG_LEVEL_INFO, "[WiFi] TCP Connected.");
                uint8_t data_payload[] = "Hello Server from STM32 WiFi!"; // 重命名避免与全局data冲突
                if (WiFi_SendTCPData(data_payload, strlen((char *)data_payload)) == AT_ERR_NONE) {
                    uint8_t rx_buf[TCP_BUFFER_SIZE]; // 重命名
                    uint16_t rx_buf_len = sizeof(rx_buf) - 1;
                    if (WiFi_ReceiveTCPData(rx_buf, &rx_buf_len, 5000) == AT_ERR_NONE && rx_buf_len > 0) {
                        rx_buf[rx_buf_len] = '\0';
                        Log_Message(LOG_LEVEL_INFO, "[WiFi] TCP Recv: %s", rx_buf);
                    }
                }
                AT_Cmd_Config close_cmd = {"AT+CIPCLOSE\r\n", "OK", 2000, 0, "Disconnect TCP"};
                WiFi_SendATCommand(&close_cmd);
            } else {
                Log_Message(LOG_LEVEL_WARNING, "[WiFi] Failed to connect TCP. Checking AP connection...");
                AT_Cmd_Config check_ap_cmd = {"AT+CWJAP?\r\n", WIFI_SSID, 3000, 0, "Check AP"};
                if (WiFi_SendATCommand(&check_ap_cmd) != AT_ERR_NONE) {
                    Log_Message(LOG_LEVEL_WARNING, "[WiFi] AP connection lost. Re-init app layer.");
                    wifi_app_initialized = 0; // 触发应用层重初始化
                }
            }
        }
        if (g_rtos_ops->Delay) g_rtos_ops->Delay(10000); // 主循环延时
    }
}

/**
 * @brief BLE 管理任务
 */
void App_BLETask(void *pvParameters)
{
    Log_Message(LOG_LEVEL_INFO, "[BLE Task] Started.");
    uint8_t esp32_ready = 0;
    uint8_t ready_retry_count = 0;
    const uint8_t max_ready_retries = 5;

    while(!esp32_ready && ready_retry_count < max_ready_retries) {
        ESP32_Device_HwReset();
        Log_Message(LOG_LEVEL_INFO, "[BLE] ESP32 Reset, waiting for boot up (attempt %d/%d)...", ready_retry_count + 1, max_ready_retries);
        if (g_rtos_ops->Delay) g_rtos_ops->Delay(3000);

        AT_Cmd_Config at_test_cmd = {"AT\r\n", "OK", 2000, 2, "ESP32 Ready Test for BLE"};
        if (BLE_SendATCommand(&at_test_cmd) == AT_ERR_NONE) { // 使用 BLE_SendATCommand
            Log_Message(LOG_LEVEL_INFO, "[BLE] ESP32 is ready.");
            esp32_ready = 1;
        } else {
            Log_Message(LOG_LEVEL_WARNING, "[BLE] ESP32 not ready. Retrying...");
            ready_retry_count++;
            if (g_rtos_ops->Delay && ready_retry_count < max_ready_retries) g_rtos_ops->Delay(2000);
        }
    }

    if (!esp32_ready) {
        Log_Message(LOG_LEVEL_ERROR, "[BLE] ESP32 failed to become ready after %d attempts. Suspending task.", max_ready_retries);
        if (g_rtos_ops->Task_Suspend) g_rtos_ops->Task_Suspend(NULL); else for(;;);
        return;
    }

    static uint8_t ble_app_initialized = 0;
    uint8_t app_init_retry_count = 0;
    const uint8_t max_app_init_retries = 3;

    while (1) {
        if (!ble_app_initialized) {
            AT_Cmd_Config init_cmds[] = {
                {"AT+BLEINIT=2\r\n", "OK", 2000, 2, "Initialize BLE Peripheral"}, // 示例指令
                {"AT+BLEADVSTART\r\n", "OK", 2000, 3, "Start BLE Advertising"}, // 示例指令
                {NULL, NULL, 0, 0, NULL}
            };
            uint8_t init_success = 1;
            for (const AT_Cmd_Config *cmd = init_cmds; cmd->at_cmd != NULL; cmd++) {
                if (BLE_SendATCommand(cmd) != AT_ERR_NONE) {
                    init_success = 0;
                    break;
                }
                if (g_rtos_ops->Delay) g_rtos_ops->Delay(200);
            }

            if (init_success) {
                ble_app_initialized = 1;
                app_init_retry_count = 0;
                Log_Message(LOG_LEVEL_INFO, "[BLE] Application Layer Initialization successful.");
            } else {
                app_init_retry_count++;
                 Log_Message(LOG_LEVEL_WARNING, "[BLE] App Layer Init failed, retry %d/%d.", app_init_retry_count, max_app_init_retries);
                if (app_init_retry_count >= max_app_init_retries) {
                    Log_Message(LOG_LEVEL_ERROR, "[BLE] Max App Layer Init retries. ESP32 may need full reset cycle.");
                    esp32_ready = 0;
                    ready_retry_count = 0;
                    ble_app_initialized = 0;
                    if (g_rtos_ops->Delay) g_rtos_ops->Delay(5000);
                    continue;
                }
                 if (g_rtos_ops->Delay) g_rtos_ops->Delay(3000);
            }
        }

        if (ble_app_initialized) {
            uint8_t ble_data_payload[] = "Hello via BLE from STM32!"; // 重命名
            if (BLE_SendData(ble_data_payload, strlen((char*)ble_data_payload)) == AT_ERR_NONE) {
                uint8_t ble_rx_buf[TCP_BUFFER_SIZE]; // 重命名
                uint16_t ble_rx_buf_len = sizeof(ble_rx_buf) - 1;
                if (BLE_ReceiveData(ble_rx_buf, &ble_rx_buf_len, 5000) == AT_ERR_NONE && ble_rx_buf_len > 0) {
                    ble_rx_buf[ble_rx_buf_len] = '\0';
                    Log_Message(LOG_LEVEL_INFO, "[BLE] Recv: %s", ble_rx_buf);
                }
            } else {
                Log_Message(LOG_LEVEL_WARNING, "[BLE] Failed to send data.");
                // ble_app_initialized = 0; // 可选：发送失败则重新初始化应用
            }
        }
        if (g_rtos_ops->Delay) g_rtos_ops->Delay(10000);
    }
}

/**
 * @brief CAN 管理任务
 */
void App_CANTask(void *pvParameters) {
    CAN_Device_t *can_dev = (CAN_Device_t *)pvParameters;
    if (!can_dev) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] Invalid device parameter. Task suspending.");
        if(g_rtos_ops && g_rtos_ops->Task_Suspend) g_rtos_ops->Task_Suspend(NULL); else for(;;);
    }
    Log_Message(LOG_LEVEL_INFO, "[CAN Task] Started.");
    while (1) {
        CAN_Message_t tx_msg = {
            .id = 0x123, .length = 8,
            .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}
        };
        if (CAN_Operations.SendMessage(can_dev, &tx_msg) == CAN_OK) {
            // Log_Message(LOG_LEVEL_DEBUG, "[CAN] Sent message ID: 0x%03X", tx_msg.id);
        } else {
            Log_Message(LOG_LEVEL_ERROR, "[CAN] Failed to send message");
        }

        CAN_Message_t rx_msg;
        if (CAN_Operations.ReceiveMessage(can_dev, &rx_msg, 1000) == CAN_OK) { // 1秒超时
            // char data_str[24] = {0}; // 8 bytes * 3 chars/byte (XX ) = 24
            // for (int i = 0; i < rx_msg.length && i < 8; i++) {
            //     snprintf(data_str + i*3, 4, "%02X ", rx_msg.data[i]);
            // }
            // Log_Message(LOG_LEVEL_DEBUG, "[CAN] Recv ID: 0x%03X, Data: %s", rx_msg.id, data_str);
        }
        if (g_rtos_ops && g_rtos_ops->Delay) g_rtos_ops->Delay(1000);
    }
}

/**
 * @brief SPI Flash 管理任务
 */
void App_SPIFlashTask(void *pvParameters) {
    SPI_Flash_Device_t *flash_dev = (SPI_Flash_Device_t *)pvParameters;
    if (!flash_dev || !flash_dev->config) {
        Log_Message(LOG_LEVEL_ERROR, "[SPI Flash] Invalid device parameter. Task suspending.");
        if(g_rtos_ops && g_rtos_ops->Task_Suspend) g_rtos_ops->Task_Suspend(NULL); else for(;;);
    }
    Log_Message(LOG_LEVEL_INFO, "[SPI Flash Task] Started.");
    uint8_t test_val = 0;
    while (1) {
        uint8_t write_buffer[4]; // 重命名
        write_buffer[0] = test_val++;
        write_buffer[1] = 0xBB;
        write_buffer[2] = 0xCC;
        write_buffer[3] = 0xDD;

        uint32_t address = 0x000000; // 始终写入同一地址进行测试
        uint16_t data_len = sizeof(write_buffer);

        if (SPI_Flash_Write_With_Erase(flash_dev->config, write_buffer, address, data_len) == 0) { // 假设0是成功
            uint8_t read_buffer[4]; // 重命名
            if (SPI_Flash_ReadData(flash_dev->config, read_buffer, address, data_len) == FLASH_OK) {
                if (memcmp(write_buffer, read_buffer, data_len) == 0) {
                    // Log_Message(LOG_LEVEL_DEBUG, "[SPI Flash] R/W Test OK: %02X %02X %02X %02X",
                    //             read_buffer[0], read_buffer[1], read_buffer[2], read_buffer[3]);
                } else {
                    Log_Message(LOG_LEVEL_ERROR, "[SPI Flash] R/W Mismatch!");
                }
            } else {
                Log_Message(LOG_LEVEL_ERROR, "[SPI Flash] Read failed after write.");
            }
        } else {
            Log_Message(LOG_LEVEL_ERROR, "[SPI Flash] Write with erase failed.");
        }
        if (g_rtos_ops && g_rtos_ops->Delay) g_rtos_ops->Delay(5000);
    }
}

// 可以将此代码段添加到 app_tasks.c 的末尾

/**
 * @brief  当 pvPortMalloc() 返回 NULL 时，此钩子函数会被调用。
 * @note   通常是因为FreeRTOS的堆空间不足。
 */
void vApplicationMallocFailedHook(void)
{
    // 在这里设置一个断点！
    // 如果程序停在这里，就说明是总堆空间(configTOTAL_HEAP_SIZE)不足导致的。
    taskDISABLE_INTERRUPTS();
    for(;;)
    {
    }
}

/**
 * @brief  当检测到任务栈溢出时，此钩子函数会被调用。
 * @param  pxTask: 发生溢出的任务句柄
 * @param  pcTaskName: 发生溢出的任务名
 */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pxTask; // 未使用参数

    // 在这里设置一个断点！
    // 如果程序停在这里，就说明名为 pcTaskName 的任务发生了栈溢出。
    // 你需要增加创建该任务时分配的堆栈大小。
    Log_Message(LOG_LEVEL_ERROR, "FATAL: Stack overflow in task: %s", pcTaskName);
    taskDISABLE_INTERRUPTS();
    for(;;)
    {
    }
}
