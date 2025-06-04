/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 19:10:06
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-04 21:40:54
 * @FilePath: \Demo\Application\app_tasks.c
 * @Description: 应用任务实现
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "app_tasks.h"
#include "device_manager.h"
#include "rtos_abstraction.h"
#include "log_system.h"
#include "api_eeprom.h"   // 假设存在并定义了 IIC1_EEPROM 和相关操作
#include "api_wifi.h"     // 已更新为统一ESP32接口
#include "can_driver.h"
#include "serial_driver.h"
#include "serial_interface.h" // 包含 Serial_Operations
#include "pcf8574.h"      // 假设存在并定义了 IIC1_PCF8574 和相关操作
#include "spi_flash.h"
#include "spi_flash_driver.h" // 假设包含 W25Qxx_Init
#include "stm32f4xx.h"
#include "pch.h" // 假设包含 IIC1_config 和 flash_config

// 设备实例定义
Serial_Device_t RS485_Device = {
    .instance = USART1,
    .tx_port = GPIOA, .tx_pin = GPIO_Pin_9,
    .rx_port = GPIOA, .rx_pin = GPIO_Pin_10,
    .de_port = GPIOA, .de_pin = GPIO_Pin_8,
    .baudrate = 115200, .af = GPIO_AF_USART1, .irqn = USART1_IRQn,
    .mode = RS485_MODE, .silent_ticks = 0, .rx_buffer = {0}
};

Serial_Device_t UART_Device = {
    .instance = USART2,
    .tx_port = GPIOA, .tx_pin = GPIO_Pin_2,
    .rx_port = GPIOA, .rx_pin = GPIO_Pin_3,
    .de_port = NULL, .de_pin = 0,
    .baudrate = 9600, .af = GPIO_AF_USART2, .irqn = USART2_IRQn,
    .mode = UART_MODE, .silent_ticks = 0, .rx_buffer = {0}
};

Serial_Device_t ESP32_Serial = { // ESP32模块的底层串口
    .instance = USART6,
    .tx_port = GPIOC, .tx_pin = GPIO_Pin_6,
    .rx_port = GPIOC, .rx_pin = GPIO_Pin_7,
    .de_port = NULL, .de_pin = 0,
    .baudrate = 115200, .af = GPIO_AF_USART6, .irqn = USART6_IRQn,
    .mode = UART_MODE, .silent_ticks = 0, .rx_buffer = {0}
};

CAN_Device_t CAN1_Device = {
    .instance = CAN1,
    .tx_port = GPIOA, .tx_pin = GPIO_Pin_12,
    .rx_port = GPIOA, .rx_pin = GPIO_Pin_11,
    .baudrate = 500000, .af = GPIO_AF_CAN1, .irqn = CAN1_RX0_IRQn,
    .rx_buffer = {0} // CAN驱动的rx_buffer可能与串口不同，此处仅为占位
};

// 假设 IIC1_EEPROM 和 IIC1_PCF8574 是 IIC_Ops_t 类型或类似结构
// 如果它们是其他类型，DeviceManager_Register 和初始化需要相应调整
// 例如:
//IIC_Ops_t IIC1_EEPROM = { // 示例定义，请根据您的实际IIC驱动调整
//    .hi2c = &hi2c1, // 假设您使用STM32 HAL库的I2C句柄
//    .dev_addr = 0xA0 // EEPROM的典型地址，可能需要移位
//};
//IIC_Ops_t IIC1_PCF8574 = { // 示例定义
//    .hi2c = &hi2c1,
//    .dev_addr = 0x40 // PCF8574的典型地址，可能需要移位
//};


SPI_Flash_Device_t SPIFlash_Device = {
    .config = &flash_config, // 假设 flash_config 是 SPI_Flash_Config_t 类型指针
    .id = 0 // 设备管理器中的ID，非Flash芯片ID
};

/**
 * @brief 全局共享ESP32设备实例的定义和初始化
 */
ESP32_Shared_Device_t ESP32_Device = {
    .serial_dev = &ESP32_Serial,
    .mutex = NULL,
    .reset_port = GPIOA,
    .reset_pin = GPIO_Pin_4
};

// 设备管理器实例
#define MAX_DEVICES 10
static Device_Handle_t device_array[MAX_DEVICES];
static Device_Manager_t device_mgr;

// 中断处理函数
void USART1_IRQHandler(void) { if(RS485_Device.instance) Serial_Driver_IRQHandler(&RS485_Device); }
void USART2_IRQHandler(void) { if(UART_Device.instance) Serial_Driver_IRQHandler(&UART_Device); }
void USART6_IRQHandler(void) { if(ESP32_Serial.instance) Serial_Driver_IRQHandler(&ESP32_Serial); }
void CAN1_RX0_IRQHandler(void) { if(CAN1_Device.instance) CAN_IRQHandler(&CAN1_Device); }

/**
 * @brief 初始化应用任务
 */
void App_Init(void)
{
    g_rtos_ops = &FreeRTOS_Ops;
    if (!g_rtos_ops || !g_rtos_ops->SemaphoreCreate || !g_rtos_ops->TaskCreate || !g_rtos_ops->TaskStartScheduler) {
        // 无法记录日志，因为日志系统可能依赖RTOS
        for(;;); // 严重错误，停机
    }

    void* esp32_shared_mutex = g_rtos_ops->SemaphoreCreate();
    if (!esp32_shared_mutex) {
        // Log_Message(LOG_LEVEL_FATAL, "[App] Failed to create ESP32 shared mutex");
        for(;;); // 停机
    }
    ESP32_Device.mutex = esp32_shared_mutex;

    DeviceManager_Init(&device_mgr, device_array, MAX_DEVICES);

    // 注册所有设备
    DeviceManager_Register(&device_mgr, &RS485_Device, DEVICE_TYPE_SERIAL, 1);
    DeviceManager_Register(&device_mgr, &UART_Device, DEVICE_TYPE_SERIAL, 2);
    DeviceManager_Register(&device_mgr, &ESP32_Serial, DEVICE_TYPE_SERIAL, 3);
    DeviceManager_Register(&device_mgr, &CAN1_Device, DEVICE_TYPE_CAN, 1);
    DeviceManager_Register(&device_mgr, &IIC1_EEPROM, DEVICE_TYPE_EEPROM, 1);
    DeviceManager_Register(&device_mgr, &IIC1_PCF8574, DEVICE_TYPE_PCF8574, 1);
    DeviceManager_Register(&device_mgr, &SPIFlash_Device, DEVICE_TYPE_SPI_FLASH, 1);
    DeviceManager_Register(&device_mgr, &ESP32_Device, DEVICE_TYPE_ESP32, 1);

    // 初始化基础硬件和驱动
    IIC_INIT(); // 假设此函数初始化I2C外设
    ESP32_Device_HwInit();

    // 初始化各个设备驱动 (直接使用全局实例)
    if (Serial_Operations.Init(&RS485_Device) != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[App] Failed to init RS485");
    }
    if (Serial_Operations.Init(&UART_Device) != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[App] Failed to init UART");
    }
    if (Serial_Operations.Init(&ESP32_Serial) != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[App] Failed to init ESP32 serial (USART6)");
    }
    if (CAN_Operations.Init(&CAN1_Device) != CAN_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[App] Failed to init CAN");
    }
    // EEPROM 和 PCF8574 的初始化依赖于其具体驱动实现
    // 假设有类似 API_EEPROM_Init(&IIC1_EEPROM) 和 API_PCF8574_Init(&IIC1_PCF8574) 的函数
    // 或者它们的初始化已包含在 IIC_INIT() 或其他地方
    // if (API_EEPROM_Init(&IIC1_EEPROM) != STATUS_OK) { // 示例
    //     Log_Message(LOG_LEVEL_ERROR, "[App] Failed to init EEPROM");
    // }
    // if (API_PCF8574_Init(&IIC1_PCF8574) != STATUS_OK) { // 示例
    //     Log_Message(LOG_LEVEL_ERROR, "[App] Failed to init PCF8574");
    // }
    if (SPIFlash_Device.config && W25Qxx_Init(SPIFlash_Device.config) != FLASH_OK) { // 确保config有效
        Log_Message(LOG_LEVEL_ERROR, "[App] Failed to init SPI Flash");
    }


    // 创建任务
    g_rtos_ops->TaskCreate(App_RS485_PollTask, "RS485_Poll", 256, &RS485_Device, 1);
    g_rtos_ops->TaskCreate(App_SerialRxTask, "Serial_Rx", 256, &RS485_Device, 1);
    g_rtos_ops->TaskCreate(App_ErrorLogTask, "Error_Log", 256, NULL, 1);
    g_rtos_ops->TaskCreate(App_WifiTask, "WiFi", 512, NULL, 2); // 提高WiFi/BLE任务优先级
    g_rtos_ops->TaskCreate(App_BLETask, "BLE", 512, NULL, 2);
    g_rtos_ops->TaskCreate(App_CANTask, "CAN", 256, &CAN1_Device, 1);
    g_rtos_ops->TaskCreate(App_SPIFlashTask, "SPI_Flash", 256, &SPIFlash_Device, 1);

    Log_Message(LOG_LEVEL_INFO, "[App] Starting scheduler...");
    g_rtos_ops->TaskStartScheduler();
    Log_Message(LOG_LEVEL_ERROR, "[App] Scheduler exited unexpectedly!"); // 不应执行到此
    for(;;);
}

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
        if (g_rtos_ops->Delay) g_rtos_ops->Delay(3000); // 等待3秒

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
