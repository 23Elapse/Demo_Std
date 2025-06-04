/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 19:10:06
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-04 21:20:06
 * @FilePath: \Demo\Application\app_tasks.c
 * @Description: 应用任务实现
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "app_tasks.h"
#include "device_manager.h"
#include "rtos_abstraction.h"
#include "log_system.h"
#include "api_eeprom.h"
#include "api_wifi.h"
#include "can_driver.h"
#include "serial_driver.h"
#include "serial_interface.h"
#include "pcf8574.h"
#include "spi_flash.h"
#include "spi_flash_driver.h"
#include "stm32f4xx.h"
#include "pch.h"

// 设备实例定义
Serial_Device_t RS485_Device = {
    .instance = USART1,
    .tx_port = GPIOA,
    .tx_pin = GPIO_Pin_9,
    .rx_port = GPIOA,
    .rx_pin = GPIO_Pin_10,
    .de_port = GPIOA,
    .de_pin = GPIO_Pin_8,
    .baudrate = 115200,
    .af = GPIO_AF_USART1,
    .irqn = USART1_IRQn,
    .mode = RS485_MODE,
    .silent_ticks = 0,
    .rx_buffer = {0}};

Serial_Device_t UART_Device = {
    .instance = USART2,
    .tx_port = GPIOA,
    .tx_pin = GPIO_Pin_2,
    .rx_port = GPIOA,
    .rx_pin = GPIO_Pin_3,
    .de_port = NULL,
    .de_pin = 0,
    .baudrate = 9600,
    .af = GPIO_AF_USART2,
    .irqn = USART2_IRQn,
    .mode = UART_MODE,
    .silent_ticks = 0,
    .rx_buffer = {0}};

Serial_Device_t ESP32_Serial = {
    .instance = USART6,
    .tx_port = GPIOC,
    .tx_pin = GPIO_Pin_6,
    .rx_port = GPIOC,
    .rx_pin = GPIO_Pin_7,
    .de_port = NULL,
    .de_pin = 0,
    .baudrate = 115200,
    .af = GPIO_AF_USART6,
    .irqn = USART6_IRQn,
    .mode = UART_MODE,
    .silent_ticks = 0,
    .rx_buffer = {0}};

CAN_Device_t CAN1_Device = {
    .instance = CAN1,
    .tx_port = GPIOA,
    .tx_pin = GPIO_Pin_12,
    .rx_port = GPIOA,
    .rx_pin = GPIO_Pin_11,
    .baudrate = 500000,
    .af = GPIO_AF_CAN1,
    .irqn = CAN1_RX0_IRQn,
    .rx_buffer = {0}};

SPI_Flash_Device_t SPIFlash_Device = {
    .config = &flash_config,
    .id = 0};

// static void* esp32_shared_mutex = NULL;

// 定义全局共享ESP32设备实例
ESP32_Device_t ESP32_Device = {
    .serial_dev = &ESP32_Serial, // ESP32_Serial 是已定义的 Serial_Device_t 实例
    .mutex = NULL,               // 将在 App_Init 中创建和赋值
    .reset_port = GPIOA,         // 根据你之前的 atk_mb026_hw_init
    .reset_pin = GPIO_Pin_4      // 根据你之前的 atk_mb026_hw_init
};

// 设备管理器实例
#define MAX_DEVICES 10
static Device_Handle_t device_array[MAX_DEVICES];
static Device_Manager_t device_mgr;

void USART1_IRQHandler(void)
{
    Serial_Driver_IRQHandler(&RS485_Device);
}

void USART2_IRQHandler(void)
{
    Serial_Driver_IRQHandler(&UART_Device);
}

void USART6_IRQHandler(void)
{
    Serial_Driver_IRQHandler(&ESP32_Serial);
}

void CAN1_RX0_IRQHandler(void)
{
    CAN_IRQHandler(&CAN1_Device);
}

/**
 * @brief 初始化应用任务
 */
void App_Init(void)
{
    // 设置全局 RTOS 操作接口
    g_rtos_ops = &FreeRTOS_Ops;
    if (!g_rtos_ops)
    {
        Log_Message(LOG_LEVEL_ERROR, "[App] RTOS ops not initialized");
        return;
    }
    // 创建并分配共享互斥锁给 ESP32_Device
    void* esp32_shared_mutex = g_rtos_ops->SemaphoreCreate();
    if (!esp32_shared_mutex) {
        Log_Message(LOG_LEVEL_ERROR, "[App] Failed to create ESP32 shared mutex");
        return;
    }
    ESP32_Device.mutex = esp32_shared_mutex;
    // 初始化设备管理器
    DeviceManager_Init(&device_mgr, device_array, MAX_DEVICES);

    // 注册设备
    DeviceManager_Register(&device_mgr, &RS485_Device, DEVICE_TYPE_SERIAL, 1);
    DeviceManager_Register(&device_mgr, &UART_Device, DEVICE_TYPE_SERIAL, 2);
    DeviceManager_Register(&device_mgr, &ESP32_Serial, DEVICE_TYPE_SERIAL, 3);
    DeviceManager_Register(&device_mgr, &CAN1_Device, DEVICE_TYPE_CAN, 1);
    DeviceManager_Register(&device_mgr, &IIC1_EEPROM, DEVICE_TYPE_EEPROM, 1);
    DeviceManager_Register(&device_mgr, &IIC1_PCF8574, DEVICE_TYPE_PCF8574, 1);
    DeviceManager_Register(&device_mgr, &SPIFlash_Device, DEVICE_TYPE_SPI_FLASH, 1);
    DeviceManager_Register(&device_mgr, &ESP32_Device, DEVICE_TYPE_ESP32, 1);
    // 初始化 IIC 设备（PCF8574 和 EEPROM）
    IIC_INIT();
    ESP32_Device_HwInit(); 

    // 初始化串口设备
    Device_Handle_t *rs485_handle = DeviceManager_Find(&device_mgr, DEVICE_TYPE_SERIAL, 1);
    Device_Handle_t *uart_handle = DeviceManager_Find(&device_mgr, DEVICE_TYPE_SERIAL, 2);
    Device_Handle_t *esp32_serial_handle = DeviceManager_Find(&device_mgr, DEVICE_TYPE_SERIAL, 3);
    Device_Handle_t *esp32_handle = DeviceManager_Find(&device_mgr, DEVICE_TYPE_ESP32, 1);
    Device_Handle_t *can_handle = DeviceManager_Find(&device_mgr, DEVICE_TYPE_CAN, 1);
    Device_Handle_t *eeprom_handle = DeviceManager_Find(&device_mgr, DEVICE_TYPE_EEPROM, 1);
    Device_Handle_t *pcf8574_handle = DeviceManager_Find(&device_mgr, DEVICE_TYPE_PCF8574, 1);
    Device_Handle_t *spiflash_handle = DeviceManager_Find(&device_mgr, DEVICE_TYPE_SPI_FLASH, 1);


    if (rs485_handle && Serial_Operations.Init((Serial_Device_t *)rs485_handle->device) != SERIAL_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[App] Failed to init RS485");
    }
    if (uart_handle && Serial_Operations.Init((Serial_Device_t *)uart_handle->device) != SERIAL_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[App] Failed to init UART");
    }
    if (esp32_serial_handle && Serial_Operations.Init((Serial_Device_t *)esp32_serial_handle->device) != SERIAL_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[App] Failed to init esp32 serial");
    }
    if (can_handle && CAN_Operations.Init((CAN_Device_t *)can_handle->device) != CAN_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[App] Failed to init CAN");
    }
    if (eeprom_handle && IIC_Check(&IIC1_config, (IIC_Ops_t *)eeprom_handle->device) != 0)
    {
        Log_Message(LOG_LEVEL_ERROR, "[App] Failed to init EEPROM");
    }
    if (pcf8574_handle && pcf8574_init() != 0)
    {
        Log_Message(LOG_LEVEL_ERROR, "[App] Failed to init PCF8574");
    }
    if (spiflash_handle && W25Qxx_Init(((SPI_Flash_Device_t *)spiflash_handle->device)->config) != FLASH_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[App] Failed to init SPI Flash");
    }

    // 创建任务
    g_rtos_ops->TaskCreate(App_RS485_PollTask, "RS485_Poll", 256, rs485_handle->device, 1);
    g_rtos_ops->TaskCreate(App_SerialRxTask, "Serial_Rx", 256, rs485_handle->device, 1);
    g_rtos_ops->TaskCreate(App_ErrorLogTask, "Error_Log", 256, NULL, 1);
    g_rtos_ops->TaskCreate(App_WifiTask, "WiFi", 512, NULL, 1);
    g_rtos_ops->TaskCreate(App_BLETask, "BLE", 512, NULL, 1);   
    g_rtos_ops->TaskCreate(App_CANTask, "CAN", 256, can_handle->device, 1);
    g_rtos_ops->TaskCreate(App_SPIFlashTask, "SPI_Flash", 256, spiflash_handle->device, 1);

    // 启动调度器
    g_rtos_ops->TaskStartScheduler();
    Log_Message(LOG_LEVEL_ERROR, "[App] Failed to start scheduler");
    while (1)
    {
        // 调度器启动失败时，进入空循环
    }
}

/**
 * @brief RS485 轮询任务
 * @param pvParameters 任务参数（串口设备实例）
 */
void App_RS485_PollTask(void *pvParameters)
{
    Serial_Device_t *dev = (Serial_Device_t *)pvParameters;
    while (1)
    {
        Serial_Operations.PollSendRS485(dev);
        if (g_rtos_ops)
            g_rtos_ops->Delay(100);
    }
}

/**
 * @brief 串口接收任务
 * @param pvParameters 任务参数（串口设备实例）
 */
void App_SerialRxTask(void *pvParameters)
{
    Serial_Device_t *dev = (Serial_Device_t *)pvParameters;
    Protocol_Data_t rx_data;
    uint32_t index = 0;
    uint8_t state = 0;
    uint8_t expected_length = 0;

    while (1)
    {
        uint8_t byte;
        if (RingBuffer_IsAvailable(&dev->rx_buffer))
        {
            RingBuffer_Read(&dev->rx_buffer, &byte);
            if (Protocol_ProcessByte(dev, &rx_data, byte, &index, &state, &expected_length, NULL))
            {
                if (rx_data.is_rs485)
                {
                    RS485_Frame_t *frame = &rx_data.rs485_frame;
                    Log_Message(LOG_LEVEL_INFO, "[RS485] Received frame: addr1=0x%02X, addr2=0x%02X, cmd=0x%02X",
                                frame->addr1, frame->addr2, frame->cmd);
                }
                else
                {
                    Log_Message(LOG_LEVEL_INFO, "[UART] Received data");
                }
            }
        }
        if (g_rtos_ops)
            g_rtos_ops->Delay(10);
    }
}

/**
 * @brief 错误日志任务
 * @param pvParameters 任务参数
 */
void App_ErrorLogTask(void *pvParameters)
{
    Serial_ErrorLog_t log;
    while (1)
    {
        if (Serial_Operations.GetErrorLog(&log, 0xFFFFFFFF) == SERIAL_OK)
        {
            Log_Message(LOG_LEVEL_WARNING, "[Error] Log: type=%d, instance=%p, timestamp=%u",
                        log.type, log.instance, (unsigned int)log.timestamp);
        }
    }
}

/**
 * @brief WiFi 管理任务
 * @param pvParameters 任务参数（未使用）
 */
void App_WifiTask(void *pvParameters)
{
    ESP32_Device_HwReset();  // 调用新的统一硬件复位函数
    // 【关键修改】在硬件复位后，增加延时等待ESP32启动
    Log_Message(LOG_LEVEL_INFO, "[WiFi] ESP32 BCM_SWRESET, waiting for boot up...");
    g_rtos_ops->Delay(3000); // 等待3秒钟，这个时间可能需要根据实际情况调整，可以尝试2-5秒
    Log_Message(LOG_LEVEL_INFO, "[WiFi] ESP32 boot wait complete. Flushing buffer and starting AT commands.");

    // 在这之后，第一次调用 WiFi_SendATCommand 时，其内部的缓冲区清空逻辑
    // (while (RingBuffer_IsAvailable(...)) RingBuffer_Read(...);)
    // 将会清空这3秒等待期间ESP32可能输出的任何残余启动信息。

    static uint8_t is_initialized = 0;
    uint8_t retry_count = 0;
    const uint8_t max_retries = 3;


    while (1)
    {
        // 初始化 WiFi 模块
        if (!is_initialized)
        {
            AT_Cmd_Config init_cmds[] = {
                {"AT\r\n", "OK", 1000, 3, "AT command test"},
                {"AT+CWMODE=1\r\n", "OK", 2000, 2, "Set Station Mode"},
                {"AT+CWJAP=\"" WIFI_SSID "\",\"" WIFI_PASSWORD "\"\r\n", "OK", 10000, 3, "Connect to WiFi"},
                {NULL, NULL, 0, 0, NULL}
            };
            uint8_t init_success = 1;
            for (const AT_Cmd_Config *cmd = init_cmds; cmd->at_cmd != NULL; cmd++)
            {
                if (WiFi_SendATCommand(cmd) != AT_ERR_NONE)
                {
                    Log_Message(LOG_LEVEL_ERROR, "[WiFi] Init command failed: %s", cmd->description);
                    init_success = 0;
                    break;
                }
                g_rtos_ops->Delay(100);
            }
            if (init_success)
            {
                is_initialized = 1;
                retry_count = 0;
                Log_Message(LOG_LEVEL_INFO, "[WiFi] Initialization successful");
            }
            else
            {
                retry_count++;
                if (retry_count >= max_retries)
                {
                    Log_Message(LOG_LEVEL_ERROR, "[WiFi] Max retries reached, suspending task");
                    g_rtos_ops->Task_Suspend(NULL);
                }
                ESP32_Device_HwReset(); // 硬件复位 ESP32
                Log_Message(LOG_LEVEL_WARNING, "[WiFi] Init failed, retry %d/%d", retry_count, max_retries);
                g_rtos_ops->Delay(5000);
                continue;
            }
        }

        // 连接 TCP 服务器
        char cmd_str[64];
        snprintf(cmd_str, sizeof(cmd_str), "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", TCP_SERVER_IP, TCP_PORT);
        AT_Cmd_Config tcp_cmd = {
            .at_cmd = cmd_str,
            .expected_resp = "OK",
            .timeout_ms = 5000,
            .retries = 3,
            .description = "Connect to TCP server"
        };
        if (WiFi_SendATCommand(&tcp_cmd) == AT_ERR_NONE)
        {
            // 发送 TCP 数据
            uint8_t data[] = "Hello from STM32!";
            uint16_t data_len = strlen((char *)data);
            if (WiFi_SendTCPData(data, data_len) == AT_ERR_NONE)
            {
                // 接收 TCP 数据
                uint8_t rx_buffer[TCP_BUFFER_SIZE];
                uint16_t rx_len = TCP_BUFFER_SIZE;
                if (WiFi_ReceiveTCPData(rx_buffer, &rx_len, 5000) == AT_ERR_NONE)
                {
                    Log_Message(LOG_LEVEL_INFO, "[WiFi] Received TCP data: %.*s", rx_len, rx_buffer);
                }
            }
            // 断开 TCP 连接
            AT_Cmd_Config close_cmd = {
                .at_cmd = "AT+CIPCLOSE\r\n",
                .expected_resp = "OK",
                .timeout_ms = 2000,
                .retries = 2,
                .description = "Disconnect TCP server"
            };
            WiFi_SendATCommand(&close_cmd);
        }
        else
        {
            Log_Message(LOG_LEVEL_WARNING, "[WiFi] Failed to connect to TCP server, resetting init state");
            is_initialized = 0; // 连接失败，重新初始化
        }
        g_rtos_ops->Delay(10000);
    }
}

/**
 * @brief BLE 管理任务
 * @param pvParameters 任务参数（未使用）
 */
void App_BLETask(void *pvParameters)
{
    ESP32_Device_HwReset();  // 调用新的统一硬件复位函数
    // 【关键修改】在硬件复位后，增加延时等待ESP32启动
    Log_Message(LOG_LEVEL_INFO, "[WiFi] ESP32 BCM_SWRESET, waiting for boot up...");
    g_rtos_ops->Delay(3000); // 等待3秒钟，这个时间可能需要根据实际情况调整，可以尝试2-5秒
    Log_Message(LOG_LEVEL_INFO, "[WiFi] ESP32 boot wait complete. Flushing buffer and starting AT commands.");
    static uint8_t is_initialized = 0;
    uint8_t retry_count = 0;
    const uint8_t max_retries = 3;

    while (1)
    {
        // 初始化 BLE 模块
        if (!is_initialized)
        {
            AT_Cmd_Config init_cmds[] = {
                {"AT\r\n", "OK", 1000, 3, "BLE AT command test"},
                {"AT+BLEINIT=1\r\n", "OK", 2000, 2, "Initialize BLE"},
                {"AT+BLEADVERT=1\r\n", "OK", 5000, 3, "Start BLE advertising"},
                {NULL, NULL, 0, 0, NULL}
            };
            uint8_t init_success = 1;
            for (const AT_Cmd_Config *cmd = init_cmds; cmd->at_cmd != NULL; cmd++)
            {
                if (BLE_SendATCommand(cmd) != AT_ERR_NONE)
                {
                    Log_Message(LOG_LEVEL_ERROR, "[BLE] Init command failed: %s", cmd->description);
                    init_success = 0;
                    break;
                }
                g_rtos_ops->Delay(100);
            }
            if (init_success)
            {
                is_initialized = 1;
                retry_count = 0;
                Log_Message(LOG_LEVEL_INFO, "[BLE] Initialization successful");
            }
            else
            {
                retry_count++;
                if (retry_count >= max_retries)
                {
                    Log_Message(LOG_LEVEL_ERROR, "[BLE] Max retries reached, suspending task");
                    g_rtos_ops->Task_Suspend(NULL);
                }
                ESP32_Device_HwReset(); // 硬件复位 ESP32
                Log_Message(LOG_LEVEL_WARNING, "[BLE] Init failed, retry %d/%d", retry_count, max_retries);
                g_rtos_ops->Delay(5000);
                continue;
            }
        }

        // 发送 BLE 数据（示例）
        uint8_t data[] = "BLE Data from STM32";
        uint16_t data_len = strlen((char *)data);
        if (BLE_SendData(data, data_len) == AT_ERR_NONE)
        {
            // 接收 BLE 数据
            uint8_t rx_buffer[TCP_BUFFER_SIZE];
            uint16_t rx_len = TCP_BUFFER_SIZE;
            if (BLE_ReceiveData(rx_buffer, &rx_len, 5000) == AT_ERR_NONE)
            {
                Log_Message(LOG_LEVEL_INFO, "[BLE] Received data: %.*s", rx_len, rx_buffer);
            }
        }
        else
        {
            Log_Message(LOG_LEVEL_WARNING, "[BLE] Failed to send data, resetting init state");
            is_initialized = 0; // 通信失败，重新初始化
        }
        g_rtos_ops->Delay(10000);
    }
}

/**
 * @brief CAN 管理任务
 * @param pvParameters 任务参数（CAN 设备实例）
 */
void App_CANTask(void *pvParameters)
{
    CAN_Device_t *can_dev = (CAN_Device_t *)pvParameters;
    if (!can_dev)
    {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] Invalid handle");
        if (g_rtos_ops)
            vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        // 发送 CAN 消息
        CAN_Message_t tx_msg = {
            .id = 0x123,
            .length = 8,
            .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}};
        if (CAN_Operations.SendMessage(can_dev, &tx_msg) == CAN_OK)
        {
            Log_Message(LOG_LEVEL_INFO, "[CAN] Sent message ID: 0x%03X", tx_msg.id);
        }
        else
        {
            Log_Message(LOG_LEVEL_ERROR, "[CAN] Failed to send message");
        }

        // 接收 CAN 消息
        CAN_Message_t rx_msg;
        if (CAN_Operations.ReceiveMessage(can_dev, &rx_msg, 1000) == CAN_OK)
        {
            char data_str[3 * 8 + 1] = {0};
            for (int i = 0; i < rx_msg.length && i < 8; i++)
            {
                char byte_str[4];
                snprintf(byte_str, sizeof(byte_str), "%02X ", rx_msg.data[i]);
                strcat(data_str, byte_str);
            }
            Log_Message(LOG_LEVEL_INFO, "[CAN] Received ID: 0x%03X, len: %d, data: %s",
                        rx_msg.id, rx_msg.length, data_str);
        }

        if (g_rtos_ops)
            g_rtos_ops->Delay(1000);
    }
}

/**
 * @brief SPI Flash 管理任务
 * @param pvParameters 任务参数（SPI Flash 设备实例）
 */
void App_SPIFlashTask(void *pvParameters)
{
    SPI_Flash_Device_t *flash_dev = (SPI_Flash_Device_t *)pvParameters;
    if (!flash_dev || !flash_dev->config)
    {
        Log_Message(LOG_LEVEL_ERROR, "[SPI Flash] Invalid handle");
        if (g_rtos_ops)
            vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        uint8_t write_data[] = {0xAA, 0xBB, 0xCC, 0xDD};
        uint32_t address = 0x000000;
        uint16_t data_len = sizeof(write_data);

        if (SPI_Flash_Write_With_Erase(flash_dev->config, write_data, address, data_len) == 0)
        {
            uint8_t read_data[4];
            if (SPI_Flash_ReadData(flash_dev->config, read_data, address, data_len) == FLASH_OK)
            {
                Log_Message(LOG_LEVEL_INFO, "[SPI Flash] Read: 0x%02X, 0x%02X, 0x%02X, 0x%02X",
                            read_data[0], read_data[1], read_data[2], read_data[3]);
            }
            else
            {
                Log_Message(LOG_LEVEL_ERROR, "[SPI Flash] Read failed");
            }
        }
        else
        {
            Log_Message(LOG_LEVEL_ERROR, "[SPI Flash] Write with erase failed");
        }

        if (g_rtos_ops)
            g_rtos_ops->Delay(5000);
    }
}
