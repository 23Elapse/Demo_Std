/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-27 19:10:06
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-03 15:00:00
 * @FilePath: \Demo\Application\Src\app_tasks.c
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

Serial_Device_t WiFi_Serial = {
    .instance = USART3,
    .tx_port = GPIOB,
    .tx_pin = GPIO_Pin_10,
    .rx_port = GPIOB,
    .rx_pin = GPIO_Pin_11,
    .de_port = NULL,
    .de_pin = 0,
    .baudrate = 115200,
    .af = GPIO_AF_USART3,
    .irqn = USART3_IRQn,
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

// IIC 设备通过 IIC_AttachDevice 挂载，无需在此定义 IIC_Ops_t 实例

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

void USART3_IRQHandler(void)
{
    Serial_Driver_IRQHandler(&WiFi_Serial);
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
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops)
    {
        Log_Message(LOG_LEVEL_ERROR, "[App] RTOS ops not initialized");
        return;
    }

    RTOS_SetOps(&FreeRTOS_Ops);

    // 初始化设备管理器
    DeviceManager_Init(&device_mgr, device_array, MAX_DEVICES);

    // 注册设备
    DeviceManager_Register(&device_mgr, &RS485_Device, DEVICE_TYPE_SERIAL, 1);
    DeviceManager_Register(&device_mgr, &UART_Device, DEVICE_TYPE_SERIAL, 2);
    DeviceManager_Register(&device_mgr, &WiFi_Serial, DEVICE_TYPE_SERIAL, 3);
    DeviceManager_Register(&device_mgr, &CAN1_Device, DEVICE_TYPE_CAN, 1);
    DeviceManager_Register(&device_mgr, &IIC1_EEPROM, DEVICE_TYPE_EEPROM, 1);
    DeviceManager_Register(&device_mgr, &IIC1_PCF8574, DEVICE_TYPE_PCF8574, 1);

    // 初始化 IIC 设备（PCF8574 和 EEPROM）
    IIC_INIT();

    // 初始化串口设备
    Device_Handle_t *rs485_handle = DeviceManager_Find(&device_mgr, DEVICE_TYPE_SERIAL, 1);
    Device_Handle_t *uart_handle = DeviceManager_Find(&device_mgr, DEVICE_TYPE_SERIAL, 2);
    Device_Handle_t *wifi_handle = DeviceManager_Find(&device_mgr, DEVICE_TYPE_SERIAL, 3);
    Device_Handle_t *can_handle = DeviceManager_Find(&device_mgr, DEVICE_TYPE_CAN, 1);
    Device_Handle_t *eeprom_handle = DeviceManager_Find(&device_mgr, DEVICE_TYPE_EEPROM, 1);
    Device_Handle_t *pcf8574_handle = DeviceManager_Find(&device_mgr, DEVICE_TYPE_PCF8574, 1);

    if (rs485_handle && Serial_Operations.Init((Serial_Device_t *)rs485_handle->device) != SERIAL_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[App] Failed to init RS485");
    }
    if (uart_handle && Serial_Operations.Init((Serial_Device_t *)uart_handle->device) != SERIAL_OK)
    {
        Log_Message(LOG_LEVEL_ERROR, "[App] Failed to init UART");
    }
    if (wifi_handle && WiFi_Init((Serial_Device_t *)wifi_handle->device) != AT_ERR_NONE)
    {
        Log_Message(LOG_LEVEL_ERROR, "[App] Failed to init WiFi");
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

    // 创建任务
    rtos_ops->TaskCreate(App_RS485_PollTask, "RS485_Poll", 256, rs485_handle->device, 1);
    rtos_ops->TaskCreate(App_SerialRxTask, "Serial_Rx", 256, rs485_handle->device, 1);
    rtos_ops->TaskCreate(App_ErrorLogTask, "Error_Log", 256, NULL, 1);
    rtos_ops->TaskCreate(App_EEPROMTask, "EEPROM", 256, eeprom_handle->device, 1);
    rtos_ops->TaskCreate(App_WifiTask, "WiFi", 512, wifi_handle->device, 1);
    rtos_ops->TaskCreate(App_CANTask, "CAN", 256, can_handle->device, 1);
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
        const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
        if (rtos_ops)
            rtos_ops->Delay(100);
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
    while (1)
    {
        if (Serial_Operations.ReceiveFromBuffer(dev, &rx_data, 0xFFFFFFFF) == SERIAL_OK)
        {
            if (rx_data.is_rs485)
            {
                RS485_Frame_t *frame = &rx_data.rs485_frame;
                Log_Message(LOG_LEVEL_INFO, "[RS485] Received frame: addr1=0x%02X, addr2=0x%02X, cmd=0x%02X",
                            frame->addr1, frame->addr2, frame->cmd);
            }
            else
            {
                Log_Message(LOG_LEVEL_INFO, "[UART] Received data: length=%d", rx_data.uart_data.length);
            }
        }
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
 * @brief EEPROM 管理任务
 * @param pvParameters 任务参数（EEPROM 设备实例）
 */
void App_EEPROMTask(void *pvParameters)
{
    IIC_Ops_t *eeprom = (IIC_Ops_t *)pvParameters;
    if (!eeprom)
    {
        Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Invalid handle");
        vTaskDelete(NULL);
    }

    while (1)
    {
        uint8_t write_data[] = {0x11, 0x22, 0x33, 0x44};
        if (EEPROMWriteBytesToReg(0x00, write_data, 4) == IIC_OK)
        {
            uint8_t read_data[4];
            if (EEPROMReadBytesFromReg(0x00, read_data, 4) == IIC_OK)
            {
                Log_Message(LOG_LEVEL_INFO, "[EEPROM] Read: 0x%02X, 0x%02X, 0x%02X, 0x%02X",
                            read_data[0], read_data[1], read_data[2], read_data[3]);
            }
            else
            {
                Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Multi-byte read failed");
            }
        }
        else
        {
            Log_Message(LOG_LEVEL_ERROR, "[EEPROM] Multi-byte write failed");
        }
        const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
        if (rtos_ops)
            rtos_ops->Delay(5000);
    }
}

/**
 * @brief WiFi 管理任务
 * @param pvParameters 任务参数（串口设备实例）
 */
void App_WifiTask(void *pvParameters)
{
    Serial_Device_t *serial_dev = (Serial_Device_t *)pvParameters;
    if (!serial_dev)
    {
        Log_Message(LOG_LEVEL_ERROR, "[WiFi] Invalid handle");
        vTaskDelete(NULL);
    }

    while (1)
    {
        if (WiFi_ConnectTCPServer(serial_dev, TCP_SERVER_IP, TCP_PORT) == AT_ERR_NONE)
        {
            uint8_t data[] = "Hello from STM32!";
            uint16_t data_len = strlen((char *)data);
            if (WiFi_SendTCPData(serial_dev, data, data_len) == AT_ERR_NONE)
            {
                uint8_t rx_buffer[TCP_BUFFER_SIZE];
                uint16_t rx_len = TCP_BUFFER_SIZE;
                if (WiFi_ReceiveTCPData(serial_dev, rx_buffer, &rx_len, 5000) == AT_ERR_NONE)
                {
                    Log_Message(LOG_LEVEL_INFO, "[WiFi] Received TCP data: %.*s", rx_len, rx_buffer);
                }
                else
                {
                    Log_Message(LOG_LEVEL_WARNING, "[WiFi] Failed to receive TCP data");
                }
            }
            else
            {
                Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to send TCP data");
            }
        }
        else
        {
            Log_Message(LOG_LEVEL_ERROR, "[WiFi] Failed to connect to TCP server");
        }
        const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
        if (rtos_ops)
            rtos_ops->Delay(10000);
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
        vTaskDelete(NULL);
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
            Log_Message(LOG_LEVEL_INFO, "[CAN] Received message ID: 0x%03X, length: %d", rx_msg.id, rx_msg.length);
        }

        const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
        if (rtos_ops)
            rtos_ops->Delay(1000);
    }
}

/*
 * 示例用法：
 * 1. 在主函数中调用 App_Init()
 * App_Init();
 *
 * 2. 确保 RTOS 抽象层已设置
 * RTOS_SetOps(&FreeRTOS_Ops);
 *
 * 3. 设备通过 DeviceManager 注册并初始化
 */
