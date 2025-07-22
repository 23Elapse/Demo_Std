/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-04-01 20:52:45
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-07-21 21:06:36
 * @FilePath: \Demo\Application\api_wifi.h
 * @Description: ESP32 WiFi 和 BLE 模块统一驱动头文件 (Refactored)
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __API_WIFI_H
#define __API_WIFI_H

#include "log_system.h"
#include "rtos_abstraction.h"
#include "serial_driver.h" // 依赖底层串口驱动

/**
 * @brief WiFi/BLE 配置宏
 */
#define WIFI_SSID       "104"
#define WIFI_PASSWORD   "ABC104104"
#define TCP_SERVER_IP   "192.168.2.100"
#define TCP_PORT        "5000"
#define UART_TIMEOUT    5000 // 通用串口操作超时
#define TCP_BUFFER_SIZE 256 // TCP数据缓冲区大小

#define AKT_DEV_ID      "14456411462837499097" // AKT设备ID
#define AKT_DEV_SECRET  "12345678"  // AKT设备密钥

#define MAX_FRAME_SIZE     512  // 单帧最大数据长度
#define CRC16_LEN          4    // CRC16以ASCII十六进制表示，长度固定4字节
#define PEEK_BUF_SIZE      128  // 从环形缓冲区中预读的最大字节数

/**
 * @brief 前缀定义结构体
 */
typedef struct {
    const char* prefix;
    uint8_t prefix_len;
} AT_DataPrefix_t;

/**
 * @brief AT 指令错误码枚举
 */
typedef enum {
    AT_OK = 0,                  // 操作成功
    AT_ERR_PARAM,               // 参数错误
    AT_ERR_TIMEOUT,             // 操作超时
    AT_ERR_UNEXPECTED_RESPONSE, // 收到非预期响应
    AT_ERR_SEND_FAILED,         // 发送数据失败
    AT_ERR_CONNECTION_FAILED,   // 连接失败
    AT_ERR_DISCONNECTED,        // 已断开连接
    AT_ERR_MUTEX_TIMEOUT        // 获取互斥锁超时
} AT_Status_t;

/**
 * @brief AT 指令配置结构体
 * @param at_cmd: 要发送的AT指令字符串
 * @param expected_resp: 期望的响应字符串 (用于判断成功)
 * @param timeout_ms: 等待响应的超时时间 (毫秒)
 * @param retries: 重试次数 (针对AT指令发送)
 * @param description: 指令描述 (用于日志)
 */
typedef struct {
    const char* at_cmd;
    const char* expected_resp;
    uint32_t timeout_ms;
    uint8_t retries;
    const char* description;
    void (*callback)(const char* resp); // 回调函数，指令完成后调用
} AT_Cmd_Config_t;
/**
 * @brief ESP32 设备信息结构体
 * 保存设备的版本、IP地址、MAC地址、SSID、密码等信息
 */
typedef struct {
    char version[32];      // ESP32固件版本号
    char ip_addr[16];      // IP地址 (xxx.xxx.xxx.xxx)
    char mac_addr[18];     // MAC地址 (XX:XX:XX:XX:XX:XX)
    char ssid[32];         // WiFi SSID
    char password[64];     // WiFi密码
} ESP32_Device_Info_t;

// 声明全局唯一的ESP32设备信息实例
extern ESP32_Device_Info_t g_esp32_info;
/**
 * @brief ESP32 通信类型枚举
 * 用于区分 AT 命令是针对 WiFi 还是 BLE 功能
 */
typedef enum {
    ESP32_COMM_TYPE_WIFI,
    ESP32_COMM_TYPE_BLE
} ESP32_Comm_Type_t;

/**
 * @brief ESP32 共享设备结构体
 * 该结构体封装了ESP32模块共享的硬件资源和同步机制。
 */
typedef struct {
    Serial_Device_t *serial_dev;    // 指向共享的底层串口设备 (例如 g_esp32_serial)
    void *mutex;                    // 指向用于同步访问ESP32的共享互斥锁
    GPIO_TypeDef *reset_port;       // ESP32 复位引脚的GPIO端口
    uint16_t reset_pin;             // ESP32 复位引脚的GPIO引脚号
} ESP32_Shared_Device_t;

// 声明全局共享ESP32设备实例 (在 dev_config.c 中定义)
extern ESP32_Shared_Device_t g_esp32_dev;

/**
 * @brief ESP32 硬件相关初始化 (例如复位引脚)
 * @param dev 指向ESP32共享设备实例
 */
void ESP32_Hw_Init(ESP32_Shared_Device_t *dev);

/**
 * @brief ESP32 硬件复位
 * @param dev 指向ESP32共享设备实例
 */
void ESP32_Hw_Reset(ESP32_Shared_Device_t *dev);

/**
 * @brief 回调函数：解析AT+CIPSTAMAC?响应并保存MAC地址
 * @param response 响应字符串
 */
void get_mac_addr_callback(const char *response);

/**
 * @brief 回调函数：解析AT+CIPSTA?响应并保存IP地址
 * @param response 响应字符串
 */
void get_ip_addr_callback(const char *response);

/**
 * @brief 回调函数：解析AT+GMR响应并保存ESP32版本信息
 * @param response 响应字符串
 */
void get_version_callback(const char *response);

/**
 * @brief 回调函数：设置WiFi SSID和密码
 * @param response 响应字符串
 */
void set_ssid_password_callback(const char *response);

/**
 * @brief ESP32 AT 命令操作接口
 * 提供统一的AT命令发送和数据收发接口，用于WiFi和BLE功能。
 */
typedef struct {
    /**
     * @brief 发送 ESP32 AT 指令 (通用接口)
     * @param dev 指向ESP32共享设备实例
     * @param cmd AT 指令配置
     * @param type 通信类型 (WiFi/BLE)
     * @return AT_Status_t 操作状态
     */
    AT_Status_t (*SendATCommand)(ESP32_Shared_Device_t *dev, const AT_Cmd_Config_t *cmd, ESP32_Comm_Type_t type);

    /**
     * @brief 发送数据 (TCP/BLE透传数据)
     * @param dev 指向ESP32共享设备实例
     * @param data 要发送的数据缓冲区
     * @param length 数据长度
     * @param type 通信类型 (WiFi/BLE)
     * @return AT_Status_t 操作状态
     */
    AT_Status_t (*SendData)(ESP32_Shared_Device_t *dev, const uint8_t *data, uint16_t length, ESP32_Comm_Type_t type);

    /**
     * @brief 接收数据 (TCP/BLE透传数据)
     * @param dev 指向ESP32共享设备实例
     * @param buffer 数据存储缓冲区
     * @param length 期望接收的长度（输入），实际接收的长度（输出）
     * @param timeout_ms 超时时间（毫秒）
     * @param type 通信类型 (WiFi/BLE)
     * @return AT_Status_t 操作状态
     */
    AT_Status_t (*ReceiveData)(ESP32_Shared_Device_t *dev, uint8_t *buffer, uint16_t *length, uint32_t timeout_ms, ESP32_Comm_Type_t type);
} ESP32_AT_Ops_t;

// 声明全局唯一的ESP32 AT操作接口实例
extern const ESP32_AT_Ops_t g_esp32_at_ops;

#endif /* __API_WIFI_H */
