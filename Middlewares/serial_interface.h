/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-05-02 21:30:06
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-06-14 19:02:03
 * @FilePath: \Demo_backup\Middlewares\serial_interface.h
 * @Description: 串口中间层接口，提供协议处理、错误日志和RS485帧队列管理
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#ifndef __SERIAL_INTERFACE_H
#define __SERIAL_INTERFACE_H

#include "serial_driver.h"    // 依赖底层串口驱动
#include "protocol_handler.h" // 依赖协议处理模块

/**
 * @brief 错误日志类型枚举
 */
typedef enum
{
    SERIAL_ERROR_CRC = 0,    // CRC校验错误
    SERIAL_ERROR_TIMEOUT,    // 接收超时
    SERIAL_ERROR_FRAME_ERR   // 帧格式错误
} Serial_ErrorType_t;

/**
 * @brief 错误日志结构体
 */
typedef struct
{
    Serial_ErrorType_t type;    // 错误类型
    uint32_t timestamp;         // 错误发生的时间戳 (RTOS TickCount)
    USART_TypeDef *instance;    // 发生错误的USART实例
} Serial_ErrorLog_t;

// RS485 发送帧队列大小
#define RS485_TX_QUEUE_SIZE 10

/**
 * @brief RS485 发送帧队列结构体
 * @note  用于存储待发送的RS485协议帧
 */
typedef struct
{
    RS485_Frame_t frames[RS485_TX_QUEUE_SIZE];        // 存储协议帧
    uint8_t info_buffers[RS485_TX_QUEUE_SIZE][200]; // 存储每个帧的信息域数据
    uint8_t head;   // 队列头索引
    uint8_t tail;   // 队列尾索引
    uint8_t count;  // 队列中当前帧的数量
    void* mutex;    // 用于保护队列访问的互斥锁
} RS485_TxFrameQueue_t;

/**
 * @brief 串口中间层操作接口
 * @note  提供统一的串口初始化、数据收发、协议帧收发及错误日志管理。
 * 本层将底层 serial_driver 和高层 protocol_handler 进行整合。
 */
typedef struct
{
    /**
     * @brief 初始化串口中间层及底层驱动
     * @param dev 指向要初始化的串口设备结构体
     * @return Serial_Status_t 操作状态
     */
    Serial_Status_t (*Init)(Serial_Device_t *dev);

    /**
     * @brief 反初始化串口中间层及底层驱动
     * @param dev 指向要反初始化的串口设备结构体
     * @return Serial_Status_t 操作状态
     */
    Serial_Status_t (*Deinit)(Serial_Device_t *dev);

    /**
     * @brief 通过串口发送原始数据
     * @param dev 指向串口设备结构体
     * @param data 要发送的数据缓冲区
     * @param length 数据长度
     * @return Serial_Status_t 操作状态
     */
    Serial_Status_t (*SendData)(Serial_Device_t *dev, const uint8_t *data, uint32_t length);

    /**
     * @brief 通过串口发送一个RS485协议帧 (如果串口为RS485模式)
     * @param dev 指向串口设备结构体
     * @param frame 指向要发送的RS485帧结构体
     * @return Serial_Status_t 操作状态
     */
    Serial_Status_t (*SendFrame)(Serial_Device_t *dev, const RS485_Frame_t *frame);

    /**
     * @brief 从串口接收缓冲区读取并解析协议数据
     * @param dev 指向串口设备结构体
     * @param data_out 指向存储接收到的协议数据结构体
     * @param timeout_ms 等待数据的超时时间 (毫秒)
     * @return Serial_Status_t 操作状态 (SERIAL_OK, SERIAL_ERR_NO_DATA, SERIAL_ERR_FRAME等)
     */
    Serial_Status_t (*ReceiveProtocolData)(Serial_Device_t *dev, Protocol_Data_t *data_out, uint32_t timeout_ms);

    /**
     * @brief 获取一条错误日志
     * @param log_entry 指向存储错误日志的结构体
     * @param timeout_ms 等待日志的超时时间 (毫秒)
     * @return Serial_Status_t 操作状态 (SERIAL_OK 或 SERIAL_ERR_NO_DATA)
     */
    Serial_Status_t (*GetErrorLog)(Serial_ErrorLog_t *log_entry, uint32_t timeout_ms);

    /**
     * @brief 将一个RS485协议帧添加到指定串口的发送队列中
     * @param dev 指向RS485模式的串口设备结构体
     * @param frame 指向要添加的RS485帧结构体
     * @return Serial_Status_t 操作状态
     */
    Serial_Status_t (*AddFrameToQueue)(Serial_Device_t *dev, const RS485_Frame_t *frame);

    /**
     * @brief 轮询指定RS485模式串口的发送队列，发送待发送的帧
     * @param dev 指向RS485模式的串口设备结构体
     */
    void (*PollSendRS485)(Serial_Device_t *dev);

} Serial_Ops_t;

// 声明全局唯一的串口中间层操作接口实例
extern const Serial_Ops_t g_serial_ops;

/**
 * @brief Modbus CRC16 计算函数
 * @param data 输入数据缓冲区
 * @param length 数据长度
 * @return 计算得到的CRC16值
 */
uint16_t Modbus_CRC16(const uint8_t *data, uint32_t length);

#endif /* __SERIAL_INTERFACE_H */
