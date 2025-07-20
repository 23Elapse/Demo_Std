/**
 * =====================================================================================
 * @file        rs485_driver.h
 * @brief       RS485协议层驱动头文件
 * @author      23Elapse & Gemini
 * @version     2.2 (Refactored based on serial_interface)
 * @date        2025-06-14
 * @note        本驱动工作在串口应用层接口之上，提供RS485协议相关的封装。
 * =====================================================================================
 */
#ifndef __RS485_DRIVER_H
#define __RS485_DRIVER_H

#include "serial_driver.h" // 依赖底层串口驱动定义
#include "serial_interface.h" // 依赖串口应用层接口

/**
 * @brief RS485 操作状态枚举
 */
typedef enum {
    RS485_OK = 0,
    RS485_ERR_PARAM,         // 参数错误
    RS485_ERR_INIT,          // 初始化失败
    RS485_ERR_TRANSMIT,      // 传输失败 (底层串口发送失败)
    RS485_ERR_BUFFER_FULL,   // 发送队列满
    RS485_ERR_MUTEX_TIMEOUT,  // 获取互斥锁超时
    RS485_ERR_FRAME,         // 协议帧错误 (如CRC校验失败)
    RS485_ERR_NO_DATA,       // 没有接收到数据
    
} RS485_Status_t;

/**
 * @brief RS485 设备结构体
 * @note  封装了RS485所使用的底层串口设备
 */
typedef struct {
    Serial_Device_t* serial_dev; // 指向用于RS485通信的底层串口设备实例
    // 可以添加一些RS485特有的配置，例如：
    // uint32_t response_timeout_ms; // 等待从机响应的超时时间
} RS485_Device_t;

/**
 * @brief RS485 操作接口
 * @note  提供统一的RS485协议层操作函数指针集合
 */
typedef struct {
    /**
     * @brief 初始化RS485设备（逻辑层）
     * @param dev 指向RS485设备实例
     * @return RS485_Status_t 操作状态
     * @note  底层串口的硬件初始化应在更早的应用层（如App_Driver_Init）完成。
     */
    RS485_Status_t (*Init)(RS485_Device_t* dev);

    /**
     * @brief 通过RS485发送原始数据（不封装成协议帧）
     * @param dev 指向RS485设备实例
     * @param data 指向要发送的数据缓冲区
     * @param len 要发送的数据长度
     * @return RS485_Status_t 操作状态
     */
    RS485_Status_t (*SendData)(RS485_Device_t* dev, const uint8_t* data, uint16_t len);

    /**
     * @brief 通过RS485发送一个封装好的协议帧
     * @param dev 指向RS485设备实例
     * @param frame 指向要发送的RS485帧结构体
     * @return RS485_Status_t 操作状态
     */
    RS485_Status_t (*SendFrame)(RS485_Device_t* dev, const RS485_Frame_t* frame);

    /**
     * @brief 从RS485接收一个协议帧（阻塞，带超时）
     * @param dev 指向RS485设备实例
     * @param data_out 指向存储接收到的协议数据结构体
     * @param timeout_ms 超时时间（毫秒）
     * @return RS485_Status_t 操作状态
     */
    RS485_Status_t (*ReceiveFrame)(RS485_Device_t* dev, Protocol_Data_t* data_out, uint32_t timeout_ms);

    /**
     * @brief 将一个RS485协议帧添加到发送队列中，以供轮询任务发送（非阻塞）
     * @param dev 指向RS485设备实例
     * @param frame 指向要发送的RS485帧结构体
     * @return RS485_Status_t 操作状态
     */
    RS485_Status_t (*QueueFrameForSend)(RS485_Device_t* dev, const RS485_Frame_t* frame);

    /**
     * @brief 轮询RS485发送队列，发送待发送的帧
     * @param dev 指向RS485设备实例
     */
    void (*PollSendQueue)(RS485_Device_t* dev);

} RS485_Ops_t;

// 声明全局唯一的RS485操作接口实例
extern const RS485_Ops_t g_rs485_ops;

#endif // __RS485_DRIVER_H
