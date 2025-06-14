/**
 * =====================================================================================
 * @file        rs485_driver.c
 * @brief       RS485应用层接口实现 (已修正)
 * @author      23Elapse & Gemini
 * @version     2.1 (Corrected based on serial_interface)
 * @date        2025-06-08
 * @note        本文件提供调用高层串行接口(Serial_Operations)的封装，以适应项目架构。
 * RS485的DE/RE时序控制和发送轮询由底层驱动和上层任务负责。
 * =====================================================================================
 */
#include "rs485_driver.h"
#include "serial_interface.h" // 关键：现在依赖于高层接口
#include "log_system.h"
#include <string.h>

/**
 * @brief 初始化RS485设备（逻辑层）
 * @note  本函数仅为逻辑初始化，确保底层串口已由App_Driver_Init初始化。
 * 并且RS485的DE引脚应在底层serial_driver中管理。
 * @param serial_dev 指向用于RS485通信的底层串口设备实例
 * @return RS485_Status_t 操作状态
 */
RS485_Status_t RS485_Init(Serial_Device_t* serial_dev) {
    if (!serial_dev) {
        Log_Message(LOG_LEVEL_ERROR, "Error: RS485_Init received null serial device.");
        return RS485_ERR_PARAM;
    }

    // 检查是否为RS485模式
    if (serial_dev->mode != RS485_MODE) {
        Log_Message(LOG_LEVEL_WARNING, "Warning: Device passed to RS485_Init is not in RS485_MODE.");
    }
    
    // 底层串口的GPIO, NVIC, USART等初始化由更早的 App_Driver_Init -> Serial_Operations.Init 完成
    // 此处无需重复操作。只需打印日志表明逻辑层已准备就绪。
    Log_Message(LOG_LEVEL_INFO, "[RS485] Protocol layer initialized for USART instance %p.", serial_dev->instance);
    return RS485_OK;
}

/**
 * @brief 将一个RS485帧添加到发送队列中，以供轮询任务发送（非阻塞）
 * @param frame 指向要发送的RS485帧结构体
 * @return RS485_Status_t 操作状态
 */
RS485_Status_t RS485_QueueFrameForSend(RS485_Frame_t* frame) {
    if (!frame) {
        return RS485_ERR_PARAM;
    }
    
    // 正确的做法是调用 serial_interface 中提供的队列添加函数
    Serial_Status status = Serial_Operations.AddFrameToQueue(frame);

    if (status == SERIAL_ERR_BUFFER_FULL) {
        Log_Message(LOG_LEVEL_WARNING, "[RS485] Transmit queue is full.");
        return RS485_ERR_BUFFER_FULL;
    }
    
    if (status != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "Error: Failed to queue RS485 frame, status: %d", status);
        return RS485_ERR_TRANSMIT;
    }

    return RS485_OK;
}

/**
 * @brief 【不推荐】直接发送一个RS485帧（阻塞/同步方式）
 * @note  此函数绕过了项目的轮询发送队列机制，仅用于特殊调试场景。
 * 常规应用应使用 RS485_QueueFrameForSend。
 * DE/RE引脚的控制依赖于底层 Serial_Driver_SendData 的正确实现。
 * @param serial_dev 指向底层串口设备实例
 * @param frame 指向要发送的RS485帧结构体
 * @return RS485_Status_t 操作状态
 */
RS485_Status_t RS485_SendFrameImmediately(Serial_Device_t* serial_dev, RS485_Frame_t* frame) {
    if (!serial_dev || !frame) {
        return RS485_ERR_PARAM;
    }

    // 直接调用 serial_interface 提供的帧发送函数
    Serial_Status status = Serial_Operations.SendFrame(serial_dev, frame);

    if (status != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "Error: RS485 immediate send failed with status %d", status);
        return RS485_ERR_TRANSMIT;
    }

    return RS485_OK;
}
