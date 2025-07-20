/**
 * =====================================================================================
 * @file        rs485_driver.c
 * @brief       RS485应用层接口实现 (Refactored)
 * @author      23Elapse & Gemini
 * @version     2.2 (Refactored based on serial_interface)
 * @date        2025-06-14
 * @note        本文件提供调用高层串行接口(g_serial_ops)的封装，以适应项目架构。
 * RS485的DE/RE引脚控制和发送轮询由底层串口驱动和serial_interface负责。
 * =====================================================================================
 */
#include "rs485_driver.h"
#include "serial_interface.h" // 关键：现在依赖于高层接口 g_serial_ops
#include "log_system.h"
#include <string.h> // For memcpy (used indirectly in frame handling)

// 注意：原有的 RS485_FrameToBytes 和 Modbus_CRC16 内部函数已移至 protocol_handler.c，
// 并在 protocol_handler.h 中声明为公共函数。此处不再需要它们的重复定义。

/*
 * =====================================================================================
 * RS485 操作接口实现
 * =====================================================================================
 */

/**
 * @brief 初始化RS485设备（协议逻辑层）
 * @note  本函数仅为逻辑初始化，确保底层串口已由App_Driver_Init初始化。
 * 并且RS485的DE引脚应在底层Serial_Driver中管理。
 * @param dev 指向用于RS485通信的底层串口设备实例
 * @return RS485_Status_t 操作状态
 */
static RS485_Status_t RS485_Driver_Init(RS485_Device_t* dev) {
    if (!dev || !dev->serial_dev) {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] Init: Received null serial device pointer.");
        return RS485_ERR_PARAM;
    }

    // 验证传入的设备是否配置为RS485模式
    if (dev->serial_dev->mode != RS485_MODE) {
        Log_Message(LOG_LEVEL_WARNING, "[RS485] Init: Device %p is not configured for RS485_MODE. Check dev_config.", dev->serial_dev->instance);
        // 虽然不是错误，但可能是配置问题，记录警告
    }
    
    // 底层串口的GPIO, NVIC, USART等初始化由更早的 App_Driver_Init -> g_serial_ops.Init 完成
    // 这里只需打印日志，表示RS485协议层已准备就绪。
    Log_Message(LOG_LEVEL_INFO, "[RS485] Protocol layer initialized for USART instance %p.", dev->serial_dev->instance);
    return RS485_OK;
}

/**
 * @brief 通过RS485发送原始数据（不封装成协议帧）
 * @param dev 指向RS485设备实例
 * @param data 指向要发送的数据缓冲区
 * @param len 要发送的数据长度
 * @return RS485_Status_t 操作状态
 */
static RS485_Status_t RS485_Driver_SendData(RS485_Device_t* dev, const uint8_t* data, uint16_t len) {
    if (!dev || !dev->serial_dev || !data || len == 0) {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] SendData: Invalid parameters.");
        return RS485_ERR_PARAM;
    }

    // 直接调用串口应用层接口发送函数，其内部会调用底层串口发送并处理DE/RE控制
    Serial_Status_t status = g_serial_ops.SendData(dev->serial_dev, data, len);
    if (status != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] SendData: Failed to send data via underlying serial (%d).", status);
        return RS485_ERR_TRANSMIT;
    }
    Log_Message(LOG_LEVEL_DEBUG, "[RS485] Sent %u raw bytes.", len);
    return RS485_OK;
}

/**
 * @brief 通过RS485发送一个封装好的协议帧
 * @param dev 指向RS485设备实例
 * @param frame 指向要发送的RS485帧结构体
 * @return RS485_Status_t 操作状态
 */
static RS485_Status_t RS485_Driver_SendFrame(RS485_Device_t* dev, const RS485_Frame_t* frame) {
    if (!dev || !dev->serial_dev || !frame) {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] SendFrame: Invalid parameters.");
        return RS485_ERR_PARAM;
    }

    // 调用串口应用层接口发送帧函数，其内部会处理帧打包、CRC计算和底层发送
    Serial_Status_t status = g_serial_ops.SendFrame(dev->serial_dev, (RS485_Frame_t*)frame); // 强制类型转换，因为SendFrame接收非const

    if (status != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] SendFrame: Failed to send frame via underlying serial (%d).", status);
        return RS485_ERR_TRANSMIT;
    }
    Log_Message(LOG_LEVEL_INFO, "[RS485] Sent frame: Cmd=0x%02X, InfoLen=%u.", frame->cmd, frame->info_len);
    return RS485_OK;
}

/**
 * @brief 从RS485接收一个协议帧（阻塞，带超时）
 * @param dev 指向RS485设备实例
 * @param data_out 指向存储接收到的协议数据结构体
 * @param timeout_ms 超时时间（毫秒）
 * @return RS485_Status_t 操作状态
 */
static RS485_Status_t RS485_Driver_ReceiveFrame(RS485_Device_t* dev, Protocol_Data_t* data_out, uint32_t timeout_ms) {
    if (!dev || !dev->serial_dev || !data_out) {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] ReceiveFrame: Invalid parameters.");
        return RS485_ERR_PARAM;
    }

    // 调用串口应用层接口接收并解析协议数据
    Serial_Status_t serial_status = g_serial_ops.ReceiveProtocolData(dev->serial_dev, data_out, timeout_ms);

    if (serial_status == SERIAL_OK) {
        if (data_out->is_rs485) { // 确认是RS485帧
            Log_Message(LOG_LEVEL_INFO, "[RS485] Received frame: Addr1=0x%02X, Cmd=0x%02X, InfoLen=%u.",
                        data_out->rs485_frame.addr1, data_out->rs485_frame.cmd, data_out->rs485_frame.info_len);
            return RS485_OK;
        } else {
            // 如果解析成功但不是RS485帧，则可能是有其他协议数据，但此处我们只关注RS485
            Log_Message(LOG_LEVEL_DEBUG, "[RS485] Received non-RS485 protocol data. Ignoring.");
            // 可以选择返回 PROTOCOL_IN_PROGRESS 或 PROTOCOL_ERROR_FRAME
            return RS485_ERR_FRAME; // 标记为帧错误，表示不是期望的RS485帧
        }
    } else if (serial_status == SERIAL_ERR_NO_DATA) {
        Log_Message(LOG_LEVEL_WARNING, "[RS485] ReceiveFrame: No data received within timeout (%lu ms).", timeout_ms);
        return RS485_ERR_NO_DATA;
    } else if (serial_status == SERIAL_ERR_FRAME) {
        Log_Message(LOG_LEVEL_WARNING, "[RS485] ReceiveFrame: Frame parsing error on underlying serial (%d).", serial_status);
        return RS485_ERR_FRAME; // 映射底层帧错误
    } else {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] ReceiveFrame: Underlying serial error (%d).", serial_status);
        return RS485_ERR_TRANSMIT; // 映射底层传输错误
    }
}

/**
 * @brief 将一个RS485协议帧添加到发送队列中，以供轮询任务发送（非阻塞）
 * @param dev 指向RS485设备实例
 * @param frame 指向要发送的RS485帧结构体
 * @return RS485_Status_t 操作状态
 */
static RS485_Status_t RS485_Driver_QueueFrameForSend(RS485_Device_t* dev, const RS485_Frame_t* frame) {
    if (!dev || !dev->serial_dev || !frame) {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] QueueFrameForSend: Invalid parameters.");
        return RS485_ERR_PARAM;
    }

    // 调用串口应用层接口的队列添加函数
    Serial_Status_t status = g_serial_ops.AddFrameToQueue((Serial_Device_t *)dev->serial_dev, (RS485_Frame_t*)frame); // 强制类型转换，因为AddFrameToQueue接收非const

    if (status == SERIAL_ERR_BUFFER_FULL) {
        Log_Message(LOG_LEVEL_WARNING, "[RS485] Transmit queue for USART %p is full.", dev->serial_dev->instance);
        return RS485_ERR_BUFFER_FULL;
    }

    if (status != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[RS485] Failed to queue RS485 frame for USART %p, status: %d.", dev->serial_dev->instance, status);
        return RS485_ERR_TRANSMIT;
    }

    Log_Message(LOG_LEVEL_DEBUG, "[RS485] Frame queued for USART %p: Cmd=0x%02X.", dev->serial_dev->instance, frame->cmd);
    return RS485_OK;
}

/**
 * @brief 轮询RS485发送队列，发送待发送的帧
 * @param dev 指向RS485设备实例
 */
static void RS485_Driver_PollSendQueue(RS485_Device_t* dev) {
    if (!dev || !dev->serial_dev || dev->serial_dev->mode != RS485_MODE) {
        return; // 无效设备或非RS485模式
    }

    // 调用串口应用层接口的轮询发送函数
    g_serial_ops.PollSendRS485(dev->serial_dev);
}

// 定义全局唯一的RS485操作接口实例
const RS485_Ops_t g_rs485_ops = {
    .Init = RS485_Driver_Init,
    .SendData = RS485_Driver_SendData,
    .SendFrame = RS485_Driver_SendFrame,
    .ReceiveFrame = RS485_Driver_ReceiveFrame,
    .QueueFrameForSend = RS485_Driver_QueueFrameForSend,
    .PollSendQueue = RS485_Driver_PollSendQueue,
};
