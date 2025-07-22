/**
 * =====================================================================================
 * @file        serial_interface.c
 * @brief       串口中间层接口实现，提供协议处理、错误日志和RS485帧队列管理
 * @author      23Elapse & Gemini
 * @version     2.1 (Refactored)
 * @date        2025-06-14
 * @note        本层整合了底层 serial_driver 和高层 protocol_handler，并管理队列。
 * =====================================================================================
 */
#include "serial_interface.h"
#include "rtos_abstraction.h"
#include "log_system.h"
#include "pch.h" // For pdTRUE, pdFALSE etc.
#include <string.h> // For memcpy, memset

// 错误日志缓冲区
static RingBuffer_t s_error_log_buffer;
// RS485 发送帧队列 (目前只支持一个队列，如果需要多RS485设备，应将其移入 Serial_Device_t 或更高级别管理)
static RS485_TxFrameQueue_t s_rs485_tx_queue = {0};
// serial_driver.h 或 serial_driver.c

// 注意：rx_data, rx_index, state, expected_length 等变量已移至 Protocol_Handler 内部管理，
// 或者应由调用者（例如 App_SerialRxTask）为每个串口设备实例单独维护，以实现真正的多实例并发。
// 在 Protocol_ProcessByte 的原始设计中，它们是静态的，因此在多串口场景下可能存在问题。
// 此处假设 Protocol_ProcessByte 内部已正确处理其状态或通过传递 context 避免冲突。

/*
 * =====================================================================================
 * 内部辅助函数
 * =====================================================================================
 */

/**
 * @brief 错误日志缓冲区初始化
 * @return Serial_Status_t 操作状态
 */
static Serial_Status_t _Serial_ErrorLog_Init(void)
{
    if (s_error_log_buffer.buffer == NULL) { // 避免重复初始化
        // 信号量在 RingBuffer_Init 内部创建
        if (RingBuffer_Init(&s_error_log_buffer, 16, sizeof(Serial_ErrorLog_t)) != RB_OK) {
            Log_Message(LOG_LEVEL_ERROR, "[Serial IF] Error log buffer init failed.");
            return SERIAL_ERR_INIT;
        }
    }
    Log_Message(LOG_LEVEL_INFO, "[Serial IF] Error log buffer initialized.");
    return SERIAL_OK;
}

/**
 * @brief 记录串口错误日志
 * @param type 错误类型
 * @param instance 发生错误的USART实例
 */
static void _Serial_LogError(Serial_ErrorType_t type, USART_TypeDef *instance)
{
    Serial_ErrorLog_t log_entry = {
        .type = type,
        .timestamp = g_rtos_ops->GetTickCount(),
        .instance = instance
    };
    // 非阻塞写入，如果缓冲区满则丢弃
    if (RingBuffer_Write(&s_error_log_buffer, &log_entry) != RB_OK) {
        Log_Message(LOG_LEVEL_WARNING, "[Serial IF] Failed to write error log (buffer full).");
    }
}

/**
 * @brief Modbus CRC16 计算函数
 * @param data 输入数据缓冲区
 * @param length 数据长度
 * @return 计算得到的CRC16值
 */
uint16_t Modbus_CRC16(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/*
 * =====================================================================================
 * 串口中间层操作接口实现
 * =====================================================================================
 */

/**
 * @brief 初始化串口中间层及底层驱动
 * @param dev 指向要初始化的串口设备结构体
 * @return Serial_Status_t 操作状态
 */
static Serial_Status_t Serial_Interface_Init(Serial_Device_t *dev)
{
    // 初始化错误日志系统
    if (_Serial_ErrorLog_Init() != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[Serial IF] Initialization failed: Error log init issue.");
        return SERIAL_ERR_INIT;
    }

    // 初始化 RS485 发送队列 (如果尚未初始化)
    if (s_rs485_tx_queue.mutex == NULL) {
        s_rs485_tx_queue.mutex = g_rtos_ops->SemaphoreCreate();
        if (s_rs485_tx_queue.mutex == NULL) {
            Log_Message(LOG_LEVEL_ERROR, "[Serial IF] Initialization failed: RS485 TX queue mutex creation failed.");
            return SERIAL_ERR_INIT;
        }
        s_rs485_tx_queue.head = 0;
        s_rs485_tx_queue.tail = 0;
        s_rs485_tx_queue.count = 0;
        Log_Message(LOG_LEVEL_INFO, "[Serial IF] RS485 TX queue initialized.");
    }

    // 初始化底层串口驱动
    Serial_Status_t status = Serial_Driver_Init(dev);
    if (status != SERIAL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[Serial IF] Initialization failed: Underlying serial driver init issue (%d).", status);
        return status;
    }
    Log_Message(LOG_LEVEL_INFO, "[Serial IF] Middleware for USART %p initialized successfully.", dev->instance);
    return SERIAL_OK;
}

/**
 * @brief 反初始化串口中间层及底层驱动
 * @param dev 指向要反初始化的串口设备结构体
 * @return Serial_Status_t 操作状态
 */
static Serial_Status_t Serial_Interface_Deinit(Serial_Device_t *dev)
{
    // 反初始化底层串口驱动
    Serial_Status_t status = Serial_Driver_Deinit(dev);
    if (status != SERIAL_OK) {
        Log_Message(LOG_LEVEL_WARNING, "[Serial IF] Deinitialization warning: Underlying serial driver deinit issue (%d).", status);
    }

    // 反初始化 RS485 发送队列互斥锁 (如果需要，并且是唯一使用者)
    // 通常，全局队列的互斥锁不应在这里释放，除非确定整个系统都不再使用。
    // if (s_rs485_tx_queue.mutex != NULL) {
    //     g_rtos_ops->SemaphoreDelete(s_rs485_tx_queue.mutex);
    //     s_rs485_tx_queue.mutex = NULL;
    // }

    Log_Message(LOG_LEVEL_INFO, "[Serial IF] Middleware for USART %p deinitialized.", dev->instance);
    return status;
}

/**
 * @brief 通过串口发送原始数据
 * @param dev 指向串口设备结构体
 * @param data 要发送的数据缓冲区
 * @param length 数据长度
 * @return Serial_Status_t 操作状态
 */
static Serial_Status_t Serial_Interface_SendData(Serial_Device_t *dev, const uint8_t *data, uint32_t length)
{
    Serial_Status_t status = Serial_Driver_SendData(dev, data, length);
    if (status != SERIAL_OK)
    {
        _Serial_LogError(SERIAL_ERROR_TIMEOUT, dev->instance); // 假设发送失败是超时导致
        Log_Message(LOG_LEVEL_ERROR, "[Serial IF] SendData: Failed for USART %p, status %d.", dev->instance, status);
    } else {
        // Log_Message(LOG_LEVEL_DEBUG, "[Serial IF] SendData: %lu bytes sent on USART %p.", length, dev->instance);
    }
    return status;
}

/**
 * @brief 通过串口发送一个RS485协议帧 (如果串口为RS485模式)
 * @param dev 指向串口设备结构体
 * @param frame 指向要发送的RS485帧结构体
 * @return Serial_Status_t 操作状态
 */
static Serial_Status_t Serial_Interface_SendFrame(Serial_Device_t *dev, const RS485_Frame_t *frame)
{
    if (!dev || !frame || dev->mode != RS485_MODE) {
        Log_Message(LOG_LEVEL_ERROR, "[Serial IF] SendFrame: Invalid parameters (dev/frame or not RS485 mode).");
        return SERIAL_ERR_PARAM;
    }

    uint8_t bytes[256]; // 临时缓冲区，用于存储帧的字节流
    uint32_t bytes_len;

    // 将 RS485_Frame_t 结构体转换为字节数组，并计算CRC
    // 这里需要一个辅助函数来完成这个转换，其应该在 RS485_Driver 内部，或者在此处实现。
    // 为了避免循环依赖，这里直接实现或假设存在一个通用的协议打包函数。
    // 假设 Protocol_Handler 提供了 Protocol_PackRS485Frame 函数
    Protocol_Status_t proto_pack_status = Protocol_PackRS485Frame(frame, bytes, sizeof(bytes), &bytes_len);

    if (proto_pack_status != PROTOCOL_OK) {
        Log_Message(LOG_LEVEL_ERROR, "[Serial IF] SendFrame: Failed to pack RS485 frame (Error: %d).", proto_pack_status);
        return SERIAL_ERR_TRANSMIT;
    }

    // 调用底层串口发送函数
    return Serial_Interface_SendData(dev, bytes, bytes_len);
}

/**
 * @brief 从串口接收缓冲区读取并解析协议数据
 * @param dev 指向串口设备结构体
 * @param data_out 指向存储接收到的协议数据结构体
 * @param timeout_ms 等待数据的超时时间 (毫秒)
 * @return Serial_Status_t 操作状态 (SERIAL_OK, SERIAL_ERR_NO_DATA, SERIAL_ERR_FRAME等)
 */
static Serial_Status_t Serial_Interface_ReceiveProtocolData(Serial_Device_t *dev, Protocol_Data_t *data_out, uint32_t timeout_ms)
{
    if (!dev || !data_out) {
        Log_Message(LOG_LEVEL_ERROR, "[Serial IF] ReceiveProtocolData: Invalid parameters.");
        return SERIAL_ERR_PARAM;
    }

    uint8_t byte;
    Protocol_Status_t proto_status;
    uint32_t start_tick = g_rtos_ops->GetTickCount();

    // Protocol_Handler 内部应维护其自身的解析状态机，以便支持多实例或连续流。
    // 如果 Protocol_ProcessByte 是无状态的或需要外部状态，则这里的实现需要调整。
    // 假设 Protocol_ProcessByte 内部是无状态的或通过 dev 传入上下文。
    
    // 每次开始接收新帧时，清零 Protocol_Data_t 结构体
    memset(data_out, 0, sizeof(Protocol_Data_t));
    
    while (true) {
        if ((g_rtos_ops->GetTickCount() - start_tick) >= timeout_ms) {
            Log_Message(LOG_LEVEL_DEBUG, "[Serial IF] ReceiveProtocolData: Timeout for USART %p.", dev->instance);
            return SERIAL_ERR_NO_DATA;
        }

        // 尝试从底层串口的环形缓冲区读取一个字节 (带短超时，避免忙等)
        Serial_Status_t read_status = Serial_Driver_ReadByte(dev, &byte);
        if (read_status == SERIAL_OK) {
            // 将字节送入协议解析器
            proto_status = Protocol_ProcessByte(dev, data_out, byte); // 假设 Protocol_ProcessByte 不需要外部索引/状态

            if (proto_status == PROTOCOL_OK) {
                // Log_Message(LOG_LEVEL_INFO, "[Serial IF] Received complete protocol data on USART %p.", dev->instance);
                return SERIAL_OK;
            } else if (proto_status == PROTOCOL_IN_PROGRESS) {
                // 帧解析进行中，继续等待
            } else { // 协议解析错误 (CRC, 帧格式等)
                if (proto_status == PROTOCOL_ERROR_CRC) {
                    _Serial_LogError(SERIAL_ERROR_CRC, dev->instance);
                    Log_Message(LOG_LEVEL_WARNING, "[Serial IF] Protocol error (CRC) on USART %p. Resetting parser.", dev->instance);
                } else if (proto_status == PROTOCOL_ERROR_FRAME) {
                    _Serial_LogError(SERIAL_ERROR_FRAME_ERR, dev->instance);
                    Log_Message(LOG_LEVEL_WARNING, "[Serial IF] Protocol error (Frame) on USART %p. Resetting parser.", dev->instance);
                }
                // 重置 Protocol_Handler 内部状态 (如果需要)
                Protocol_ResetParser(dev); // 假设 Protocol_Handler 有复位接口
                // 清零输出结构体，准备接收下一帧
                memset(data_out, 0, sizeof(Protocol_Data_t));
                // 可以选择返回错误或继续等待下一帧。这里选择继续等待。
                // return SERIAL_ERR_FRAME;
            }
        } else if (read_status == SERIAL_ERR_NO_DATA) {
            // 缓冲区空，短暂延时让出CPU
            g_rtos_ops->Delay(1);
        } else {
            // 其他底层串口读取错误
            Log_Message(LOG_LEVEL_ERROR, "[Serial IF] ReceiveProtocolData: Underlying serial read error (%d) on USART %p.", read_status, dev->instance);
            _Serial_LogError(SERIAL_ERROR_FRAME_ERR, dev->instance); // 记录错误
            return read_status; // 返回底层错误
        }
    }
}

/**
 * @brief 获取一条错误日志
 * @param log_entry 指向存储错误日志的结构体
 * @param timeout_ms 等待日志的超时时间 (毫秒)
 * @return Serial_Status_t 操作状态 (SERIAL_OK 或 SERIAL_ERR_NO_DATA)
 */
static Serial_Status_t Serial_Interface_GetErrorLog(Serial_ErrorLog_t *log_entry, uint32_t timeout_ms)
{
    if (!log_entry) {
        return SERIAL_ERR_PARAM;
    }
    // 从错误日志环形缓冲区读取，带超时
    RB_Status rb_status = RingBuffer_Read(&s_error_log_buffer, log_entry);
    if (rb_status == RB_OK) {
        return SERIAL_OK;
    } else if (rb_status == RB_ERROR_BUFFER_EMPTY) {
        return SERIAL_ERR_NO_DATA;
    } else {
        // 其他环形缓冲区错误，例如初始化问题
        Log_Message(LOG_LEVEL_ERROR, "[Serial IF] GetErrorLog: Ring buffer read error (%d).", rb_status);
        return SERIAL_ERR_BUFFER_FULL; // 或更合适的错误码
    }
}

/**
 * @brief 将一个RS485协议帧添加到指定串口的发送队列中
 * @param dev 指向RS485模式的串口设备结构体
 * @param frame 指向要添加的RS485帧结构体
 * @return Serial_Status_t 操作状态
 */
static Serial_Status_t Serial_Interface_AddFrameToQueue(Serial_Device_t *dev, const RS485_Frame_t *frame)
{
    if (!dev || dev->mode != RS485_MODE || !frame) {
        Log_Message(LOG_LEVEL_ERROR, "[Serial IF] AddFrameToQueue: Invalid parameters (dev/frame or not RS485 mode).");
        return SERIAL_ERR_PARAM;
    }

    // 获取队列互斥锁
    if (g_rtos_ops->SemaphoreTake(s_rs485_tx_queue.mutex, 100) != pdTRUE) { // 短暂超时
        Log_Message(LOG_LEVEL_WARNING, "[Serial IF] AddFrameToQueue: Failed to get queue mutex (timeout).");
        return SERIAL_ERR_MUTEX_TIMEOUT;
    }

    if (s_rs485_tx_queue.count >= RS485_TX_QUEUE_SIZE) {
        Log_Message(LOG_LEVEL_WARNING, "[Serial IF] RS485 TX queue for USART %p is full.", dev->instance);
        g_rtos_ops->SemaphoreGive(s_rs485_tx_queue.mutex);
        return SERIAL_ERR_BUFFER_FULL;
    }

    RS485_Frame_t *queued_frame = &s_rs485_tx_queue.frames[s_rs485_tx_queue.tail];
    // 复制帧的基本信息
    memcpy(queued_frame, frame, sizeof(RS485_Frame_t));
    // 复制信息域数据到独立的缓冲区，并更新指针
    if (frame->info_len > 0 && frame->info != NULL) {
        if (frame->info_len > sizeof(s_rs485_tx_queue.info_buffers[0])) {
            Log_Message(LOG_LEVEL_ERROR, "[Serial IF] AddFrameToQueue: Frame info_len %u too large for buffer.", frame->info_len);
            g_rtos_ops->SemaphoreGive(s_rs485_tx_queue.mutex);
            return SERIAL_ERR_BUFFER_FULL;
        }
        memcpy(s_rs485_tx_queue.info_buffers[s_rs485_tx_queue.tail], frame->info, frame->info_len);
        queued_frame->info = s_rs485_tx_queue.info_buffers[s_rs485_tx_queue.tail]; // 指向新分配的缓冲区
    } else {
        queued_frame->info = NULL;
        queued_frame->info_len = 0;
    }

    s_rs485_tx_queue.tail = (s_rs485_tx_queue.tail + 1) % RS485_TX_QUEUE_SIZE;
    s_rs485_tx_queue.count++;

    g_rtos_ops->SemaphoreGive(s_rs485_tx_queue.mutex);
    Log_Message(LOG_LEVEL_DEBUG, "[Serial IF] Frame queued for USART %p. Current count: %u.", dev->instance, s_rs485_tx_queue.count);
    return SERIAL_OK;
}

/**
 * @brief 轮询指定RS485模式串口的发送队列，发送待发送的帧
 * @param dev 指向RS485模式的串口设备结构体
 */
static void Serial_Interface_PollSendRS485(Serial_Device_t *dev)
{
    if (!dev || dev->mode != RS485_MODE || s_rs485_tx_queue.count == 0) {
        return; // 无效设备、非RS485模式或队列为空
    }

    // 尝试获取队列互斥锁 (非阻塞或短超时，避免卡住轮询任务)
    if (g_rtos_ops->SemaphoreTake(s_rs485_tx_queue.mutex, 0) != pdTRUE) { // 0表示非阻塞
        return; // 如果无法获取锁，等待下次轮询
    }

    RS485_Frame_t *frame_to_send = &s_rs485_tx_queue.frames[s_rs485_tx_queue.head];

    // 发送帧 (调用 SendFrame，它会处理底层发送和DE/RE控制)
    Serial_Status_t send_status = Serial_Interface_SendFrame(dev, frame_to_send);

    if (send_status == SERIAL_OK) {
        // 发送成功，更新队列头指针和计数
        s_rs485_tx_queue.head = (s_rs485_tx_queue.head + 1) % RS485_TX_QUEUE_SIZE;
        s_rs485_tx_queue.count--;
        // Log_Message(LOG_LEVEL_DEBUG, "[Serial IF] PollSend: Frame sent. Remaining: %u.", s_rs485_tx_queue.count);

        // RS485发送后通常需要一个总线静默时间，在 Serial_Driver_SendData 内部已处理。
        // dev->silent_ticks 可以在这里用于额外的延时，但通常 TC 标志和底层DE控制已足够。
        // g_rtos_ops->Delay(dev->silent_ticks);
    } else {
        // 发送失败，记录错误。可以选择重试当前帧，或跳过。
        // 为了避免阻塞轮询，通常不在此处重试，让下一次轮询重新处理。
        Log_Message(LOG_LEVEL_WARNING, "[Serial IF] PollSend: Failed to send frame for USART %p. Status: %d.", dev->instance, send_status);
        _Serial_LogError(SERIAL_ERROR_FRAME_ERR, dev->instance);
    }

    g_rtos_ops->SemaphoreGive(s_rs485_tx_queue.mutex); // 释放互斥锁
}


// 定义全局唯一的串口中间层操作接口实例
const Serial_Ops_t g_serial_ops = {
    .Init = Serial_Interface_Init,
    .Deinit = Serial_Interface_Deinit,
    .SendData = Serial_Interface_SendData,
    .SendFrame = Serial_Interface_SendFrame,
    .ReceiveProtocolData = Serial_Interface_ReceiveProtocolData,
    .GetErrorLog = Serial_Interface_GetErrorLog,
    .AddFrameToQueue = Serial_Interface_AddFrameToQueue,
    .PollSendRS485 = Serial_Interface_PollSendRS485,
};
