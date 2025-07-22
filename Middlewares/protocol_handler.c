/**
 * =====================================================================================
 * @file        protocol_handler.c
 * @brief       通用协议解析器实现，提供RS485帧处理功能。
 * @author      23Elapse & Gemini
 * @version     2.0 (Refactored)
 * @date        2025-06-14
 * @note        本模块处理协议帧的打包和解包，提供可重入的字节流解析器。
 * =====================================================================================
 */
#include "protocol_handler.h"
#include "log_system.h" // For logging
#include <string.h>     // For memcpy, memset

// 注意：原有的 static Protocol_HandlerEntry_t handlers 和 handler_count
// 是用于多协议分发机制的。由于当前 Protocol_ProcessByte 只实现RS485解析，
// 且该机制在原代码的ProcessByte中并未完全实现分发逻辑，
// 为简化并与重入性要求兼容，此处暂时不使用这些静态注册表。
// 如果未来需要支持多种帧头对应的不同协议，可以重新设计Protocol_ProcessByte
// 以使用一个 Protocol_Manager 或类似的结构来管理并分发给不同的解析函数。

// /**
//  * @brief Modbus CRC16 计算函数
//  * @param data 输入数据缓冲区
//  * @param length 数据长度
//  * @return 计算得到的CRC16值
//  */
// uint16_t Modbus_CRC16(const uint8_t *data, uint32_t length)
// {
//     uint16_t crc = 0xFFFF;
//     for (uint32_t i = 0; i < length; i++)
//     {
//         crc ^= data[i];
//         for (uint8_t j = 0; j < 8; j++)
//         {
//             if (crc & 0x0001)
//             {
//                 crc >>= 1;
//                 crc ^= 0xA001; // Modbus-RTU 多项式
//             }
//             else
//             {
//                 crc >>= 1;
//             }
//         }
//     }
//     return crc;
// }

/**
 * @brief 协议模块初始化
 * @note  执行任何全局或一次性初始化。目前无需特定初始化。
 */
void Protocol_Init(void)
{
    // 例如：可以初始化日志系统中的协议相关模块，或者注册一些默认的协议处理句柄。
    // Protocol_RegisterHandler(FRAME_HEADER_7E, NULL); // 示例：如果需要基于帧头分发
    // Log_Message(LOG_LEVEL_INFO, "[Protocol] Initialized.");
}

/**
 * @brief 将RS485帧结构体打包成字节流，并计算CRC。
 * @param frame_in 指向要打包的RS485帧结构体
 * @param buffer_out 存储打包后字节流的缓冲区
 * @param max_buffer_len 缓冲区最大长度
 * @param packed_len_out 打包后实际字节长度输出
 * @return Protocol_Status_t 操作状态 (PROTOCOL_OK 或错误码)
 */
Protocol_Status_t Protocol_PackRS485Frame(const RS485_Frame_t *frame_in, uint8_t *buffer_out, uint32_t max_buffer_len, uint32_t *packed_len_out)
{
    if (!frame_in || !buffer_out || !packed_len_out) {
        Log_Message(LOG_LEVEL_ERROR, "[Protocol] PackRS485Frame: Invalid parameters.");
        return PROTOCOL_ERROR_PARAM;
    }

    // 计算信息域长度是否合法
    if (frame_in->info_len > PROTOCOL_MAX_INFO_LEN) {
        Log_Message(LOG_LEVEL_ERROR, "[Protocol] PackRS485Frame: Info length %u exceeds max %u.", frame_in->info_len, PROTOCOL_MAX_INFO_LEN);
        return PROTOCOL_ERROR_PARAM;
    }

    // 计算总帧长度
    uint16_t total_frame_len = PROTOCOL_MIN_FRAME_LEN + frame_in->info_len;

    // 检查缓冲区是否足够大
    if (total_frame_len > max_buffer_len) {
        Log_Message(LOG_LEVEL_ERROR, "[Protocol] PackRS485Frame: Buffer too small (%u) for frame of size %u.", max_buffer_len, total_frame_len);
        return PROTOCOL_ERROR_BUFFER_FULL;
    }

    // 填充帧头和固定部分
    buffer_out[0] = frame_in->sof;
    buffer_out[1] = frame_in->addr1;
    buffer_out[2] = frame_in->addr2;
    buffer_out[3] = frame_in->cmd;
    buffer_out[4] = frame_in->cmd_sub;
    buffer_out[5] = frame_in->info_len; // 协议中的LENGTH字段是信息域的长度

    // 填充信息域
    if (frame_in->info_len > 0 && frame_in->info != NULL) {
        memcpy(&buffer_out[6], frame_in->info, frame_in->info_len);
    } else if (frame_in->info_len > 0) { // info_len > 0 但 info 为 NULL
        Log_Message(LOG_LEVEL_WARNING, "[Protocol] PackRS485Frame: Info length is non-zero but info pointer is NULL.");
        // 可以选择在这里返回错误，或者填充零
        memset(&buffer_out[6], 0, frame_in->info_len);
    }

    // 计算CRC16 (从SOF到信息域结束)
    uint16_t crc = Modbus_CRC16(buffer_out, 6 + frame_in->info_len);
    buffer_out[6 + frame_in->info_len] = crc & 0xFF;        // CRC低字节
    buffer_out[6 + frame_in->info_len + 1] = (crc >> 8) & 0xFF; // CRC高字节

    *packed_len_out = total_frame_len;
    // Log_Message(LOG_LEVEL_DEBUG, "[Protocol] Packed RS485 frame. Len: %u", total_frame_len);
    return PROTOCOL_OK;
}


/**
 * @brief 逐字节解析接收到的数据，构建协议帧。
 * @param context 指向协议解析上下文，用于维护状态和存储中间数据。
 * @param byte_in 接收到的单个字节
 * @param data_out 指向存储解析结果的 Protocol_Data_t 结构体。如果解析成功，此结构体将被填充。
 * @return Protocol_Status_t 解析状态 (PROTOCOL_OK表示一帧完成，PROTOCOL_IN_PROGRESS表示正在解析中，其他为错误)
 * @note  此函数是可重入的，通过传入 context 来管理状态。
 */
Protocol_Status_t Protocol_ProcessByte(Protocol_ParserContext_t *context, uint8_t byte_in, Protocol_Data_t *data_out)
{
    if (!context || !data_out) {
        Log_Message(LOG_LEVEL_ERROR, "[Protocol] ProcessByte: Invalid context or data_out pointer.");
        return PROTOCOL_ERROR_PARAM;
    }

    uint8_t *temp_buffer = context->temp_buffer; // 使用上下文中的临时缓冲区
    uint32_t *current_idx = &context->current_idx; // 使用上下文中的索引
    Protocol_ParseState_t *parse_state = &context->parse_state; // 使用上下文中的解析状态
    uint8_t *expected_len = &context->expected_len; // 使用上下文中的期望长度

    // 确保每次处理一帧时数据_out被清零，以便干净地开始
    // 只有当一帧完整解析或发生错误时才清零 data_out，否则会丢失部分解析结果
    if (*parse_state == PROTOCOL_PARSE_STATE_WAIT_SOF && *current_idx == 0) {
        memset(data_out, 0, sizeof(Protocol_Data_t));
        // Reset the parsed_data inside context as well, to avoid stale data
        memset(&context->parsed_data, 0, sizeof(Protocol_Data_t));
    }


    switch (*parse_state)
    {
        case PROTOCOL_PARSE_STATE_WAIT_SOF:
            // 寻找帧头 (这里假设帧头是 0x7E 或 0xF6，可根据协议扩展)
            if (byte_in == FRAME_HEADER_7E || byte_in == FRAME_HEADER_F6 ||
                byte_in == FRAME_HEADER_F7 || byte_in == FRAME_HEADER_52) // 检查是否是有效帧头
            {
                temp_buffer[(*current_idx)++] = byte_in;
                *parse_state = PROTOCOL_PARSE_STATE_READ_FIXED_HEADER;
                data_out->is_rs485 = true; // 假设这些帧头都是RS485协议
            }
            // 否则，丢弃当前字节，继续等待帧头
            return PROTOCOL_IN_PROGRESS;

        case PROTOCOL_PARSE_STATE_READ_FIXED_HEADER:
            temp_buffer[(*current_idx)++] = byte_in;

            // 读取到长度字段 (第6个字节，索引为5)
            if (*current_idx == 6) {
                *expected_len = temp_buffer[5] + PROTOCOL_MIN_FRAME_LEN; // 协议中的LENGTH是信息域长度，这里计算总帧长度

                // 检查帧总长度的合法性
                if (*expected_len < PROTOCOL_MIN_FRAME_LEN || *expected_len > PROTOCOL_MAX_FRAME_LEN) {
                    Log_Message(LOG_LEVEL_WARNING, "[Protocol] ProcessByte: Invalid frame length %u. Resetting parser.", *expected_len);
                    Protocol_ResetParser(context); // 长度非法，重置解析器
                    return PROTOCOL_ERROR_FRAME;
                }
                *parse_state = PROTOCOL_PARSE_STATE_READ_INFO; // 长度有效，进入读取信息域
            } else if (*current_idx > 6 && *current_idx < PROTOCOL_MIN_FRAME_LEN - PROTOCOL_CRC_LEN) {
                // 读取固定头字段的后续字节（如果固定头有更多字节）
                // 确保 temp_buffer 不溢出
                if (*current_idx >= PROTOCOL_MAX_FRAME_LEN) {
                    Log_Message(LOG_LEVEL_WARNING, "[Protocol] ProcessByte: Buffer overflow during fixed header read. Resetting parser.");
                    Protocol_ResetParser(context);
                    return PROTOCOL_ERROR_BUFFER_FULL;
                }
            } else if (*current_idx > PROTOCOL_MAX_FRAME_LEN) {
                 Log_Message(LOG_LEVEL_WARNING, "[Protocol] ProcessByte: Current index (%lu) exceeds max frame length. Resetting parser.", *current_idx);
                 Protocol_ResetParser(context);
                 return PROTOCOL_ERROR_BUFFER_FULL;
            }
            return PROTOCOL_IN_PROGRESS;

        case PROTOCOL_PARSE_STATE_READ_INFO:
            temp_buffer[(*current_idx)++] = byte_in;

            // 读取完整帧 (包括信息域和CRC前一个字节)
            if (*current_idx >= *expected_len) {
                // 帧数据已收集完毕，进行CRC校验
                uint16_t received_crc = (temp_buffer[*expected_len - 2] | (temp_buffer[*expected_len - 1] << 8));
                uint16_t calculated_crc = Modbus_CRC16(temp_buffer, *expected_len - PROTOCOL_CRC_LEN); // CRC校验范围到倒数第三个字节

                if (calculated_crc != received_crc) {
                    Log_Message(LOG_LEVEL_WARNING, "[Protocol] ProcessByte: CRC mismatch (Calc: 0x%04X, Recv: 0x%04X). Resetting parser.", calculated_crc, received_crc);
                    Protocol_ResetParser(context); // CRC不匹配，重置解析器
                    return PROTOCOL_ERROR_CRC;
                }

                // CRC校验通过，填充 Protocol_Data_t 结构体
                RS485_Frame_t *frame = &data_out->rs485_frame; // 直接使用 data_out 中的帧结构
                frame->sof = temp_buffer[0];
                frame->addr1 = temp_buffer[1];
                frame->addr2 = temp_buffer[2];
                frame->cmd = temp_buffer[3];
                frame->cmd_sub = temp_buffer[4];
                frame->info_len = temp_buffer[5]; // 信息域长度
                // 将信息域数据拷贝到 Protocol_Data_t 内部的通用缓冲区
                // 并将 rs485_frame.info 指向这个缓冲区
                if (frame->info_len > 0) {
                    if (frame->info_len > PROTOCOL_MAX_INFO_LEN) { // 再次检查信息域长度
                        Log_Message(LOG_LEVEL_ERROR, "[Protocol] ProcessByte: Info length from frame header (%u) exceeds internal buffer.", frame->info_len);
                        Protocol_ResetParser(context);
                        return PROTOCOL_ERROR_FRAME;
                    }
                    memcpy(data_out->uart_data.data, &temp_buffer[6], frame->info_len);
                    frame->info = data_out->uart_data.data; // 将指针指向内部缓冲区
                } else {
                    frame->info = NULL; // 没有信息域数据
                }
                frame->crc = received_crc; // 存储接收到的CRC

                data_out->is_rs485 = true; // 标记为RS485帧

                Protocol_ResetParser(context); // 成功解析一帧，重置解析器状态
                return PROTOCOL_OK; // 返回成功
            }
             else if (*current_idx >= PROTOCOL_MAX_FRAME_LEN) {
                 Log_Message(LOG_LEVEL_WARNING, "[Protocol] ProcessByte: Buffer overflow during info read. Resetting parser.");
                 Protocol_ResetParser(context);
                 return PROTOCOL_ERROR_BUFFER_FULL;
             }
            return PROTOCOL_IN_PROGRESS;

        default: // 未知状态，重置
            Log_Message(LOG_LEVEL_ERROR, "[Protocol] ProcessByte: Unknown parser state %d. Resetting parser.", *parse_state);
            Protocol_ResetParser(context);
            return PROTOCOL_ERROR_FRAME;
    }
}

/**
 * @brief 重置协议解析器的状态。
 * @param context 指向要重置的协议解析上下文。
 */
void Protocol_ResetParser(Protocol_ParserContext_t *context)
{
    if (context) {
        context->current_idx = 0;
        context->parse_state = PROTOCOL_PARSE_STATE_WAIT_SOF;
        context->expected_len = 0;
        // 清空临时缓冲区
        memset(context->temp_buffer, 0, sizeof(context->temp_buffer));
        // 不需要清空 parsed_data，因为 Protocol_ProcessByte 在接收到 SOF 时会清零 data_out
        // 或者在发送成功后将其内容交给上层处理。
    }
}
