#include "api_tcp.h"
#include "pch.h"
#include "protocol_handler.h"
#include "log_system.h"

/**
 * @brief 解码函数，根据自定义协议规则将转义后的输入数据解码为原始数据。
 *
 * 协议转义规则如下：
 *   - 输入中 0xFD 0x00 表示原始数据 0xFD
 *   - 输入中 0xFD 0x01 表示原始数据 0xFE
 *   - 其他所有数据字节直接复制
 *
 * @param decoder         解码器状态结构体指针，保持转义状态
 * @param input           输入数据结构体，包含原始数据与当前位置
 * @param output          输出数据结构体，解码数据写入此结构中
 * @param decoded_bytes   实际写入输出缓冲区的字节数
 * @return true   解码成功
 * @return false  参数无效或为 NULL 指针
 */
bool decode_bytes(Decoder *decoder, Input *input, Output *output, uint16_t *decoded_bytes)
{
    // -------- 1. 参数有效性检查 --------
    if (!decoder || !input || !output || !decoded_bytes ||
        !input->data || !output->data)
    {
        Log_Message(LOG_LEVEL_ERROR, "[decode] Parameter is a null pointer");
        return false;
    }

    // -------- 2. 保存起始写入位置，以便计算解码的字节数 --------
    uint16_t start_offset = output->offset;

    // -------- 3. 解码主循环 --------
    while (input->offset < input->length)
    {
        uint8_t byte = input->data[input->offset];

        if (decoder->escape)
        {
            // 上一个字节是转义符 0xFD，现在判断转义内容
            switch (byte)
            {
            case 0x00:
                output->data[output->offset++] = 0xFD;
                break;
            case 0x01:
                output->data[output->offset++] = 0xFE;
                break;
            default:
                // 如果遇到非法的转义序列，作为容错处理，将 0xFD 与当前字节一起透传
                Log_Message(LOG_LEVEL_WARNING, "[decode] Illegal escape sequence: 0xFD 0x%02X", byte);
                output->data[output->offset++] = 0xFD;
                output->data[output->offset++] = byte;
                break;
            }
            decoder->escape = false; // 重置转义状态
            input->offset++;         // 消费当前字节
        }
        else if (byte == 0xFD)
        {
            // 当前字节是转义起始符 0xFD
            if (input->offset + 1 < input->length)
            {
                // 后面还有数据，可以进入转义状态，等下一轮处理
                decoder->escape = true;
                input->offset++;
            }
            else
            {
                // 转义符是最后一个字节，保留状态，等待下批数据
                decoder->escape = true;
                input->offset++;
                break;
            }
        }
        else
        {
            // 普通字节，直接拷贝到输出
            output->data[output->offset++] = byte;
            input->offset++;
        }
    }

    // -------- 4. 写出解码出的字节数 --------
    *decoded_bytes = output->offset - start_offset;
    return true;
}

net_header_t parse_packet_header(const uint8_t *data, uint16_t length) {
    net_header_t packet_header;

    // 解析 st_header_long_t 结构体
    if (length >= sizeof(net_header_t)) {
        const net_header_t *header_union = (const net_header_t *)data;
        const struct st_header_long_t *header = &header_union->st_header_long;
        // 示例解析：假设 source/type/cmd_id/data_len 需要根据协议自定义
        packet_header.sys_type = header->sys_type[0]; // 假设来源在 sys_type[0]
        packet_header.cmd_type = header->cmd_type[0];   // 假设类型在 cmd_type[0]
        packet_header.cmd_id = (header->seq[0] << 8) | header->seq[1]; // 假设命令ID用 seq[0,1]
        // 假设数据长度在 net_header_len[1]
        packet_header.data_len = header->net_header_len[1];
        if (packet_header.data_len > 0 && length >= sizeof(net_header_t) + packet_header.data_len) {
            packet_header.data = (uint8_t *)&data[sizeof(net_header_t)];
        } else {
            packet_header.data_len = 0;
            packet_header.data = NULL;
        }
        return packet_header;
    }

}



/**
 * @brief 串口接收任务 (处理RS485协议数据)
 * @param pvParameters 指向 Serial_Device_t 实例 (通常是 g_rs485_serial)
 */
void App_SerialRxTask(void *pvParameters) {
    Serial_Device_t *serial_dev = (Serial_Device_t *)pvParameters;
    if (!serial_dev) {
        Log_Message(LOG_LEVEL_ERROR, "[Serial Rx] Invalid device parameter. Suspending task.");
        if (g_rtos_ops && g_rtos_ops->Task_Suspend) g_rtos_ops->Task_Suspend(NULL); else for(;;);
    }
    Log_Message(LOG_LEVEL_INFO, "[Serial Rx] Task started for USART %p.", serial_dev->instance);

    Protocol_Data_t received_data; // 用于存储接收到的协议数据
    uint8_t rx_buffer[256];
    uint16_t rx_len;
    while (1) {
        // 从串口中间层接收并解析协议数据，带超时
        ESP32_Comm_Type_t comm_type = ESP32_COMM_TYPE_NULL; // 默认通信类型
        AT_Status_t status = g_esp32_at_ops.ReceiveData(&g_esp32_dev, rx_buffer, &rx_len, 50, &comm_type);
        if (status == AT_OK) {

            const net_header_t *header_union = (const net_header_t *)rx_buffer;
            const struct st_header_long_t *header = &header_union->st_header_long;

            // 根据来源分发
            if (comm_type == ESP32_COMM_TYPE_WIFI) {
                xQueueSend(tcp_cmd_queue, &packet, 0);
            } else if (comm_type == ESP32_COMM_TYPE_BLE) {
                xQueueSend(ble_cmd_queue, &packet, 0);
            }
            if (received_data.is_rs485) {
                // 处理RS485协议数据
                Log_Message(LOG_LEVEL_INFO, "[RS485 Rx] Frame: Addr1=0x%02X, Cmd=0x%02X, InfoLen=%u",
                            received_data.rs485_frame.addr1,
                            received_data.rs485_frame.cmd,
                            received_data.rs485_frame.info_len);
                // 可以在这里进一步处理接收到的RS485帧，例如根据命令字进行响应
            } else {
                // 处理其他（非RS485）协议数据，如果存在
                Log_Message(LOG_LEVEL_INFO, "[UART Rx] Non-RS485 data received on USART %p.", serial_dev->instance);
            }
        } else if (status == SERIAL_ERR_NO_DATA) {
            // 没有数据，继续循环
        } else {
            // 其他错误，例如 SERIAL_ERR_FRAME_ERR (CRC或帧格式错误)
            Log_Message(LOG_LEVEL_WARNING, "[Serial Rx] Error receiving protocol data on USART %p. Status: %d.", serial_dev->instance, status);
        }
        // 如果上面有数据处理，不需要额外延时，否则可以延时
    }
}