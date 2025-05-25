#include "protocol_handler.h"
#include "pch.h"

static Protocol_HandlerEntry_t handlers[4] = {0};
static uint8_t handler_count = 0;

static uint16_t Modbus_CRC16(const uint8_t *data, uint32_t length)
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

void Protocol_Init(void)
{
    Protocol_RegisterHandler(FRAME_HEADER_7E, NULL);
    Protocol_RegisterHandler(FRAME_HEADER_F6, NULL);
}

void Protocol_RegisterHandler(Protocol_FrameHeader_t header, Protocol_Handler_t handler)
{
    if (handler_count >= 4)
        return;

    for (uint8_t i = 0; i < handler_count; i++)
    {
        if (handlers[i].header == header)
        {
            handlers[i].handler = handler;
            return;
        }
    }

    handlers[handler_count].header = header;
    handlers[handler_count].handler = NULL;
    handler_count++;
}

int Protocol_ProcessByte(void *dev, Protocol_Data_t *data, uint8_t byte,
                         uint32_t *index, uint8_t *state, uint8_t *expected_length,
                         void *xHigherPriorityTaskWoken)
{
    static uint8_t temp_buffer[208];
    static uint8_t info_buffer[200];
    static uint32_t idx = 0;

    if (*state == 0) {
        idx = 0;
        *index = 0;
        *expected_length = 0;
    }

    // 检查帧头
    if (idx == 0) {
        for (uint8_t i = 0; i < handler_count; i++) {
            if (handlers[i].header == byte) {
                temp_buffer[idx++] = byte;
                *state = 1;
                return 0;
            }
        }
        return 0;
    }

    // 收集帧数据
    temp_buffer[idx++] = byte;
    if (idx == 6) {
        *expected_length = temp_buffer[5];
        if (*expected_length < 8 || *expected_length > 208) {
            idx = 0;
            *state = 0;
            *expected_length = 0;
            return 0;
        }
    }

    // 完整帧收集完成
    if (idx >= *expected_length) {
        RS485_Frame_t *frame = &data->rs485_frame;
        frame->sof = temp_buffer[0];
        frame->addr1 = temp_buffer[1];
        frame->addr2 = temp_buffer[2];
        frame->cmd = temp_buffer[3];
        frame->cmd_sub = temp_buffer[4];
        frame->length = temp_buffer[5];
        frame->info_len = frame->length - 8;
        frame->info = info_buffer;
        memcpy(frame->info, &temp_buffer[6], frame->info_len);
        frame->crc = (temp_buffer[*expected_length - 2] | (temp_buffer[*expected_length - 1] << 8));

        uint8_t crc_buffer[208];
        memcpy(crc_buffer, temp_buffer, *expected_length - 2);
        uint16_t calc_crc = Modbus_CRC16(crc_buffer, *expected_length - 2);
        if (calc_crc != frame->crc) {
            idx = 0;
            *state = 0;
            *expected_length = 0;
            return 0;
        }

        data->is_rs485 = 1;
        idx = 0;
        *state = 0;
        *index = 0;
        *expected_length = 0;
        return 1;
    }

    *index = idx;
    return 0;
}
