#include "protocol_handler.h"
#include "state_machine.h"
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

static int HandleState_Init(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    Protocol_Data_t *data = (Protocol_Data_t *)user_data;
    for (uint8_t i = 0; i < handler_count; i++)
    {
        if (handlers[i].header == byte)
        {
            data->is_rs485 = 1;
            ctx->index = 1;
            ctx->current_state = STATE_START;
            return 1;
        }
    }
    return 0;
}

static int HandleState_Start(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    Protocol_Data_t *data = (Protocol_Data_t *)user_data;
    static uint8_t temp_buffer[208];
    static uint8_t info_buffer[200];

    temp_buffer[ctx->index++] = byte;
    if (ctx->index == 5)
    {
        ctx->expected_length = byte;
        if (ctx->expected_length < 8 || ctx->expected_length > 208)
        {
            ctx->index = 0;
            ctx->expected_length = 0;
            ctx->current_state = STATE_INIT;
            return 0;
        }
        ctx->current_state = STATE_DATA;
    }
    return 1;
}

static int HandleState_Data(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    Protocol_Data_t *data = (Protocol_Data_t *)user_data;
    static uint8_t temp_buffer[208];
    static uint8_t info_buffer[200];

    temp_buffer[ctx->index++] = byte;
    if (ctx->index >= ctx->expected_length - 2)
    {
        ctx->current_state = STATE_END;
    }
    return 1;
}

static int HandleState_End(StateContext_t *ctx, uint8_t byte, void *user_data)
{
    Protocol_Data_t *data = (Protocol_Data_t *)user_data;
    static uint8_t temp_buffer[208];
    static uint8_t info_buffer[200];

    temp_buffer[ctx->index++] = byte;
    if (ctx->index >= ctx->expected_length)
    {
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
        frame->crc = (temp_buffer[ctx->expected_length - 2] | (temp_buffer[ctx->expected_length - 1] << 8));

        uint8_t crc_buffer[208];
        memcpy(crc_buffer, temp_buffer, ctx->expected_length - 2);
        uint16_t calc_crc = Modbus_CRC16(crc_buffer, ctx->expected_length - 2);
        if (calc_crc != frame->crc)
        {
            ctx->index = 0;
            ctx->expected_length = 0;
            ctx->current_state = STATE_INIT;
            return 0;
        }

        ctx->index = 0;
        ctx->expected_length = 0;
        ctx->current_state = STATE_INIT;
        return 1;
    }
    return 1;
}

static StateTransition_t rs485_transitions[] = {
    {STATE_INIT, HandleState_Init},
    {STATE_START, HandleState_Start},
    {STATE_DATA, HandleState_Data},
    {STATE_END, HandleState_End},
    {0, NULL}};

void Protocol_Init(void)
{
    Protocol_RegisterHandler(FRAME_HEADER_7E, NULL); // 使用状态机处理
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
    handlers[handler_count].handler = NULL; // 状态机处理
    handler_count++;
}

int Protocol_ProcessByte(void *dev, Protocol_Data_t *data, uint8_t byte,
                         uint32_t *index, uint8_t *state, uint8_t *expected_length,
                         void *xHigherPriorityTaskWoken)
{
    static StateContext_t ctx;
    if (*state == 0)
    {
        StateMachine_Init(&ctx, data);
        *index = ctx.index;
        *state = ctx.current_state;
        *expected_length = ctx.expected_length;
    }

    StateMachine_Process(&ctx, rs485_transitions, byte);
    *index = ctx.index;
    *state = ctx.current_state;
    *expected_length = ctx.expected_length;

    return (*state == STATE_INIT && ctx.index == 0) ? 1 : 0;
}