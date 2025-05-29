#ifndef __PROTOCOL_HANDLER_H
#define __PROTOCOL_HANDLER_H

#include "stdint.h"
#include "stdbool.h"

typedef enum
{
    FRAME_HEADER_7E = 0x7E,
    FRAME_HEADER_F6 = 0xF6,
    FRAME_HEADER_F7 = 0xF7,
    FRAME_HEADER_52 = 0x52
} Protocol_FrameHeader_t;

typedef struct
{
    uint8_t sof;
    uint8_t addr1;
    uint8_t addr2;
    uint8_t cmd;
    uint8_t cmd_sub;
    uint8_t length;
    uint8_t *info;
    uint16_t info_len;
    uint16_t crc;
} RS485_Frame_t;

typedef struct
{
    union
    {
        RS485_Frame_t rs485_frame;
        struct
        {
            uint8_t data[208];
            uint32_t length;
        } uart_data;
    };
    uint8_t is_rs485;
} Protocol_Data_t;

typedef int (*Protocol_Handler_t)(void *dev, Protocol_Data_t *data, uint8_t byte,
                                  uint32_t *index, uint8_t *state, uint8_t *expected_length,
                                  void *xHigherPriorityTaskWoken);

typedef struct
{
    Protocol_FrameHeader_t header;
    Protocol_Handler_t handler;
} Protocol_HandlerEntry_t;

void Protocol_Init(void);
void Protocol_RegisterHandler(Protocol_FrameHeader_t header, Protocol_Handler_t handler);
int Protocol_ProcessByte(void *dev, Protocol_Data_t *data, uint8_t byte,
                         uint32_t *index, uint8_t *state, uint8_t *expected_length,
                         void *xHigherPriorityTaskWoken);

#endif /* __PROTOCOL_HANDLER_H */
