#include "serial_interface.h"
#include "rtos_abstraction.h"
#include "pch.h"
static RingBuffer_t error_log_buffer;
static RS485_TxFrameQueue_t rs485_tx_queue = {0};

static Protocol_Data_t rx_data = {0};
static uint32_t rx_index = 0;
static uint8_t state = 0;
static uint8_t expected_length = 0;

static Serial_Status Serial_ErrorLog_Init(void)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops)
        return SERIAL_ERR_INIT;
    if (RingBuffer_Init(&error_log_buffer, 16, sizeof(Serial_ErrorLog_t)) != RB_OK)
    {
        return SERIAL_ERR_INIT;
    }
    return SERIAL_OK;
}

static void Serial_LogError(Serial_ErrorType_t type, USART_TypeDef *instance)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops)
        return;

    Serial_ErrorLog_t log = {
        .type = type,
        .timestamp = rtos_ops->GetTickCount(),
        .instance = instance};
    RingBuffer_Write(&error_log_buffer, &log);
}

Serial_Status Serial_GetErrorLog(Serial_ErrorLog_t *log, uint32_t timeout)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !log)
        return SERIAL_ERR_INIT;

    if (rtos_ops->SemaphoreTake(error_log_buffer.sem, timeout))
    {
        RB_Status rb_status = RingBuffer_Read(&error_log_buffer, log);
        if (rb_status != RB_OK)
        {
            rtos_ops->SemaphoreGive(error_log_buffer.sem);
            return (rb_status == RB_ERROR_BUFFER_EMPTY) ? SERIAL_ERR_NO_DATA : SERIAL_ERR_BUFFER_FULL;
        }
        return SERIAL_OK;
    }
    return SERIAL_ERR_NO_DATA;
}

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

static Serial_Status RS485_FrameToBytes(RS485_Frame_t *frame, uint8_t *bytes, uint32_t max_len, uint32_t *bytes_len)
{
    if (!frame || !bytes || !bytes_len)
        return SERIAL_ERR_INIT;

    frame->length = 8 + frame->info_len;
    if (frame->length > max_len || frame->length > 208)
        return SERIAL_ERR_BUFFER_FULL;

    bytes[0] = frame->sof;
    bytes[1] = frame->addr1;
    bytes[2] = frame->addr2;
    bytes[3] = frame->cmd;
    bytes[4] = frame->cmd_sub;
    bytes[5] = frame->length;
    memcpy(&bytes[6], frame->info, frame->info_len);

    frame->crc = Modbus_CRC16(bytes, frame->length - 2);
    bytes[frame->length - 2] = frame->crc & 0xFF;
    bytes[frame->length - 1] = (frame->crc >> 8) & 0xFF;

    *bytes_len = frame->length;
    return SERIAL_OK;
}

static Serial_Status Serial_Init(Serial_Device_t *dev)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !dev)
        return SERIAL_ERR_INIT;

    static uint8_t log_initialized = 0;
    if (!log_initialized)
    {
        if (Serial_ErrorLog_Init() != SERIAL_OK)
        {
            return SERIAL_ERR_INIT;
        }
        log_initialized = 1;
    }

    Protocol_Init();
    return Serial_Driver_Init(dev);
}

static Serial_Status Serial_Deinit(Serial_Device_t *dev)
{
    return Serial_Driver_Deinit(dev);
}

static Serial_Status Serial_SendData(Serial_Device_t *dev, const uint8_t *data, uint32_t length)
{
    Serial_Status status = Serial_Driver_SendData(dev, data, length);
    if (status != SERIAL_OK)
    {
        Serial_LogError(ERROR_TIMEOUT, dev->instance);
    }
    return status;
}

static Serial_Status Serial_SendFrame(Serial_Device_t *dev, RS485_Frame_t *frame)
{
    if (!dev || !frame || dev->mode != RS485_MODE)
        return SERIAL_ERR_INIT;

    uint8_t bytes[208];
    uint32_t bytes_len;
    Serial_Status status = RS485_FrameToBytes(frame, bytes, sizeof(bytes), &bytes_len);
    if (status != SERIAL_OK)
        return status;

    return Serial_SendData(dev, bytes, bytes_len);
}

static Serial_Status Serial_ReceiveFromBuffer(Serial_Device_t *dev, Protocol_Data_t *data, uint32_t timeout)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !dev || !data)
        return SERIAL_ERR_INIT;

    uint8_t byte;
    if (rtos_ops->SemaphoreTake(dev->rx_buffer.sem, timeout))
    {
        RB_Status rb_status = RingBuffer_Read(&dev->rx_buffer, &byte);
        if (rb_status != RB_OK)
        {
            rtos_ops->SemaphoreGive(dev->rx_buffer.sem);
            return (rb_status == RB_ERROR_BUFFER_EMPTY) ? SERIAL_ERR_NO_DATA : SERIAL_ERR_BUFFER_FULL;
        }

        if (Protocol_ProcessByte(dev, data, byte, &rx_index, &state, &expected_length, NULL))
        {
            return SERIAL_OK;
        }
        else
        {
            Serial_LogError(data->is_rs485 ? ERROR_CRC : ERROR_FRAME, dev->instance);
            rx_index = 0;
            state = 0;
            expected_length = 0;
            memset(&rx_data, 0, sizeof(Protocol_Data_t));
            return SERIAL_ERR_FRAME;
        }
    }
    return SERIAL_ERR_NO_DATA;
}

static Serial_Status Serial_AddFrameToQueue(RS485_Frame_t *frame)
{
    if (!frame || rs485_tx_queue.count >= RS485_TX_QUEUE_SIZE)
    {
        return SERIAL_ERR_BUFFER_FULL;
    }

    RS485_Frame_t *queued_frame = &rs485_tx_queue.frames[rs485_tx_queue.tail];
    *queued_frame = *frame;
    queued_frame->info = rs485_tx_queue.info_buffers[rs485_tx_queue.tail];
    memcpy(queued_frame->info, frame->info, frame->info_len);

    rs485_tx_queue.tail = (rs485_tx_queue.tail + 1) % RS485_TX_QUEUE_SIZE;
    rs485_tx_queue.count++;

    return SERIAL_OK;
}

static void Serial_PollSendRS485(Serial_Device_t *dev)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !dev || dev->mode != RS485_MODE || rs485_tx_queue.count == 0)
    {
        return;
    }

    RS485_Frame_t *frame = &rs485_tx_queue.frames[rs485_tx_queue.head];
    Serial_SendFrame(dev, frame);

    rs485_tx_queue.head = (rs485_tx_queue.head + 1) % RS485_TX_QUEUE_SIZE;
    rs485_tx_queue.count--;

    rtos_ops->Delay(dev->silent_ticks);
}

const Serial_Ops_t Serial_Operations = {
    .Init = Serial_Init,
    .Deinit = Serial_Deinit,
    .SendData = Serial_SendData,
    .SendFrame = Serial_SendFrame,
    .ReceiveFromBuffer = Serial_ReceiveFromBuffer,
    .GetErrorLog = Serial_GetErrorLog,
    .AddFrameToQueue = Serial_AddFrameToQueue,
    .PollSendRS485 = Serial_PollSendRS485};
