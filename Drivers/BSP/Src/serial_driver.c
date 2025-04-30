#include "serial_driver.h"
#include "pch.h"
#include "rtos_abstraction.h"

static Serial_Device_t *serial_devices[4] = {0};
static uint8_t serial_device_count = 0;
static RingBuffer_t error_log_buffer;
static FrameHandlerEntry_t frame_handlers[4] = {0};
static uint8_t frame_handler_count = 0;
static RS485_TxFrameQueue_t rs485_tx_queue = {0};





static Serial_Status Serial_ErrorLog_Init(void)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops) return SERIAL_ERR_INIT;
    if (RingBuffer_Init(&error_log_buffer, 16, sizeof(Serial_ErrorLog_t)) != RB_OK) {
        return SERIAL_ERR_INIT;
    }
    return SERIAL_OK;
}

static void Serial_LogError(Serial_ErrorType_t type, USART_TypeDef* instance)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops) return;

    Serial_ErrorLog_t log = {
        .type = type,
        .timestamp = rtos_ops->GetTickCount(),
        .instance = instance
    };
    RingBuffer_Write(&error_log_buffer, &log);
}

Serial_Status Serial_GetErrorLog(Serial_ErrorLog_t *log, uint32_t timeout)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !log) return SERIAL_ERR_INIT;

    if (rtos_ops->SemaphoreTake(error_log_buffer.sem, timeout)) {
        RB_Status rb_status = RingBuffer_Read(&error_log_buffer, log);
        if (rb_status != RB_OK) {
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
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

Serial_Status RS485_FrameFromBytes(uint8_t *bytes, RS485_Frame_t *frame, uint8_t *info_buffer, uint16_t info_max_len)
{
    if (!bytes || !frame || !info_buffer) return SERIAL_ERR_INIT;

    uint8_t len = bytes[5];
    if (len < 8 || len > 208) return SERIAL_ERR_FRAME;

    frame->sof = bytes[0];
    frame->addr1 = bytes[1];
    frame->addr2 = bytes[2];
    frame->cmd = bytes[3];
    frame->cmd_sub = bytes[4];
    frame->length = len;
    frame->info_len = len - 8;
    frame->info = info_buffer;

    if (frame->info_len > info_max_len) return SERIAL_ERR_BUFFER_FULL;

    memcpy(frame->info, &bytes[6], frame->info_len);

    frame->crc = (bytes[len - 2] | (bytes[len - 1] << 8));

    uint8_t crc_buffer[208];
    crc_buffer[0] = frame->sof;
    crc_buffer[1] = frame->addr1;
    crc_buffer[2] = frame->addr2;
    crc_buffer[3] = frame->cmd;
    crc_buffer[4] = frame->cmd_sub;
    crc_buffer[5] = frame->length;
    memcpy(&crc_buffer[6], frame->info, frame->info_len);
    uint16_t calc_crc = Modbus_CRC16(crc_buffer, len - 2);
    if (calc_crc != frame->crc) return SERIAL_ERR_CRC;

    return SERIAL_OK;
}

Serial_Status RS485_FrameToBytes(RS485_Frame_t *frame, uint8_t *bytes, uint32_t max_len, uint32_t *bytes_len)
{
    if (!frame || !bytes || !bytes_len) return SERIAL_ERR_INIT;

    frame->length = 8 + frame->info_len;
    if (frame->length > max_len || frame->length > 208) return SERIAL_ERR_BUFFER_FULL;

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

static Serial_Status Serial_RegisterFrameHandler(Serial_FrameHeader_t header, Serial_FrameHandler_t handler)
{
    if (frame_handler_count >= 4) {
        return SERIAL_ERR_INIT;
    }

    for (uint8_t i = 0; i < frame_handler_count; i++) {
        if (frame_handlers[i].header == header) {
            frame_handlers[i].handler = handler;
            return SERIAL_OK;
        }
    }

    frame_handlers[frame_handler_count].header = header;
    frame_handlers[frame_handler_count].handler = handler;
    frame_handler_count++;
    return SERIAL_OK;
}

static Serial_Status Serial_GPIO_Init(Serial_Device_t *dev)
{
    if (!dev->tx_port || !dev->rx_port || (dev->mode == RS485_MODE && !dev->de_port)) {
        return SERIAL_ERR_INIT;
    }

    if (Common_GPIO_Init(dev->tx_port, dev->tx_pin, GPIO_Mode_AF, GPIO_OType_PP,
                         GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af) != COMMON_OK) {
        return SERIAL_ERR_INIT;
    }

    if (Common_GPIO_Init(dev->rx_port, dev->rx_pin, GPIO_Mode_AF, GPIO_OType_PP,
                         GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af) != COMMON_OK) {
        return SERIAL_ERR_INIT;
    }

    if (dev->mode == RS485_MODE) {
        if (Common_GPIO_Init(dev->de_port, dev->de_pin, GPIO_Mode_OUT, GPIO_OType_PP,
                             GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0) != COMMON_OK) {
            return SERIAL_ERR_INIT;
        }
        GPIO_ResetBits(dev->de_port, dev->de_pin);
    }

    return SERIAL_OK;
}

static Serial_Status Serial_USART_Init(Serial_Device_t *dev)
{
    if (!dev->instance) {
        return SERIAL_ERR_INIT;
    }

    if (Common_USART_Init(dev->instance, dev->baudrate, USART_WordLength_8b,
                         USART_StopBits_1, USART_Parity_No) != COMMON_OK) {
        return SERIAL_ERR_INIT;
    }

    return SERIAL_OK;
}

static Serial_Status Serial_NVIC_Init(Serial_Device_t *dev)
{
    if (!dev->instance) {
        return SERIAL_ERR_INIT;
    }

    USART_ITConfig(dev->instance, USART_IT_RXNE, ENABLE);
    NVIC_InitTypeDef nvic_init = {
        .NVIC_IRQChannel = dev->irqn,
        .NVIC_IRQChannelPreemptionPriority = 0,
        .NVIC_IRQChannelSubPriority = 0,
        .NVIC_IRQChannelCmd = ENABLE
    };
    NVIC_Init(&nvic_init);
    return SERIAL_OK;
}

static Serial_Status Serial_RingBuffer_Init(Serial_Device_t *dev)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops) return SERIAL_ERR_INIT;
    if (RingBuffer_Init(&dev->rx_buffer, 16, sizeof(Serial_RxData_t)) != RB_OK) {
        return SERIAL_ERR_INIT;
    }
    return SERIAL_OK;
}

Serial_Status Serial_Init(Serial_Device_t *dev)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !dev || !dev->instance || serial_device_count >= 4) {
        return SERIAL_ERR_INIT;
    }

    static uint8_t log_initialized = 0;
    if (!log_initialized) {
        if (Serial_ErrorLog_Init() != SERIAL_OK) {
            return SERIAL_ERR_INIT;
        }
        log_initialized = 1;
    }

    static uint8_t frame_handler_initialized = 0;
    if (!frame_handler_initialized) {
        Serial_RegisterFrameHandler(FRAME_HEADER_7E, HandleFrame_7E);
        Serial_RegisterFrameHandler(FRAME_HEADER_F6, HandleFrame_F6);
        frame_handler_initialized = 1;
    }

    Serial_Status status;

    status = Serial_GPIO_Init(dev);
    if (status != SERIAL_OK) {
        Serial_Deinit(dev);
        return status;
    }

    status = Serial_USART_Init(dev);
    if (status != SERIAL_OK) {
        Serial_Deinit(dev);
        return status;
    }

    status = Serial_NVIC_Init(dev);
    if (status != SERIAL_OK) {
        Serial_Deinit(dev);
        return status;
    }

    status = Serial_RingBuffer_Init(dev);
    if (status != SERIAL_OK) {
        Serial_Deinit(dev);
        return status;
    }

    uint32_t silent_us = (35 * 1000000) / dev->baudrate;
    dev->silent_ticks = silent_us / 1000; // 转换为ms

    serial_devices[serial_device_count++] = dev;
    return SERIAL_OK;
}

Serial_Status Serial_Deinit(Serial_Device_t *dev)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !dev) return SERIAL_ERR_INIT;

    if (dev->instance) {
        USART_ITConfig(dev->instance, USART_IT_RXNE, DISABLE);
        USART_Cmd(dev->instance, DISABLE);
    }

    if (RingBuffer_Deinit(&dev->rx_buffer) != RB_OK) {
        return SERIAL_ERR_INIT;
    }

    for (uint8_t i = 0; i < serial_device_count; i++) {
        if (serial_devices[i] == dev) {
            serial_devices[i] = serial_devices[--serial_device_count];
            serial_devices[serial_device_count] = NULL;
            break;
        }
    }

    return SERIAL_OK;
}

Serial_Status Serial_SendData(Serial_Device_t *dev, const uint8_t *data, uint32_t length)
{
    if (!dev || !data) return SERIAL_ERR_INIT;

    if (dev->mode == RS485_MODE) {
        GPIO_SetBits(dev->de_port, dev->de_pin);
    }

    uint32_t timeout = 1000000;
    for (uint32_t i = 0; i < length; i++) {
        while (USART_GetFlagStatus(dev->instance, USART_FLAG_TXE) == RESET && timeout--) {}
        if (timeout == 0) {
            if (dev->mode == RS485_MODE) {
                GPIO_ResetBits(dev->de_port, dev->de_pin);
            }
            Serial_LogError(ERROR_TIMEOUT, dev->instance);
            return SERIAL_ERR_TIMEOUT;
        }
        USART_SendData(dev->instance, data[i]);
    }

    timeout = 1000000;
    while (USART_GetFlagStatus(dev->instance, USART_FLAG_TC) == RESET && timeout--) {}
    if (timeout == 0) {
        if (dev->mode == RS485_MODE) {
            GPIO_ResetBits(dev->de_port, dev->de_pin);
        }
        Serial_LogError(ERROR_TIMEOUT, dev->instance);
        return SERIAL_ERR_TIMEOUT;
    }

    if (dev->mode == RS485_MODE) {
        GPIO_ResetBits(dev->de_port, dev->de_pin);
    }

    return SERIAL_OK;
}

Serial_Status Serial_SendFrame(Serial_Device_t *dev, RS485_Frame_t *frame)
{
    if (!dev || !frame || dev->mode != RS485_MODE) return SERIAL_ERR_INIT;

    uint8_t bytes[208];
    uint32_t bytes_len;
    Serial_Status status = RS485_FrameToBytes(frame, bytes, sizeof(bytes), &bytes_len);
    if (status != SERIAL_OK) return status;

    return Serial_SendData(dev, bytes, bytes_len);
}

Serial_Status Serial_ReceiveFromBuffer(Serial_Device_t *dev, Serial_RxData_t *data, uint32_t timeout)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !dev || !data) return SERIAL_ERR_INIT;

    if (rtos_ops->SemaphoreTake(dev->rx_buffer.sem, timeout)) {
        RB_Status rb_status = RingBuffer_Read(&dev->rx_buffer, data);
        if (rb_status != RB_OK) {
            rtos_ops->SemaphoreGive(dev->rx_buffer.sem);
            return (rb_status == RB_ERROR_BUFFER_EMPTY) ? SERIAL_ERR_NO_DATA : SERIAL_ERR_BUFFER_FULL;
        }
        return SERIAL_OK;
    }
    return SERIAL_ERR_NO_DATA;
}

Serial_Status Serial_AddFrameToQueue(RS485_Frame_t *frame)
{
    if (!frame || rs485_tx_queue.count >= RS485_TX_QUEUE_SIZE) {
        return SERIAL_ERR_BUFFER_FULL;
    }

    // 复制帧到队列
    RS485_Frame_t *queued_frame = &rs485_tx_queue.frames[rs485_tx_queue.tail];
    *queued_frame = *frame;
    queued_frame->info = rs485_tx_queue.info_buffers[rs485_tx_queue.tail];
    memcpy(queued_frame->info, frame->info, frame->info_len);

    rs485_tx_queue.tail = (rs485_tx_queue.tail + 1) % RS485_TX_QUEUE_SIZE;
    rs485_tx_queue.count++;

    return SERIAL_OK;
}

void Serial_PollSendRS485(Serial_Device_t *dev)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !dev || dev->mode != RS485_MODE || rs485_tx_queue.count == 0) {
        return;
    }

    RS485_Frame_t *frame = &rs485_tx_queue.frames[rs485_tx_queue.head];
    Serial_SendFrame(dev, frame);

    rs485_tx_queue.head = (rs485_tx_queue.head + 1) % RS485_TX_QUEUE_SIZE;
    rs485_tx_queue.count--;

    // 帧间静默时间
    rtos_ops->Delay(dev->silent_ticks);
}

int HandleFrame_7E(void *dev_ptr, Serial_RxData_t *rx_data, uint8_t byte,
                   uint32_t *rx_index, uint8_t *state, uint8_t *expected_length,
                   void *xHigherPriorityTaskWoken)
{
    Serial_Device_t *dev = (Serial_Device_t *)dev_ptr;
    static uint8_t temp_buffer[208];

    switch (*state) {
        case 0:
            temp_buffer[0] = byte;
            rx_data->is_rs485 = 1;
            *rx_index = 1;
            *state = 1;
            return 1;
        case 1:
            temp_buffer[(*rx_index)++] = byte;
            *state = 2;
            return 1;
        case 2:
            temp_buffer[(*rx_index)++] = byte;
            *state = 3;
            return 1;
        case 3:
            temp_buffer[(*rx_index)++] = byte;
            *state = 4;
            return 1;
        case 4:
            temp_buffer[(*rx_index)++] = byte;
            *state = 5;
            return 1;
        case 5:
            temp_buffer[(*rx_index)++] = byte;
            *expected_length = byte;
            if (*expected_length < 8 || *expected_length > 208) {
                Serial_LogError(ERROR_FRAME, dev->instance);
                *rx_index = 0;
                *state = 0;
                *expected_length = 0;
                memset(temp_buffer, 0, sizeof(temp_buffer));
            } else {
                *state = 6;
            }
            return 1;
        case 6:
            temp_buffer[(*rx_index)++] = byte;
            if (*rx_index >= *expected_length - 2) {
                *state = 7;
            }
            return 1;
        case 7:
            temp_buffer[(*rx_index)++] = byte;
            if (*rx_index >= *expected_length) {
                uint8_t info_buffer[200];
                Serial_Status status = RS485_FrameFromBytes(temp_buffer, &rx_data->rs485_frame, info_buffer, sizeof(info_buffer));
                if (status == SERIAL_OK) {
                    RingBuffer_WriteFromISR(&dev->rx_buffer, rx_data, xHigherPriorityTaskWoken);
                } else {
                    Serial_LogError(status == SERIAL_ERR_CRC ? ERROR_CRC : ERROR_FRAME, dev->instance);
                }
                *rx_index = 0;
                *state = 0;
                *expected_length = 0;
                memset(temp_buffer, 0, sizeof(temp_buffer));
            }
            return 1;
    }
    return 0;
}

int HandleFrame_F6(void *dev_ptr, Serial_RxData_t *rx_data, uint8_t byte,
                   uint32_t *rx_index, uint8_t *state, uint8_t *expected_length,
                   void *xHigherPriorityTaskWoken)
{
    return HandleFrame_7E(dev_ptr, rx_data, byte, rx_index, state, expected_length, xHigherPriorityTaskWoken);
}
Serial_RxData_t rx_data = {0};
uint32_t rx_index = 0;
uint8_t state = 0;
uint8_t expected_length = 0;
void Serial_IRQHandler(Serial_Device_t *dev)
{

    void *xHigherPriorityTaskWoken = NULL;

    if (USART_GetITStatus(dev->instance, USART_IT_RXNE) != RESET) {
        uint8_t byte = USART_ReceiveData(dev->instance);
        USART_ClearITPendingBit(dev->instance, USART_IT_RXNE);

        if (dev->mode == RS485_MODE) {
            if (state == 0) {
                uint8_t handler_found = 0;
                for (uint8_t i = 0; i < frame_handler_count; i++) {
                    if (frame_handlers[i].header == byte) {
                        if (frame_handlers[i].handler(dev, &rx_data, byte, &rx_index, &state,
                                                      &expected_length, xHigherPriorityTaskWoken)) {
                            handler_found = 1;
                        }
                        break;
                    }
                }
                if (!handler_found) {
                    Serial_LogError(ERROR_FRAME, dev->instance);
                }
            } else {
                for (uint8_t i = 0; i < frame_handler_count; i++) {
                    if (frame_handlers[i].header == rx_data.rs485_frame.sof) {
                        frame_handlers[i].handler(dev, &rx_data, byte, &rx_index, &state,
                                                  &expected_length, xHigherPriorityTaskWoken);
                        break;
                    }
                }
            }
        } else {
            switch (state) {
                case 0:
                    if (byte == 0xAA) {
                        rx_data.is_rs485 = 0;
                        rx_data.uart_data.data[0] = byte;
                        rx_data.uart_data.length = 1;
                        rx_index = 1;
                        state = 1;
                    } else {
                        Serial_LogError(ERROR_FRAME, dev->instance);
                    }
                    break;
                case 1:
                    if (byte <= 30) {
                        rx_data.uart_data.data[rx_index++] = byte;
                        rx_data.uart_data.length = rx_index;
                        expected_length = byte;
                        state = 2;
                    } else {
                        Serial_LogError(ERROR_FRAME, dev->instance);
                        rx_index = 0;
                        state = 0;
                        memset(&rx_data, 0, sizeof(Serial_RxData_t));
                    }
                    break;
                case 2:
                    if (rx_index < expected_length + 2) {
                        rx_data.uart_data.data[rx_index++] = byte;
                        rx_data.uart_data.length = rx_index;
                    }
                    if (rx_index >= expected_length + 2) {
                        state = 3;
                    }
                    break;
                case 3:
                    rx_data.uart_data.data[rx_index++] = byte;
                    rx_data.uart_data.length = rx_index;
                    if (byte == 0x55 && rx_index == expected_length + 3) {
                        RingBuffer_WriteFromISR(&dev->rx_buffer, &rx_data, xHigherPriorityTaskWoken);
                    } else {
                        Serial_LogError(ERROR_FRAME, dev->instance);
                    }
                    rx_index = 0;
                    state = 0;
                    expected_length = 0;
                    memset(&rx_data, 0, sizeof(Serial_RxData_t));
                    break;
            }
        }
    }

    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (rtos_ops) {
        rtos_ops->YieldFromISR(xHigherPriorityTaskWoken);
    }
}

void Serial_Silent_IRQHandler(Serial_Device_t *dev)
{
    extern Serial_RxData_t rx_data;
    extern uint32_t rx_index;
    extern uint8_t state;
    extern uint8_t expected_length;

    rx_index = 0;
    state = 0;
    expected_length = 0;
    memset(&rx_data, 0, sizeof(Serial_RxData_t));
    Serial_LogError(ERROR_FRAME, dev->instance);
}

const Serial_Ops_t Serial_Operations = {
    .Init = Serial_Init,
    .Deinit = Serial_Deinit,
    .SendData = Serial_SendData,
    .SendFrame = Serial_SendFrame,
    .ReceiveFromBuffer = Serial_ReceiveFromBuffer,
    .GetErrorLog = Serial_GetErrorLog,
    .AddFrameToQueue = Serial_AddFrameToQueue,
    .PollSendRS485 = Serial_PollSendRS485
};

void USART1_IRQHandler(void) {
    extern Serial_Device_t RS485_Device;
    Serial_IRQHandler(&RS485_Device);
}

void USART2_IRQHandler(void) {
    extern Serial_Device_t UART_Device;
    Serial_IRQHandler(&UART_Device);
}

// 3. 定义RS485设备
Serial_Device_t RS485_Device = {
    .instance = USART1,
    .tx_port = GPIOA,
    .tx_pin = GPIO_Pin_9,
    .rx_port = GPIOA,
    .rx_pin = GPIO_Pin_10,
    .de_port = GPIOA,
    .de_pin = GPIO_Pin_8,
    .baudrate = 115200,
    .af = GPIO_AF_USART1,
    .irqn = USART1_IRQn,
    .mode = RS485_MODE,
    .silent_ticks = 0,
    .rx_buffer = {0}
};

// 4. 定义UART设备
Serial_Device_t UART_Device = {
    .instance = USART2,
    .tx_port = GPIOA,
    .tx_pin = GPIO_Pin_2,
    .rx_port = GPIOA,
    .rx_pin = GPIO_Pin_3,
    .de_port = NULL,
    .de_pin = 0,
    .baudrate = 9600,
    .af = GPIO_AF_USART2,
    .irqn = USART2_IRQn,
    .mode = UART_MODE,
    .silent_ticks = 0,
    .rx_buffer = {0}
};

// 5. 初始化设备



/*
 * 示例用法（以FreeRTOS为例）：
 * 1. 定义FreeRTOS实现
 * #include "FreeRTOS.h"
 * #include "semphr.h"
 * #include "task.h"
 *
 * static void* FreeRTOS_SemaphoreCreate(void) {
 *     return xSemaphoreCreateMutex();
 * }
 *
 * static void FreeRTOS_SemaphoreDelete(void *sem) {
 *     vSemaphoreDelete(sem);
 * }
 *
 * static int FreeRTOS_SemaphoreTake(void *sem, uint32_t timeout) {
 *     return xSemaphoreTake(sem, timeout) == pdTRUE ? 1 : 0;
 * }
 *
 * static void FreeRTOS_SemaphoreGive(void *sem) {
 *     xSemaphoreGive(sem);
 * }
 *
 * static int FreeRTOS_SemaphoreGiveFromISR(void *sem, void *xHigherPriorityTaskWoken) {
 *     BaseType_t woken = pdFALSE;
 *     int ret = xSemaphoreGiveFromISR(sem, &woken);
 *     *(BaseType_t *)xHigherPriorityTaskWoken = woken;
 *     return ret == pdTRUE ? 1 : 0;
 * }
 *
 * static void FreeRTOS_YieldFromISR(void *xHigherPriorityTaskWoken) {
 *     portYIELD_FROM_ISR(*(BaseType_t *)xHigherPriorityTaskWoken);
 * }
 *
 * static uint32_t FreeRTOS_GetTickCount(void) {
 *     return xTaskGetTickCount();
 * }
 *
 * static void FreeRTOS_Delay(uint32_t ticks) {
 *     vTaskDelay(ticks);
 * }
 *
 * static void* FreeRTOS_TaskCreate(void (*task_func)(void*), const char* name, uint32_t stack_size, void* param, uint32_t priority) {
 *     TaskHandle_t task;
 *     xTaskCreate(task_func, name, stack_size, param, priority, &task);
 *     return task;
 * }
 *
 * static void FreeRTOS_TaskDelete(void* task) {
 *     vTaskDelete(task);
 * }
 *
 * const RTOS_Ops_t FreeRTOS_Ops = {
 *     .SemaphoreCreate = FreeRTOS_SemaphoreCreate,
 *     .SemaphoreDelete = FreeRTOS_SemaphoreDelete,
 *     .SemaphoreTake = FreeRTOS_SemaphoreTake,
 *     .SemaphoreGive = FreeRTOS_SemaphoreGive,
 *     .SemaphoreGiveFromISR = FreeRTOS_SemaphoreGiveFromISR,
 *     .YieldFromISR = FreeRTOS_YieldFromISR,
 *     .GetTickCount = FreeRTOS_GetTickCount,
 *     .Delay = FreeRTOS_Delay,
 *     .TaskCreate = FreeRTOS_TaskCreate,
 *     .TaskDelete = FreeRTOS_TaskDelete
 * };
 *
 * 2. 初始化RTOS抽象层
 * RTOS_SetOps(&FreeRTOS_Ops);
 *
 * 3. 定义RS485设备
 * Serial_Device_t RS485_Device = {
 *     .instance = USART1,
 *     .tx_port = GPIOA,
 *     .tx_pin = GPIO_Pin_9,
 *     .rx_port = GPIOA,
 *     .rx_pin = GPIO_Pin_10,
 *     .de_port = GPIOA,
 *     .de_pin = GPIO_Pin_8,
 *     .baudrate = 115200,
 *     .af = GPIO_AF_USART1,
 *     .irqn = USART1_IRQn,
 *     .mode = RS485_MODE,
 *     .silent_ticks = 0,
 *     .rx_buffer = {0}
 * };
 *
 * 4. 定义UART设备
 * Serial_Device_t UART_Device = {
 *     .instance = USART2,
 *     .tx_port = GPIOA,
 *     .tx_pin = GPIO_Pin_2,
 *     .rx_port = GPIOA,
 *     .rx_pin = GPIO_Pin_3,
 *     .de_port = NULL,
 *     .de_pin = 0,
 *     .baudrate = 9600,
 *     .af = GPIO_AF_USART2,
 *     .irqn = USART2_IRQn,
 *     .mode = UART_MODE,
 *     .silent_ticks = 0,
 *     .rx_buffer = {0}
 * };
 *
 * 5. 初始化设备
 * Serial_Operations.Init(&RS485_Device);
 * Serial_Operations.Init(&UART_Device);
 *
 * 6. 定义中断服务函数
 * void USART1_IRQHandler(void) {
 *     extern Serial_Device_t RS485_Device;
 *     Serial_IRQHandler(&RS485_Device);
 * }
 *
 * void USART2_IRQHandler(void) {
 *     extern Serial_Device_t UART_Device;
 *     Serial_IRQHandler(&UART_Device);
 * }
 *
 * 7. 添加RS485帧到发送队列
 * uint8_t info_data1[] = {0x01, 0x02};
 * RS485_Frame_t frame1 = {
 *     .sof = 0x7E,
 *     .addr1 = 0x01,
 *     .addr2 = 0xFF,
 *     .cmd = 0x03,
 *     .cmd_sub = 0x00,
 *     .length = 0,
 *     .info = info_data1,
 *     .info_len = sizeof(info_data1),
 *     .crc = 0
 * };
 * Serial_Operations.AddFrameToQueue(&frame1);
 *
 * 8. 轮询发送任务
 * void RS485_PollTask(void *pvParameters) {
 *     Serial_Device_t *dev = (Serial_Device_t *)pvParameters;
 *     while (1) {
 *         Serial_Operations.PollSendRS485(dev);
 *         vTaskDelay(100); // 每100ms轮询一次
 *     }
 * }
 * xTaskCreate(RS485_PollTask, "RS485_Poll", 256, &RS485_Device, 1, NULL);
 *
 * 9. 接收数据（任务中）
 * void Serial_RxTask(void *pvParameters) {
 *     Serial_Device_t *dev = (Serial_Device_t *)pvParameters;
 *     Serial_RxData_t rx_data;
 *     while (1) {
 *         if (Serial_Operations.ReceiveFromBuffer(dev, &rx_data, portMAX_DELAY) == SERIAL_OK) {
 *             if (rx_data.is_rs485) {
 *                 RS485_Frame_t *frame = &rx_data.rs485_frame;
 *                 // 自行检查frame->addr2是否匹配目标地址
 *             } else {
 *                 // 处理UART数据
 *             }
 *         }
 *     }
 * }
 *
 * 10. 读取错误日志（任务中）
 * void Error_Log_Task(void *pvParameters) {
 *     Serial_ErrorLog_t log;
 *     while (1) {
 *         if (Serial_Operations.GetErrorLog(&log, portMAX_DELAY) == SERIAL_OK) {
 *             // 处理 log
 *         }
 *     }
 * }
 *
 * 11. 释放资源
 * Serial_Operations.Deinit(&RS485_Device);
 * Serial_Operations.Deinit(&UART_Device);
 */

