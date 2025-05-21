#include "log_system.h"
#include "ring_buffer.h"
#include "rtos_abstraction.h"
#include "pch.h"
#include "stdarg.h"
#define LOG_BUFFER_SIZE 32
static RingBuffer_t log_buffer;
static uint8_t log_data[LOG_BUFFER_SIZE][64]; // 假设每条日志最大 64 字节

void Log_Init(void)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops)
        return;
    if (RingBuffer_Init(&log_buffer, LOG_BUFFER_SIZE, sizeof(uint8_t *)) != RB_OK)
    {
        return;
    }
}

void Log_ToBuffer(Log_Level_t level, const char *message)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops)
        return;

    for (uint8_t i = 0; i < LOG_BUFFER_SIZE; i++)
    {
        if (RingBuffer_IsAvailable(&log_buffer))
        {
            uint8_t *log_ptr = log_data[i];
            snprintf((char *)log_ptr, 64, "[%d] %s", level, message);
            RingBuffer_Write(&log_buffer, &log_ptr);
            break;
        }
    }
}

void Log_Message(Log_Level_t level, const char *format, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    Log_ToBuffer(level, buffer);
    if (level >= LOG_LEVEL_WARNING)
    {
        printf("%s\n", buffer); // 输出到控制台，仅限警告和错误
    }
}
