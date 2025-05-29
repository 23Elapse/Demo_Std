/*
 * @Author: 23Elapse userszy@163.com
 * @Date: 2025-05-02 21:33:53
 * @LastEditors: 23Elapse userszy@163.com
 * @LastEditTime: 2025-05-25 19:28:03
 * @FilePath: \Demo\Middlewares\Src\log_system.c
 * @Description: 日志系统实现
 *
 * Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved.
 */
#include "log_system.h"
#include "ring_buffer.h"
#include "rtos_abstraction.h"
#include "pch.h"
#include "stdarg.h"
#define LOG_BUFFER_SIZE 128
static RingBuffer_t log_buffer;
static uint8_t log_data[LOG_BUFFER_SIZE][128]; // 假设每条日志最大 128 字节

void Log_Init(void)
{
    if (RingBuffer_Init(&log_buffer, LOG_BUFFER_SIZE, sizeof(uint8_t *)) != RB_OK)
    {
        return;
    }
}

void Log_ToBuffer(Log_Level_t level, const char *message)
{
    for (uint8_t i = 0; i < LOG_BUFFER_SIZE; i++)
    {
        if (RingBuffer_IsAvailable(&log_buffer))
        {
            uint8_t *log_ptr = log_data[i];
            snprintf((char *)log_ptr, 128, "[%d] %s", level, message);
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
    if (level >= LOG_LEVEL_INFO)
    {
        printf("%s\n", buffer); // 输出到控制台，仅限警告和错误
    }
}
